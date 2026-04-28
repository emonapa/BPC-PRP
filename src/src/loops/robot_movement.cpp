#include "loops/robot_movement.hpp"
#include "algorithms/lidar_alg.hpp"
#include <opencv2/opencv.hpp> // PŘIDÁNO PRO KAMERU
#include <algorithm>
#include <cmath>

#define GET_TARGET_LEFT(yaw)   normalize_angle((yaw) + (M_PI / 2.0f))
#define GET_TARGET_RIGHT(yaw)  normalize_angle((yaw) - (M_PI / 2.0f))
#define GET_TARGET_AROUND(yaw) normalize_angle((yaw) + M_PI)

namespace loops {

MovementLoop::MovementLoop() : rclcpp::Node("robot_movement_node"),
    wall_pid_(15.0f, 0.0f, 25.0f), // PID pro držení se zdi
    turn_pid_(10.0f, 0.0f, 10.0f)   // PID pro přesné otáčení na úhel
{
    // Odběr LiDARu
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        Topic::get_lidar, 10, std::bind(&MovementLoop::lidar_callback, this, std::placeholders::_1));

    // Odběr IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/bpc_prp_robot/imu", 10, std::bind(&MovementLoop::imu_callback, this, std::placeholders::_1));

    // Publikace motorů
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);

     // Odběr Kamery (PŘIDÁNO)
    camera_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
         "/bpc_prp_robot/camera/compressed", 10, std::bind(&MovementLoop::camera_callback, this, std::placeholders::_1));
    // Hlavní smyčka
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&MovementLoop::timer_callback, this));

     turnToExit = -1;
     turnToTreasure = -1;

    RCLCPP_INFO(this->get_logger(), "Mozek robota aktivovan. Zacinam kalibraci IMU...");
}

void MovementLoop::set_speed(int left, int right) {
    left = std::clamp(left, 0, 255);
    right = std::clamp(right, 0, 255);
    std_msgs::msg::UInt8MultiArray speed_msg;
    speed_msg.data = { static_cast<uint8_t>(left), static_cast<uint8_t>(right) };
    motor_pub_->publish(speed_msg);
}

void MovementLoop::camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        // Převedení zkomprimovaných dat (JPEG) do OpenCV matice
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        if (frame.empty()) return;

        // Zavoláme náš detektor
        auto markers = aruco_detector_.detect(frame);

        // Uložíme si je, abychom s nimi mohli pracovat v timer_callbacku (např. na křižovatkách)
        last_markers_ = markers;

        // Pro ladění: Pokud robot nějakou značku uvidí, vypíše ji do terminálu
        if (!markers.empty()) {
            if(markers[0].id < 10) turnToExit = markers[0].id;
            else turnToTreasure = markers[0].id;
            RCLCPP_INFO(this->get_logger(), "Vidim ArUco znacku! ID: %d", markers[0].id);
        }

    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what());
    }
}

void MovementLoop::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
}

// Zde se jen na pozadí počítá úhel z gyroskopu
void MovementLoop::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time now = msg->header.stamp;
    if (first_imu_) {
        last_imu_time_ = now;
        first_imu_ = false;
        return;
    }
    double dt = (now - last_imu_time_).seconds();
    last_imu_time_ = now;

    if (dt <= 0.0 || dt > 0.5) return;

    float gyro_z = msg->angular_velocity.z;

    if (current_state_ == MazeState::CALIBRATION) {
        gyro_calibration_samples_.push_back(gyro_z);
        if (gyro_calibration_samples_.size() >= 100) { // Cca 2 sekundy
            imu_integrator_.setCalibration(gyro_calibration_samples_);
            current_state_ = MazeState::CORRIDOR_FOLLOWING;
            RCLCPP_INFO(this->get_logger(), "Kalibrace HOTOVA! Jedu do bludiste.");
        }
    } else {
        imu_integrator_.update(gyro_z, dt);
    }
}

float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void MovementLoop::timer_callback() {
    if (current_state_ == MazeState::CALIBRATION) {
        set_speed(127, 127);
        return;
    }

    if (!latest_scan_) return;

    algorithms::LidarFilter filter;
    auto results = filter.apply_filter(latest_scan_->ranges, latest_scan_->angle_min, latest_scan_->angle_max);

    switch (current_state_) {

case MazeState::CORRIDOR_FOLLOWING: {
    float L = results.left;
    float R = results.right;
    float F = results.front;
    
    // Hranice pre rozpoznanie steny/otvoreného priestoru
    // V 40cm bludisku je stred chodby 20cm od steny. 
    // Ak L > 0.35, znamená to, že tam stena určite nie je.
    bool is_left_open = (L > 0.38f); 
    bool is_right_open = (R > 0.38f);
    bool long_front_detection =(F < 0.45f); 
    // Znížime hranicu detekcie steny pred nami. 
    // Ak začne točiť uprostred chodby, skús 0.18f namiesto 0.22f.
    bool is_front_blocked = (F < 0.25f); 
    float current_yaw = imu_integrator_.getYaw();
    int active_cmd = -1;

    if (turnToTreasure != -1) {
        active_cmd = turnToTreasure - 10;
    } else if (turnToExit != -1) {
        active_cmd = turnToExit;
    }

    bool side_opened = ((active_cmd == 1 || active_cmd == 11) && is_left_open) ||
                       ((active_cmd == 2 || active_cmd == 12) && is_right_open);

    if (side_opened && !junction_detected) {
        junction_detected = true;
        detection_time = std::chrono::steady_clock::now();
    }

    // if (junction_detected) {
    //     auto now = std::chrono::steady_clock::now();
    //     auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - detection_time).count();

    //     // Počkáme napr. 300ms (treba vyladiť podľa rýchlosti robota), kým reálne začneme točiť
    //     if (elapsed > 600) { 
    //         if (active_cmd == 1 || active_cmd == 11) target_yaw_ = GET_TARGET_LEFT(current_yaw);
    //         else target_yaw_ = GET_TARGET_RIGHT(current_yaw);
            
    //         turnToTreasure = -1; turnToExit = -1;
    //         junction_detected = false; // Reset pre ďalšiu križovatku
    //         current_state_ = MazeState::TURNING;
    //         break;
    //     }
    // }

      
    // --- LOGIKA ROZHODOVÁNÍ O SMĚRU ---
    if (long_front_detection) {
        if(is_front_blocked){
        // set_speed(127, 127); // STOP
      
        if(is_left_open && is_right_open){
        RCLCPP_INFO(this->get_logger(), "Krizovatka: l r open %d",active_cmd);
        if(active_cmd == 1) target_yaw_ = GET_TARGET_LEFT(current_yaw);
        else if (active_cmd == 2 ) target_yaw_ = GET_TARGET_RIGHT(current_yaw);
        else  target_yaw_ = GET_TARGET_RIGHT(current_yaw);
        turnToTreasure = -1; turnToExit = -1;
        }
        else if (is_right_open) {
            target_yaw_ = GET_TARGET_RIGHT(current_yaw);
            // target_yaw_ = current_yaw - (M_PI / 2.0f);
            RCLCPP_INFO(this->get_logger(), "Zákruta: DOPRAVA");
        } else if (is_left_open) {
            target_yaw_ = GET_TARGET_LEFT(current_yaw);
            // target_yaw_ = current_yaw + (M_PI / 2.0f);
            RCLCPP_INFO(this->get_logger(), "Zákruta: DOLEVA");
        } else {
            target_yaw_ = GET_TARGET_AROUND(current_yaw);
            // target_yaw_ = current_yaw + M_PI; // Slepá ulička
            
            RCLCPP_INFO(this->get_logger(), "Slepá ulica: OTOČKA");
        }
        
        current_state_ = MazeState::TURNING;
        break; 
        }
        float steering = 0.0f;
        int base_speed = 145;
        set_speed(base_speed - static_cast<int>(steering), base_speed + static_cast<int>(steering));
        break;
    }

    // --- JAZDA CHODBOU (Udržiavanie stredu) ---
    float steering = 0.0f;
    int base_speed = 145;

    // KRITICKÁ ÚPRAVA: Ak je jedna strana otvorená, nesmieš počítať (L - R)
    if (is_left_open && is_right_open) {
           RCLCPP_INFO(this->get_logger(), "Krizovatka: L R open %d",active_cmd);
        if(active_cmd == 1) {
            target_yaw_ = GET_TARGET_LEFT(current_yaw);
            turnToTreasure = -1; turnToExit = -1; 
            current_state_ = MazeState::TURNING;
            break;
        }else if (active_cmd == 2 ) {
            target_yaw_ = GET_TARGET_RIGHT(current_yaw);
            turnToTreasure = -1; turnToExit = -1;
            current_state_ = MazeState::TURNING;
            break;
        }
        steering = 0.0f; // Sme v križovatke, drž kolesá rovno
      
    } else if (is_left_open) {
         RCLCPP_INFO(this->get_logger(), "Krizovatka: L F open %d",active_cmd);
        if(active_cmd == 1) {   
            target_yaw_ = GET_TARGET_LEFT(current_yaw);
            current_state_ = MazeState::TURNING;
            turnToTreasure = -1; turnToExit = -1; 
            break;
        }
        else{
            float error_right = 0.20f - R; 
            steering = wall_pid_.step(error_right, 0.05f);
        }
       
    } else if (is_right_open) {
         RCLCPP_INFO(this->get_logger(), "Krizovatka: R F open %d",active_cmd);
        if (active_cmd == 2 ){ 
            target_yaw_ = GET_TARGET_RIGHT(current_yaw);
            current_state_ = MazeState::TURNING;
            turnToTreasure = -1; turnToExit = -1;   
            break;
        }
        else{
        // Vidím len ľavú stenu, drž sa od nej 20cm
        float error_left = L - 0.20f;
        steering = wall_pid_.step(error_left, 0.05f);
        }
        
    } else {
        // Klasická chodba - obe steny sú blízko
        float error_center = L - R;
        steering = wall_pid_.step(error_center, 0.05f);
    }

    set_speed(base_speed - static_cast<int>(steering), base_speed + static_cast<int>(steering));
    break;
}

    case MazeState::TURNING: {
    float current_yaw = imu_integrator_.getYaw();
    float yaw_error = normalize_angle(target_yaw_ - current_yaw);

    RCLCPP_INFO(this->get_logger(), "Ciel: %.2f, Aktual: %.2f, Chyba: %.2f", target_yaw_, current_yaw, yaw_error);

    // 1. Väčšia tolerancia (0.15 rad je cca 8 stupňov)
    if (std::abs(yaw_error) < 0.15f) {
        set_speed(127, 127); 
        turn_pid_.reset();
        post_turn_start_time_ = std::chrono::steady_clock::now();
        current_state_ = MazeState::POST_TURN_MOVE;
          // current_state_ = MazeState::CORRIDOR_FOLLOWING;
        
        break;
    }

    float turn_speed = turn_pid_.step(yaw_error, 0.05f);
    int correction = std::clamp(static_cast<int>(turn_speed), -50, 50);

    // 2. LOGICKÁ OPRAVA DEADBANDU:
    // Použi min_turn IBA ak sme ďaleko od cieľa. 
    // Ak sme bližšie ako 0.4 rad (cca 23 stupňov), nechaj pracovať len čistý PID.
    int min_turn = 20; 
    if (std::abs(yaw_error) > 0.40f) {
        if (std::abs(correction) < min_turn) {
            correction = (yaw_error > 0) ? min_turn : -min_turn;
        }
    }

    set_speed(127 - correction, 127 + correction);
    break;
}

        case MazeState::POST_TURN_MOVE: {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - post_turn_start_time_).count();
            
            // Získaj aktuálnu vzdialenosť spredu z výsledkov filtra
            float F = results.front; 

            // Ak uplynulo 500ms ALEBO je stena pred nami bližšie ako 20cm, ukonči slepý pohyb
            if (elapsed > 1500 || F < 0.20f) { 
                RCLCPP_INFO(this->get_logger(), "POST_TURN ukončený (čas alebo stena).");
                current_state_ = MazeState::CORRIDOR_FOLLOWING;
            } else {
                set_speed(145, 145); 
            }
            break;
        }
    }
}
}