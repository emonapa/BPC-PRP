#include "loops/robot_movement.hpp"
#include "algorithms/lidar_alg.hpp"
#include <opencv2/opencv.hpp> // PŘIDÁNO PRO KAMERU
#include <algorithm>
#include <cmath>

namespace loops {

MovementLoop::MovementLoop() : rclcpp::Node("robot_movement_node"),
    wall_pid_(12.0f, 0.0f, 25.0f), // PID pro držení se zdi
    turn_pid_(20.0f, 0.0f, 10.0f)   // PID pro přesné otáčení na úhel
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
    RCLCPP_INFO(this->get_logger(), "Som v CORRIDOR_FOLLOWING, F: %f", results.front);
    // Hranice pre rozpoznanie steny/otvoreného priestoru
    // V 40cm bludisku je stred chodby 20cm od steny. 
    // Ak L > 0.35, znamená to, že tam stena určite nie je.
    bool is_left_open = (L > 0.38f); 
    bool is_right_open = (R > 0.38f);
    
    // Znížime hranicu detekcie steny pred nami. 
    // Ak začne točiť uprostred chodby, skús 0.18f namiesto 0.22f.
    bool is_front_blocked = (F < 0.25f); 

    // --- LOGIKA ROZHODOVÁNÍ O SMĚRU ---
    if (is_front_blocked) {
        set_speed(127, 127); // STOP
        float current_yaw = imu_integrator_.getYaw();
        
        if (is_right_open) {
            target_yaw_ = normalize_angle(current_yaw - (M_PI / 2.0f));
            // target_yaw_ = current_yaw - (M_PI / 2.0f);
            RCLCPP_INFO(this->get_logger(), "Zákruta: DOPRAVA");
        } else if (is_left_open) {
            target_yaw_ = normalize_angle(current_yaw + (M_PI / 2.0f));
            // target_yaw_ = current_yaw + (M_PI / 2.0f);
            RCLCPP_INFO(this->get_logger(), "Zákruta: DOLEVA");
        } else {
            target_yaw_ = normalize_angle(current_yaw + M_PI); // Otočka o 180°
            // target_yaw_ = current_yaw + M_PI; // Slepá ulička
            RCLCPP_INFO(this->get_logger(), "Slepá ulica: OTOČKA");
        }
        
        current_state_ = MazeState::TURNING;
        return; 
    }

    // --- JAZDA CHODBOU (Udržiavanie stredu) ---
    float steering = 0.0f;
    int base_speed = 165;

    // KRITICKÁ ÚPRAVA: Ak je jedna strana otvorená, nesmieš počítať (L - R)
    if (is_left_open && is_right_open) {
        steering = 0.0f; // Sme v križovatke, drž kolesá rovno
    } else if (is_left_open) {
        float error_right = 0.20f - R; 
        steering = wall_pid_.step(error_right, 0.05f);
    } else if (is_right_open) {
        // Vidím len ľavú stenu, drž sa od nej 20cm
        float error_left = L - 0.20f;
        steering = wall_pid_.step(error_left, 0.05f);
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

    if (std::abs(yaw_error) < 0.08f) { // Tolerancia
        set_speed(127, 127);
        
        // Statické čítač (jednoduchý trik na pauzu bez blokovania vlákna)
        static int wait_cycles = 0;
        if (wait_cycles < 5) { // Počkaj cca 250ms (5 * 50ms)
            wait_cycles++;
            return;
        }
        wait_cycles = 0;

        current_state_ = MazeState::CORRIDOR_FOLLOWING;
        turn_pid_.reset();
        break;
    }

    float turn_speed = turn_pid_.step(yaw_error, 0.05f);
    int correction = std::clamp(static_cast<int>(turn_speed), -60, 60); // Zvýšený rozsah

    // Jemnejší deadband - ak je chyba malá, zmenši aj minimálnu silu
    int min_force = (std::abs(yaw_error) < 0.2f) ? 15 : 25; 
    if (std::abs(correction) < min_force) {
        correction = (yaw_error > 0) ? min_force : -min_force;
    }

    set_speed(127 - correction, 127 + correction);
    break;
}
    }
}
}