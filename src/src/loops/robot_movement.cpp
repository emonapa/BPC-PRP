#include "loops/robot_movement.hpp"
#include "algorithms/lidar_alg.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp> // PŘIDÁNO PRO KAMERU

using namespace std;

namespace loops {

MovementLoop::MovementLoop() : rclcpp::Node("robot_movement_node"),
    wall_pid_(30.0f, 0.1f, 15.0f), // PID pro držení se zdi
    turn_pid_(10.0f, 0.5f, 15.0f)    // PID pro přesné otáčení na úhel
{
    // Odběr LiDARu
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        Topic::get_lidar, 10, std::bind(&MovementLoop::lidar_callback, this, std::placeholders::_1));

    // Odběr IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/bpc_prp_robot/imu", 10, std::bind(&MovementLoop::imu_callback, this, std::placeholders::_1));

    // Odběr Kamery (PŘIDÁNO)
    camera_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/bpc_prp_robot/camera/compressed", 10, std::bind(&MovementLoop::camera_callback, this, std::placeholders::_1));

    // Publikace motorů
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);

    // Hlavní smyčka
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&MovementLoop::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Mozek robota aktivovan. Zacinam kalibraci IMU...");
}

// ---------------------------------------------------------
// NOVÁ FUNKCE: Zpracování obrazu z kamery na pozadí
// ---------------------------------------------------------
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
// ---------------------------------------------------------

void MovementLoop::set_speed(int left, int right) {
    left = std::clamp(left, 0, 255);
    right = std::clamp(right, 0, 255);
    std_msgs::msg::UInt8MultiArray speed_msg;
    speed_msg.data = { static_cast<uint8_t>(left), static_cast<uint8_t>(right) };
    motor_pub_->publish(speed_msg);
}

void MovementLoop::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
}

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
        if (gyro_calibration_samples_.size() >= 100) {
            imu_integrator_.setCalibration(gyro_calibration_samples_);
            current_state_ = MazeState::CORRIDOR_FOLLOWING;
            RCLCPP_INFO(this->get_logger(), "Kalibrace HOTOVA! Jedu do bludiste.");
        }
    } else {
        imu_integrator_.update(gyro_z, dt);
    }
}

void MovementLoop::timer_callback() {
    if (current_state_ == MazeState::CALIBRATION) {
        set_speed(127, 127); // Během kalibrace MUSÍ stát
        return;
    }

    if (!latest_scan_) return;

    algorithms::LidarFilter filter;
    auto results = filter.apply_filter(latest_scan_->ranges, latest_scan_->angle_min, latest_scan_->angle_max);

    // Výpis pro debug
    //printf("front = %f, left = %f, right = %f, back = %f\n", results.front, results.left, results.right, results.back);

    // --- STAVOVÝ AUTOMAT ---
    switch (current_state_) {

        case MazeState::CORRIDOR_FOLLOWING: {
            // Tyto oříznuté hodnoty použijeme POUZE pro výpočet chyby pro PID
            float L = std::min(results.left, 0.5f);
            float R = std::min(results.right, 0.5f);
            float error = L - R;

            int went_left = 0;
            int went_right = 0;
            // Zjistíme, jestli nejsme v rohu (překážka vpředu)
            if (results.front <= 0.27f) {
                set_speed(127, 127); // Zastavíme před rohem
                current_state_ = MazeState::TURNING;
                float current_yaw = imu_integrator_.getYaw();

                // TADY SE DÍVÁME NA NEOŘÍZNUTÁ DATA
                if (results.right >= 0.5f) {
                    target_yaw_ = current_yaw - (M_PI / 2.0f) * 0.8; // 90 stupňů doprava
                    //RCLCPP_INFO(this->get_logger(), "Roh! Budu tocit DOPRAVA.");
                } else if (results.left >= 0.5f){
                    target_yaw_ = current_yaw + (M_PI / 2.0f) * 0.8; // 90 stupňů doleva
                    //RCLCPP_INFO(this->get_logger(), "Roh! Budu tocit DOLEVA.");
                } else {
                    target_yaw_ = current_yaw + (M_PI / 2.0f) * 0.8; // Otočka čelem vzad
                    //RCLCPP_INFO(this->get_logger(), "Slepa ulicka! Tocim DOZADU.");
                }

                // OPRAVA 1: Tady vyčistíme paměť PID regulátoru pro otáčení - pouze JEDNOU při startu!
                turn_pid_.reset();
                break; // Konec tohoto cyklu, samotné otáčení začne až v dalším
            }

            // OPRAVA 3: NORMÁLNÍ JÍZDA CHODBOU
            // Časový krok musí odpovídat timeru, tedy 0.05f (50ms)
            float steering = wall_pid_.step(error, 0.05f);
            int correction;
            if (results.right >= 0.5f || results.left >= 0.5f) {
                correction = static_cast<int>(steering * 0.6);
            } else {
                correction = static_cast<int>(steering);
            }

            // TVRDÝ LIMIT: Zabráníme tomu, aby při snaze jet rovně kola couvala
            correction = std::clamp(correction, -10, 10);

            //RCLCPP_INFO(this->get_logger(), "Target Yaw: %.2f rad | Correction: %d", target_yaw_, correction);
            int base_speed = 140;
            set_speed(base_speed - correction, base_speed + correction);
            break;
        }

        case MazeState::TURNING: {
            // ZDE JIŽ NESMÍ BÝT turn_pid_.reset() !!! (jinak ztrácí paměť)

            float current_yaw = imu_integrator_.getYaw();
            float yaw_error = target_yaw_ - current_yaw;

            while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            // Pokud jsme cílového úhlu dosáhli (tolerance)
            set_speed(127, 127); // Okamžitě zastavit motory
            if (std::abs(yaw_error) < 0.05f) {
                current_state_ = MazeState::CORRIDOR_FOLLOWING;

                // Opětovné vyčištění paměti regulátoru pro zeď, ať sebou po rohu netrhne
                wall_pid_.reset();

                //RCLCPP_INFO(this->get_logger(), "Otocka hotova, jedu rovne.");
                break;
            }

            // OPRAVA 2: Časový krok pro PID zatáčení musí být opět 0.05f (50ms)
            float turn_speed = turn_pid_.step(yaw_error, 0.05f);
            int correction;
            if (results.right >= 0.5f || results.left >= 0.5f) {
                correction = static_cast<int>(turn_speed * 0.9f);
            } else {
                correction = static_cast<int>(turn_speed);
            }

            // Limit síly zatáčení
            correction = std::clamp(correction, -14, 14);

            set_speed(127 - correction, 127 + correction);
            break;
        }
    }
}

} // namespace loops
