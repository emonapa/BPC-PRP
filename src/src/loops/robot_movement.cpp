#include "loops/robot_movement.hpp"
#include "algorithms/lidar_alg.hpp"
#include <algorithm>
#include <cmath>

namespace loops {

MovementLoop::MovementLoop() : rclcpp::Node("robot_movement_node"),
    wall_pid_(100.0f, 0.0f, 10.0f), // PID pro držení se zdi
    turn_pid_(50.0f, 0.0f, 5.0f)    // PID pro přesné otáčení na úhel
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

void MovementLoop::timer_callback() {
    if (current_state_ == MazeState::CALIBRATION) {
        set_speed(127, 127); // Během kalibrace MUSÍ stát
        return;
    }

    if (!latest_scan_) return;

    algorithms::LidarFilter filter;
    auto results = filter.apply_filter(latest_scan_->ranges, latest_scan_->angle_min, latest_scan_->angle_max);

    // --- STAVOVÝ AUTOMAT ---
    switch (current_state_) {

        case MazeState::CORRIDOR_FOLLOWING: {
            float L = std::min(results.left, 0.5f);
            float R = std::min(results.right, 0.5f);
            float error = L - R;

            // Zjistíme, jestli nejsme v rohu (překážka vpředu)
            if (results.front < 0.35f) {
                // Přepínáme se do módu "OTÁČENÍ"
                current_state_ = MazeState::TURNING;

                float current_yaw = imu_integrator_.getYaw();

                if (R >= 0.4f) {
                    // Zeď vpředu, vpravo volno -> Cíl je -90 stupňů
                    target_yaw_ = current_yaw - (M_PI / 2.0f);
                    RCLCPP_INFO(this->get_logger(), "Roh! Budu tocit DOPRAVA.");
                } else {
                    // Zeď vpředu, vlevo volno -> Cíl je +90 stupňů
                    target_yaw_ = current_yaw + (M_PI / 2.0f);
                    RCLCPP_INFO(this->get_logger(), "Roh! Budu tocit DOLEVA.");
                }
                break; // Konec tohoto cyklu, motor se nastaví až příště
            }

            // Normální jízda chodbou (PID)
            float steering = wall_pid_.step(error, 0.05f);
            int base_speed = 145;
            set_speed(base_speed - static_cast<int>(steering), base_speed + static_cast<int>(steering));
            break;
        }

        case MazeState::TURNING: {
            float current_yaw = imu_integrator_.getYaw();
            float yaw_error = target_yaw_ - current_yaw;

            // Normalizace úhlu (aby se netočil jak pes za ocasem)
            while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            // Pokud jsme cílového úhlu dosáhli (tolerance cca 3 stupně)
            if (std::abs(yaw_error) < 0.05f) {
                current_state_ = MazeState::CORRIDOR_FOLLOWING;
                turn_pid_.reset(); // Vyčistíme integrál z PID
                RCLCPP_INFO(this->get_logger(), "Otocka hotova, jedu rovne.");
                break;
            }

            // Otáčení na místě pomocí PID
            float turn_speed = turn_pid_.step(yaw_error, 0.05f);
            int correction = static_cast<int>(turn_speed);

            // Limit síly zatáčení
            correction = std::clamp(correction, -40, 40);

            set_speed(127 - correction, 127 + correction);
            break;
        }
    }
}

} // namespace loops
