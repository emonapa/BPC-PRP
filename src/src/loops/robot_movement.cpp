#include "loops/robot_movement.hpp"
#include "algorithms/lidar_alg.hpp"
#include <opencv2/opencv.hpp> // PŘIDÁNO PRO KAMERU
#include <algorithm>
#include <cmath>
#define SPEED 155


namespace loops {

MovementLoop::MovementLoop() : rclcpp::Node("robot_movement_node"),
    wall_pid_(20.0f, 0.0f, 25.0f), // PID pro držení se zdi
    turn_pid_(25.0f, 0.0f, 8.0f)   // PID pro přesné otáčení na úhel
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
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        auto markers = aruco_detector_.detect(frame);
        
        for (auto& m : markers) {
            // Treasure (10, 11, 12) má prioritu pred Exit (0, 1, 2)
            if (m.id >= 10 && m.id <= 12) {
                stored_decision_ = m.id;
                RCLCPP_INFO(this->get_logger(), "Ulozeny TREASURE kod: %d", m.id);
                break; // Treasure má top prioritu, končíme hľadanie
            } else if (m.id >= 0 && m.id <= 2) {
                if (stored_decision_ < 10) { // Prepíš len ak nemáme uložený Treasure
                    stored_decision_ = m.id;
                    RCLCPP_INFO(this->get_logger(), "Ulozeny EXIT kod: %d", m.id);
                }
            }
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


const float THRESH_HIGH = 0.42f; // Musí byť VIAC ako toto, aby sme si boli istí, že je voľno
const float THRESH_LOW  = 0.35f; // Musí klesnúť POD toto, aby sme povedali, že tam niečo je


if (L > (last_left_seen_ >= 2 ? 0.38f : THRESH_HIGH)) {
    if (last_left_seen_ < 2) last_left_seen_++; 
} else {
    if (last_left_seen_ > 0) last_left_seen_--;
}


if (R > (last_right_seen_ >= 2 ? 0.38f : THRESH_HIGH)) {
    if (last_right_seen_ < 2) last_right_seen_++;
} else {
    if (last_right_seen_ > 0) last_right_seen_--;
}

// -------------------------
// LOGIKA STAVOV (Kľúčová zmena)
// -------------------------
// Cesta sa považuje za otvorenú až keď sme si 100% istí (counter dosiahol 5)
bool is_left_open  = (last_left_seen_ >= 2);
bool is_right_open = (last_right_seen_ >= 2);


const float FRONT_BLOCK_HIGH = 0.25f; // Prah pre detekciu steny (aktivácia)
const float FRONT_BLOCK_LOW  = 0.45f;
if (front_blocked_state_) {
    if (F > FRONT_BLOCK_LOW) front_blocked_state_ = false;
} else {
    if (F < FRONT_BLOCK_HIGH) front_blocked_state_ = true;
}

bool is_front_blocked = front_blocked_state_;

bool is_front_close =F< FRONT_BLOCK_LOW;
// -------------------------
// IGNORE AFTER TURN
// -------------------------
if (ignore_side_counters_ > 0) {
    float error_center = std::clamp(static_cast<int>(L), 0, 30) - std::clamp(static_cast<int>(R), 0, 30); ;
    float steering = wall_pid_.step(error_center, 0.05f);
    // set_speed(145 - static_cast<int>(steering), 145 + static_cast<int>(steering));
    set_speed(SPEED,SPEED);
    ignore_side_counters_ --;
    return;
}

    // Identifikácia situácie
    int open_paths = (is_left_open ? 1 : 0) + (is_right_open ? 1 : 0) + (F > 0.35f ? 1 : 0);
    float current_yaw = imu_integrator_.getYaw();
    // ROZHODOVANIE - Spustí sa len ak:
    // 1. Je pred nami stena (T-križovatka alebo zákruta)
    // 2. Sú OTOVRENÉ OBE STRANY (X-križovatka) - tu predpokladáme, že sme v strede
    if(is_front_close){
    if (is_front_blocked) {
        
        
        int direction = (stored_decision_ != -1) ? (stored_decision_ % 10) : -1;
        bool state_changed = false;
        bool state_turn =false;

        // KRIŽOVATKA (viac ako 1 cesta vpred/do boku)
        if (open_paths >= 2) {
            RCLCPP_INFO(this->get_logger(), "Krizovatka %d %d %d",direction,is_left_open,is_right_open);
            if (direction == 1 && is_left_open) {
                target_yaw_ = normalize_angle(current_yaw + (M_PI / 2.0f));
                state_changed = true;
            } else if (direction == 2 && is_right_open) {
                target_yaw_ = normalize_angle(current_yaw - (M_PI / 2.0f));
                state_changed = true;
            } else if (direction == -1) {
                target_yaw_ = normalize_angle(current_yaw + (M_PI / 2.0f));
                state_changed = true;
            }
        }       
        else if (is_front_blocked) {
            state_turn =true;
             RCLCPP_INFO(this->get_logger(), "Zakruta %d %d %d" ,direction, is_left_open ,is_right_open);
            if (is_left_open) {
                target_yaw_ = normalize_angle(current_yaw + (M_PI / 2.0f));
                RCLCPP_INFO(this->get_logger(), "left %d",direction);
                state_changed = true;
                
            } else if (is_right_open) {
                target_yaw_ = normalize_angle(current_yaw - (M_PI / 2.0f));
                 RCLCPP_INFO(this->get_logger(), "right %d",direction);
                state_changed = true;
            } else {
                target_yaw_ = normalize_angle(current_yaw + M_PI); // Otočka
                 RCLCPP_INFO(this->get_logger(), "otocka %d",direction);
                state_changed = true;
            }
        }

        if (state_changed) {
            RCLCPP_INFO(this->get_logger(), "CHANGING STATE ");
            // set_speed(127, 127);
            turn_start_delay_ = 0; // cca 150ms (3 * 50ms)
            current_state_ = MazeState::TURNING;
            if(!state_turn) stored_decision_ = -1;
            return;
        }
    }
    set_speed(SPEED,SPEED);
    return;
}

    float steering = 0.0f;
    int base_speed = SPEED;

     bool state_changed =false;
    int direction = (stored_decision_ != -1) ? (stored_decision_ % 10) : -1;
    if (is_left_open && is_right_open) {
        RCLCPP_INFO(this->get_logger(), "X %d",direction);
        if (direction == 1 && is_left_open) {
            target_yaw_ = normalize_angle(current_yaw + (M_PI / 2.0f));
            state_changed = true;
        } else if (direction == 2 && is_right_open) {
            target_yaw_ = normalize_angle(current_yaw - (M_PI / 2.0f));
            state_changed = true;
        }
        else{
        //   target_yaw_ =current_yaw;
            state_changed = true;
            steering = 0.0f;
        }
    } else if (is_left_open) {
         RCLCPP_INFO(this->get_logger(), "L  %d",direction);
        if (direction == 1 && is_left_open) {
            target_yaw_ = normalize_angle(current_yaw + (M_PI / 2.0f));
            state_changed = true;
        }
        else {
            // target_yaw_ =current_yaw;
           state_changed = true;
            steering = 0.0f;
            }
    } else if (is_right_open) {
         RCLCPP_INFO(this->get_logger(), "R %d",direction);
        if (is_right_open && direction == 2) {
                target_yaw_ = normalize_angle(current_yaw - (M_PI / 2.0f));
                state_changed = true;
        }
        else{
           //  RCLCPP_INFO(this->get_logger(), "base %d",direction);
            // target_yaw_ =current_yaw;
            state_changed = true;
            steering = 0.0f;
        }
    } else {
        float error_center = L - R;
        steering = wall_pid_.step(error_center, 0.05f);
    }

     if (state_changed) {
         RCLCPP_INFO(this->get_logger(), "CHANGING STATE  1");
            // set_speed(127, 127);
            turn_start_delay_ = 7; // cca 150ms (3 * 50ms)
            current_state_ = MazeState::TURNING;
            stored_decision_ = -1;
            return;
        }

    set_speed(base_speed - static_cast<int>(steering), base_speed + static_cast<int>(steering));
    break;
}

case MazeState::TURNING: {
     RCLCPP_INFO(this->get_logger(), "IN TURNING ");
     if (turn_start_delay_ > 0) {
        set_speed(SPEED, SPEED);
        turn_start_delay_--;
    return; // nič nerob → nech sa robot “rozbehne”
}
    float current_yaw = imu_integrator_.getYaw();
    float yaw_error = normalize_angle(target_yaw_ - current_yaw);

    if (std::abs(yaw_error) < 0.08f) {
         RCLCPP_INFO(this->get_logger(), "IN TURNING RESET "); // Tolerancia
        set_speed(127, 127);
        // KONIEC OTÁČANIA
        ignore_side_counters_ = 27;

        current_state_ = MazeState::CORRIDOR_FOLLOWING;
        turn_pid_.reset();
        break;
    }

    float turn_speed = turn_pid_.step(yaw_error, 0.05f);
    int correction = std::clamp(static_cast<int>(turn_speed), -40, 40); // Zvýšený rozsah

    // Jemnejší deadband - ak je chyba malá, zmenši aj minimálnu silu
    int min_force = (std::abs(yaw_error) < 0.2f) ? 10 : 15; 
    if (std::abs(correction) < min_force) {
        correction = (yaw_error > 0) ? min_force : -min_force;
    }
     RCLCPP_INFO(this->get_logger(), "IN TURNING  %d",correction);
    set_speed(127 - correction, 127 + correction);
    break;
}
    }
}
}