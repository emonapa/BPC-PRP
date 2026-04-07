#include "loops/robot_movement.hpp"
#include "helper.hpp"
#include <algorithm>
#include "algorithms/lidar_alg.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace loops {

MovementLoop::MovementLoop() : rclcpp::Node("robot_movement_node"),
                       // Zde nastavujete konstanty (Kp, Ki, Kd).
                       // Začněte jen s Kp (P-Control), např. 1500.0, a zbytek nechte na 0.
                       pid_controller_(100.0f, 0.0f, 10.0f)
{
    // V súbore robot_movement.cpp v konštruktore pridaj:
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        Topic::get_lidar,
        10,
        std::bind(&MovementLoop::move_callback, this, std::placeholders::_1)
    );
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);

    // Smyčka běží každých 50 ms (20 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MovementLoop::timer_callback, this));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Motor initialized.");
}

void MovementLoop::move_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!msg) return;
    latest_scan_ = msg;

}

void MovementLoop::timer_callback() {
    rclcpp::Time now = this->now();
    float dt = (now - last_time_).seconds();
    if (dt <= 0) dt = 0.05;

    algorithms::LidarFilter filter;
    last_time_ = now;

    if (!latest_scan_) return;

    auto results = filter.apply_filter(
        latest_scan_->ranges,
        latest_scan_->angle_min,
        latest_scan_->angle_max
    );


    const float MAX_LOOK_DIST = 0.5f;

    // 2. Pokud je hodnota z LIDARu příliš velká nebo chybná (inf/nan), omez ji
    float L = (std::isfinite(results.left) && results.left < MAX_LOOK_DIST) ? results.left : MAX_LOOK_DIST;
    float R = (std::isfinite(results.right) && results.right < MAX_LOOK_DIST) ? results.right : MAX_LOOK_DIST;

    // 3. Výpočet chyby
    current_error_ = L - R;

    // PID výstup
    float steering_output = pid_controller_.step(current_error_, dt);

    // --- DYNAMICKÁ RYCHLOST ---
    int base_speed = 140;

    // Pokud hodně zatáčím (vysoký steering), zpomalím base_speed, abych nevyletěl
        if (std::abs(steering_output) > 20) {
            base_speed = 135;
        }

        int left_speed  = base_speed - static_cast<int>(steering_output);
        int right_speed = base_speed + static_cast<int>(steering_output);

        // Pokud je jedna strana úplně volná (např. v zatáčce)
        if (R >= MAX_LOOK_DIST && results.front < 0.4f) {
            // Vidím zeď před sebou a vpravo je volno -> Jdi ostře doprava
            left_speed = 134;
            right_speed = 120;
        }
        else if (L >= MAX_LOOK_DIST && results.front < 0.4f) {
            // Vidím zeď před sebou a vlevo je volno -> Jdi ostře doleva
            left_speed = 120;
            right_speed = 134;
        }
        else {
            // Standardní PID udržování středu
            float steering = pid_controller_.step(current_error_, dt);
            left_speed = base_speed - steering;
            right_speed = base_speed + steering;
        }

            // Publikace (clamp zůstává)
            std_msgs::msg::UInt8MultiArray speed_msg;
            speed_msg.data.push_back(static_cast<uint8_t>(std::clamp(left_speed, 0, 255)));
            speed_msg.data.push_back(static_cast<uint8_t>(std::clamp(right_speed, 0, 255)));
            motor_pub_->publish(speed_msg);
        }

} // namespace loops
