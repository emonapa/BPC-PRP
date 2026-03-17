#include "loops/line_loop.hpp"
#include "helper.hpp"
#include <algorithm>

namespace loops {

LineLoop::LineLoop() : rclcpp::Node("line_loop"),
                       // Zde nastavujete konstanty (Kp, Ki, Kd).
                       // Začněte jen s Kp (P-Control), např. 1500.0, a zbytek nechte na 0.
                       pid_controller_(1500.0f, 0.0f, 0.0f)
{
    line_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        Topic::get_line, 10,
        std::bind(&LineLoop::line_callback, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);

    // Smyčka běží každých 50 ms (20 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&LineLoop::timer_callback, this));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "LineLoop initialized.");
}

void LineLoop::line_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;
    // Callback už jen ukládá aktuální chybu. Výpočet pohybu obstará timer.
    current_error_ = LineEstimator::estimate_continuous(msg->data[0], msg->data[1]);
}

void LineLoop::timer_callback() {
    rclcpp::Time now = this->now();
    float dt = (now - last_time_).seconds();

    if (dt <= 0.0f) return; // Ochrana proti dělení nulou
    last_time_ = now;

    // 1. Krok PID regulátoru: Vypočítáme korekci (úhlovou rychlost)
    float output = pid_controller_.step(current_error_, dt);

    // 2. Aplikujeme korekci na motory
    int base_speed = 138;
    // Pokud je chyba kladná (čára vlevo), output bude kladný.
    // Levé kolo zpomalí, pravé zrychlí -> robot zatočí doleva.
    int left_speed = base_speed - static_cast<int>(output);
    int right_speed = base_speed + static_cast<int>(output);

    // 3. (Volitelné) Oříznutí výstupu - saturace
    left_speed = std::clamp(left_speed, 127, 255);
    right_speed = std::clamp(right_speed, 127, 255);

    // 4. Publikujeme rychlost
    std_msgs::msg::UInt8MultiArray speed_msg;
    speed_msg.data = { static_cast<uint8_t>(left_speed), static_cast<uint8_t>(right_speed) };
    motor_pub_->publish(speed_msg);
}

} // namespace loops
