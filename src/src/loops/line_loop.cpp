#include "loops/line_loop.hpp"
#include "helper.hpp"
#include <algorithm>

namespace loops {

LineLoop::LineLoop() : rclcpp::Node("line_loop"),
                       // Zde nastavujete konstanty (Kp, Ki, Kd).
                       // Začněte jen s Kp (P-Control), např. 1500.0, a zbytek nechte na 0.
                       pid_controller_(2000.0f, 200.0f, 50.0f)
{
    line_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        Topic::get_line, 10,
        std::bind(&LineLoop::line_callback, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);

    // Smyčka běží každých 50 ms (20 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
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
    last_time_ = now;

    // 1. Ochrana: Pokud je dt příliš velké (např. vteřina), PID by se zbláznilo
    if (dt <= 0.0f || dt > 0.2f) return;

    // 2. Výpočet PID
    float output = pid_controller_.step(current_error_, dt);

    // 3. Dynamická base_speed
    // Tip: Můžeš base_speed snižovat, pokud je chyba (error) příliš velká = v zatáčce zpomalit.
    int base_speed = 180;

    // Výpočet rychlostí
    int left_speed  = base_speed - static_cast<int>(output);
    int right_speed = base_speed + static_cast<int>(output);

    // 4. Lepší saturace:
    // Pokud tvůj driver bere 0-255, kde 127 je stop, pak:
    left_speed  = std::clamp(left_speed, 127, 255);
    right_speed = std::clamp(right_speed, 127, 255);

    // Logování pro ladění (přidáno zobrazení chyby a výstupu vedle sebe)
    RCLCPP_INFO(this->get_logger(), "Err: %.7f | Out: %.2f | L: %d R: %d",
                 current_error_, output, left_speed, right_speed);

    // 5. Publikace
    std_msgs::msg::UInt8MultiArray speed_msg;
    speed_msg.data = { static_cast<uint8_t>(left_speed), static_cast<uint8_t>(right_speed) };
    motor_pub_->publish(speed_msg);
}

} // namespace loops
