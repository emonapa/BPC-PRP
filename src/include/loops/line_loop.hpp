#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "LineEstimator.hpp"
#include "algorithms/pid.hpp"

namespace loops {

class LineLoop : public rclcpp::Node {
public:
    LineLoop();
    ~LineLoop() override = default;

private:
    void line_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    void timer_callback(); // Pravidelná smyčka pro výpočet PID

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Proměnné pro řízení
    float current_error_ = 0.0f;
    rclcpp::Time last_time_;
    algorithms::Pid pid_controller_;
};

} // namespace loops
