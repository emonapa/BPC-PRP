#pragma once

#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace nodes {

class MotorNode : public rclcpp::Node {
public:
    MotorNode();
    ~MotorNode() override = default;

    //int get_button_pressed() const;
    //void publish_led(uint8_t r, uint8_t g, uint8_t b) const;
    void publish_motor_speed(uint8_t left_speed, uint8_t right_speed) const;

private:
    void motor_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

    void timer_callback();

    std::atomic<int> button_pressed_{-1};

    double getSpeed(rclcpp::Time last_time, rclcpp::Time current_time, int current_rotation, int last_rotation);

    rclcpp::TimerBase::SharedPtr timer_t;
    rclcpp::Time start_time_;

    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr motor_speed_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_speed_publisher_;
};

} // namespace nodes
