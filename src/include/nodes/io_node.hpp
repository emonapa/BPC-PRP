#pragma once

#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {

class IoNode : public rclcpp::Node {
public:
    IoNode();
    ~IoNode() override = default;

    int get_button_pressed() const;
    void publish_led(uint8_t r, uint8_t g, uint8_t b) const;

private:
    void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    std::atomic<int> button_pressed_{-1};
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;
};

} // namespace nodes
