#pragma once

#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes {

class IoNode : public rclcpp::Node {
public:
    IoNode();
    ~IoNode() override = default;

    // vrati posledni prijate tlacitko, -1 kdyz nic neprislo
    int get_button_pressed() const;
    void publish_led(float r, float g, float b, float a) const;

private:
    void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    std::atomic<int> button_pressed_{-1};
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_publisher_;
    //led_publisher_ = this->create_publisher<std_msgs::msg::ColorRGBA>("led_light", 10);
};

} // namespace nodes
