#pragma once

#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp> // Pro rychlost motorů
#include "LineEstimator.hpp"

namespace nodes {

class LineNode : public rclcpp::Node {
public:
    LineNode();
    ~LineNode() override = default;

private:
    void on_line_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

    // Přidáme vlastní publisher pro LineNode
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_speed_publisher_;

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_subscriber_;
    float continuous_pose_ = 0.0f;
    DiscreteLinePose discrete_pose_ = DiscreteLinePose::LineNone;
};

} // namespace nodes
