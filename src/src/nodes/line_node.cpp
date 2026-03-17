#include "nodes/line_node.hpp"
#include "helper.hpp"
#include <algorithm> // Přidáno pro std::clamp

namespace nodes {

LineNode::LineNode() : rclcpp::Node("line_node") {
    // Subscriber pro čtení čáry
    line_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        Topic::get_line, 10,
        std::bind(&LineNode::on_line_callback, this, std::placeholders::_1));

    // NOVÉ: Publisher pro ovládání motorů z LineNode
    motor_speed_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", Topic::get_line.c_str());
}

void LineNode::on_line_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;

    int base_speed = 138; // Změněno na int kvůli matematickým operacím níže
    int left_speed = base_speed;
    int right_speed = base_speed;

    uint16_t left = msg->data[0];
    uint16_t right = msg->data[1];

    continuous_pose_ = LineEstimator::estimate_continuous(left, right);
    discrete_pose_ = LineEstimator::estimate_discrete(left, right);

    // Výpočet korekce
    int correction = static_cast<int>(continuous_pose_ * 1500);

    if (discrete_pose_ == DiscreteLinePose::LineOnRight) {
        left_speed = base_speed - correction;
    } else if (discrete_pose_ == DiscreteLinePose::LineOnLeft) {
        right_speed = base_speed - correction; // Předtím jste tu měli chybu se znaménkem
    }

    // Bezpečnostní oříznutí hodnot, aby nepřetekly přes 255 nebo neklesly pod 127
    left_speed = std::clamp(left_speed, 127, 255);
    right_speed = std::clamp(right_speed, 127, 255);

    // Vytvoření a odeslání zprávy
    std_msgs::msg::UInt8MultiArray speed_msg;
    speed_msg.data = { static_cast<uint8_t>(left_speed), static_cast<uint8_t>(right_speed) };
    motor_speed_publisher_->publish(speed_msg);

    // Debug výpis
    // RCLCPP_INFO(this->get_logger(), "Motors L: %d | R: %d", left_speed, right_speed);
}

} // namespace nodes
