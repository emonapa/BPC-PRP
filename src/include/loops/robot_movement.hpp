#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "algorithms/pid.hpp"
#include "algorithms/lidar_alg.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace loops {

class MovementLoop : public rclcpp::Node {
public:
    MovementLoop();
    ~MovementLoop() override = default;

private:
    // robot_movement.hpp
   void move_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback(); // Pravidelná smyčka pro výpočet PID
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    // Proměnné pro řízení
    float current_error_ = 0.0f;
    rclcpp::Time last_time_;
    algorithms::Pid pid_controller_;
};

} // namespace loops
