#include "loops/robot_movment.hpp"
#include "helper.hpp"
#include <algorithm>
#include "algorithms/lidar_alg.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace loops {

MovmentLoop::MovmentLoop() : rclcpp::Node("robot_movment_node"),
                       // Zde nastavujete konstanty (Kp, Ki, Kd).
                       // Začněte jen s Kp (P-Control), např. 1500.0, a zbytek nechte na 0.
                       pid_controller_(2000.0f, 200.0f, 50.0f)
{
    // V súbore robot_movment.cpp v konštruktore pridaj:
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        Topic::get_lidar, 
        10, 
        std::bind(&MovmentLoop::move_callback, this, std::placeholders::_1)
    );
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);
    
    // Smyčka běží každých 50 ms (20 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&MovmentLoop::timer_callback, this));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Motor initialized.");
}

void MovmentLoop::move_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!msg) return;
    latest_scan_ = msg; 
    
}

void MovmentLoop::timer_callback() {
    rclcpp::Time now = this->now();
    float dt = (now - last_time_).seconds();
    if (dt <= 0) dt = 0.001; // Prevent division by zero in PID if clocks match
    
    algorithms::LidarFilter filter;
    last_time_ = now;

    if (!latest_scan_) return;

    // 1. Get Filter Results (Assuming your filter provides an error or distance)
    auto results = filter.apply_filter(
        latest_scan_->ranges,
        latest_scan_->angle_min,
        latest_scan_->angle_max
    );

    // 2. Calculate PID Output
    // You need to set current_error_ based on your LIDAR logic first
    // For example: current_error_ = results.offset_from_center; 
    float output = pid_controller_.step(current_error_, dt);
    
    // 3. Determine Speeds
    int base_speed = 140; 
    int left_speed  = base_speed ;
    int right_speed = base_speed ;
    if(results.front < 0.21){
        // Stop/Slow down if obstacle is too close
        if(results.right >0.4){
            left_speed = 140;
            right_speed=0;
        }
         else if(results.left >0.4){
            left_speed = 140;
            right_speed=0;
        }

    }

    // Combine base speed with PID output (Differential drive logic)
   

    // 4. Log and Publish
    RCLCPP_INFO(this->get_logger(), "Front: %.4f | L: %.4f | R: %.4fB: %.4fspreed R %d speed L %d", 
                 results.front, results.left, results.right,results.back, right_speed,left_speed);

    std_msgs::msg::UInt8MultiArray speed_msg;
    // Explicitly push_back to avoid initializer_list conversion errors
    speed_msg.data.push_back(static_cast<uint8_t>(std::clamp(left_speed, 0, 255)));
    speed_msg.data.push_back(static_cast<uint8_t>(std::clamp(right_speed, 0, 255)));
    
    motor_pub_->publish(speed_msg);
}

} // namespace loops
