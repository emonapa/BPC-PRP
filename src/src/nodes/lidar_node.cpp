#include "nodes/lidar_node.hpp"
#include "helper.hpp"
#include <algorithm> 
#include "algorithms/lidar_alg.hpp"

namespace nodes{
    LidarNode::LidarNode():rclcpp::Node("lidar_node") {
    // Subscriber pro čtení čáry
    lidar_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
    Topic::get_lidar, 10,
    std::bind(&LidarNode::on_lidar_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", Topic::get_lidar.c_str());
    }   

    void LidarNode::on_lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        
        algorithms::LidarFilter filter;

        
        auto results = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max);

        
        RCLCPP_INFO(this->get_logger(), "Front: %f, Left: %f, Right: %f, Back: %f",
                    results.front, results.left, results.right, results.back);
                }
}