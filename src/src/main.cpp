#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/line_node.hpp"

#include "loops/line_loop.hpp"
#include "nodes/lidar_node.hpp"
#include "loops/robot_movment.hpp"
/*
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    //auto motor_node = std::make_shared<nodes::MotorNode>();
    //auto line_node = std::make_shared<nodes::LineNode>(); // This will now work
    // auto line_loop = std::make_shared<loops::LineLoop>(); // This will now work
    // auto lidar = std::make_shared<nodes::LidarNode>();
    auto movment =std::make_shared<loops::MovmentLoop>();
    rclcpp::executors::SingleThreadedExecutor executor;
    //executor.add_node(motor_node);
    executor.add_node(movment);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
*/

#include "nodes/imu_node.hpp"
// ...
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // auto lidar_node = std::make_shared<nodes::LidarNode>();
    auto imu_node = std::make_shared<nodes::ImuNode>(); // ZDE

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(imu_node); // ZDE

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
