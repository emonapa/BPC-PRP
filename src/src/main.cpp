#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/line_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto motor_node = std::make_shared<nodes::MotorNode>();
    auto line_node = std::make_shared<nodes::LineNode>(); // This will now work

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motor_node);
    executor.add_node(line_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
