#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nodes::IoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();


    return 0;
}
