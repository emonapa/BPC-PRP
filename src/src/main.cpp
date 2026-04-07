#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "nodes/line_node.hpp"

#include "loops/line_loop.hpp"
#include "nodes/lidar_node.hpp"
#include "loops/robot_movement.hpp"
/*
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    //auto motor_node = std::make_shared<nodes::MotorNode>();
    //auto line_node = std::make_shared<nodes::LineNode>(); // This will now work
    // auto line_loop = std::make_shared<loops::LineLoop>(); // This will now work
    // auto lidar = std::make_shared<nodes::LidarNode>();
    auto movement =std::make_shared<loops::MovementLoop>();
    rclcpp::executors::SingleThreadedExecutor executor;
    //executor.add_node(motor_node);
    executor.add_node(movement);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
*/
#include <rclcpp/rclcpp.hpp>
#include <memory>

// Zahrnutí pouze těch uzlů, které skutečně používáme
#include "nodes/motor_node.hpp"
#include "loops/robot_movement.hpp"

// Staré uzly (io_node, line_node, lidar_node, imu_node) už sem vůbec neincludujeme,
// abychom udrželi kód čistý a vyhnuli se konfliktům.

int main(int argc, char **argv) {
    // 1. Inicializace ROS 2
    rclcpp::init(argc, argv);

    // 2. Vytvoření instancí uzlů
    // MotorNode jen potichu běží na pozadí a zpracovává tiky z enkodérů pro odometrii
    auto motor_node = std::make_shared<nodes::MotorNode>();

    // MovementLoop si bere data z LiDARu a IMU, vyhodnocuje stavový automat a řídí rychlost
    auto movement_loop = std::make_shared<loops::MovementLoop>();

    // 3. Vytvoření executoru
    // Protože máme více než jeden uzel, musíme použít Executor, který se bude
    // plynule přepínat mezi callbacky obou uzlů.
    rclcpp::executors::SingleThreadedExecutor executor;

    // 4. Přidání uzlů do executoru
    executor.add_node(motor_node);
    executor.add_node(movement_loop);

    // 5. Spuštění nekonečné smyčky (program tu poběží, dokud ho neukončíte např. CTRL+C)
    executor.spin();

    // 6. Bezpečné ukončení ROS 2
    rclcpp::shutdown();

    return 0;
}
