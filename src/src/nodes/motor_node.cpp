#include "nodes/motor_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include "helper.hpp"

#include <chrono>
#define DELTA 50 //ms
#define PI 3.14f
#define RADIUS 0.033f

using namespace std::chrono_literals;

namespace nodes {

typedef struct {
    rclcpp::Time last_time;
    int last_rotations_left;
    int last_rotations_right;
    double last_speed_left;
    double last_speed_right;
} WheelInfo;

WheelInfo wheelInfo;

MotorNode::MotorNode() : rclcpp::Node("motor_node") {
    wheelInfo.last_time = this->now();
    wheelInfo.last_rotations_left = 0;
    wheelInfo.last_rotations_right = 0;
    wheelInfo.last_speed_right = 0;
    wheelInfo.last_speed_left = 0;

    motor_speed_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
        Topic::get_motor_speed, 10,
        std::bind(&MotorNode::motor_callback, this, std::placeholders::_1));

    motor_speed_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);

    start_time_ = this->now();
    timer_t = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MotorNode::timer_callback, this));
}

void MotorNode::timer_callback() {
//    ted to ridi line_loop.cpp
//
//    // Spočítáme, kolik času uběhlo od spuštění uzlu
//    auto elapsed_time = this->now() - start_time_;
//
//    // Pokud neuběhla ještě 1 sekunda, jedeme naplno
//    if (elapsed_time.seconds() < 60.0) {
//        publish_motor_speed(138, 138);
//    }
//    // Pokud uběhla více než 1 sekunda, zastavíme motory
//    else {
//        publish_motor_speed(127, 127);
//    }
}


double MotorNode::getSpeed(rclcpp::Time last_time, rclcpp::Time current_time, int current_ticks, int last_ticks) {
    // Zjistíme časový rozdíl v sekundách
    double delta_time = (current_time - last_time).seconds();

    // Ochrana proti dělení nulou (nebo zápornému času, pokud by se zbláznily hodiny)
    if (delta_time <= 0.0) {
        return 0.0;
    }

    // Výpočet: (změna tiků / 550) dá počet otáček, to vynásobíme obvodem (2 * PI * RADIUS), a vydělíme časem
    double delta_ticks = static_cast<double>(current_ticks - last_ticks);
    return (delta_ticks / 550.0 * 2.0 * PI * RADIUS) / delta_time;
}

void MotorNode::motor_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
    // Uložíme si aktuální čas
    rclcpp::Time current_time = this->now();

    // VŽDY inicializujeme rychlosti na nulu, abychom zamezili čtení "odpadu" z paměti
    double current_speed_left = 0.0;
    double current_speed_right = 0.0;

    // Zavoláme výpočet rychlosti. Už nepotřebujeme podmínku "!= 0",
    // protože i při nulových ticích chceme vědět, že rychlost je 0.
    current_speed_left = getSpeed(wheelInfo.last_time, current_time, msg->data[0], wheelInfo.last_rotations_left);
    current_speed_right = getSpeed(wheelInfo.last_time, current_time, msg->data[1], wheelInfo.last_rotations_right);

    // Aktualizujeme data ve struktuře wheelInfo pro další průběh
    wheelInfo.last_rotations_left = msg->data[0];
    wheelInfo.last_rotations_right = msg->data[1];

    wheelInfo.last_speed_left = current_speed_left;
    wheelInfo.last_speed_right = current_speed_right;

    wheelInfo.last_time = current_time;

    // Vypíšeme výsledky
    //RCLCPP_INFO(this->get_logger(), "Motor speed ticks: %u %u", msg->data[0], msg->data[1]);
    //RCLCPP_INFO(this->get_logger(), "Motor speed in m/s: %f %f", current_speed_left, current_speed_right);
}


void MotorNode::publish_motor_speed(uint8_t left_speed, uint8_t right_speed) const {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = { left_speed, right_speed };
    motor_speed_publisher_->publish(msg);
}

}
