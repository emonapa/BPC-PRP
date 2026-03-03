#include "nodes/io_node.hpp"
#include "helper.hpp"

namespace nodes {

IoNode::IoNode() : rclcpp::Node("io_node") {
    button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        Topic::buttons, 10,
        std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

    //led_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("led_commands", 10);
    led_publisher_ = this->create_publisher<std_msgs::msg::ColorRGBA>("led_light", 10);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", Topic::buttons.c_str());
}

void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    button_pressed_.store(static_cast<int>(msg->data), std::memory_order_relaxed);
    RCLCPP_INFO(this->get_logger(), "Button pressed: %u", msg->data);
    if (button_pressed_ == 1) {
        send_led_command(1,1,1,1);
    }
}

int IoNode::get_button_pressed() const {
    return button_pressed_.load(std::memory_order_relaxed);
}

void IoNode::publish_led(float r, float g, float b, float a) {
        auto msg = std_msgs::msg::ColorRGBA();
        msg.r = r; msg.g = g; msg.b = b; msg.a = a;
        led_publisher_->publish(msg);
    }


}










root@demon-papa-5430:~/ws/src# colcon build --packages-select prp_project
Starting >>> prp_project
Finished <<< prp_project [12.9s]

Summary: 1 package finished [13.1s]
root@demon-papa-5430:~/ws/src# ros2 run prp_project prp_project
[INFO] [1771950389.342317674] [io_node]: Subscribed to /bpc_prp_robot/buttons
[INFO] [1771950391.535225510] [io_node]: Button pressed: 1
[INFO] [1771950392.323391005] [io_node]: Button pressed: 0
[INFO] [1771950393.182998947] [io_node]: Button pressed: 2
[INFO] [1771950394.015934438] [io_node]: Button pressed: 1
[INFO] [1771950394.626138563] [io_node]: Button pre








// namespace nodes

/*
#include "nodes/io_node.hpp"

namespace nodes {

    // Konštruktor: inicializujeme Node a vytvoríme odberateľa
    IoNode::IoNode() : Node("io_node") {
        // "button_topic" je názov témy, ktorú budeme počúvať
        // 10 je veľkosť QoS (Quality of Service) histórie správ
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "button_events",
            10,
            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "IO Node bol spustený a čaká na tlačidlá.");
    }

    // Callback: spustí sa automaticky pri príchode správy
    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        // Uložíme dáta zo správy do našej premennej
        button_pressed_ = msg->data;

        RCLCPP_DEBUG(this->get_logger(), "Prijaté tlačidlo: %d", button_pressed_);
    }

    // Getter: vráti aktuálnu hodnotu
    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

}
*/
