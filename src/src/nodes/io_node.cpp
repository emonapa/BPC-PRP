#include "nodes/io_node.hpp"
#include "helper.hpp"

namespace nodes {

IoNode::IoNode() : rclcpp::Node("io_node") {
    button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        Topic::buttons, 10,
        std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

    led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_rgb_leds, 10);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", Topic::buttons.c_str());
}

void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    button_pressed_.store(static_cast<int>(msg->data), std::memory_order_relaxed);
    RCLCPP_INFO(this->get_logger(), "Button pressed: %u", msg->data);

    if (button_pressed_.load(std::memory_order_relaxed) == 1) {
        publish_led(255, 255, 255);
    }
}

int IoNode::get_button_pressed() const {
    return button_pressed_.load(std::memory_order_relaxed);
}

void IoNode::publish_led(uint8_t r, uint8_t g, uint8_t b) const {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = { r,g,b,  r,g,b,  r,g,b,  r,g,b };
    led_publisher_->publish(msg);
}

}


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
