#include "nodes/imu_node.hpp"
#include "helper.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

ImuNode::ImuNode() : rclcpp::Node("imu_node") {
    // Topic máte z terminálu: /bpc_prp_robot/imu
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/bpc_prp_robot/imu", 10,
        std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        Topic::set_motor_speed, 10);

    RCLCPP_INFO(this->get_logger(), "ImuNode spusteno. Zacinam kalibraci -> NEHYBAT S ROBOTEM!");
}

void ImuNode::set_target_yaw(float target_yaw) {
    target_yaw_ = target_yaw;
}

void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 1. Výpočet času dt mezi zprávami
    rclcpp::Time current_time = msg->header.stamp;

    if (first_msg_) {
        last_time_ = current_time;
        first_msg_ = false;
        return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Ochrana proti poškozeným časovým značkám
    if (dt <= 0.0 || dt > 0.5) return;

    // 2. Načtení z-osy gyroskopu (rotace na místě v rad/s)
    float gyro_z = msg->angular_velocity.z;

    // --- STAVOVÝ AUTOMAT PRO IMU ---
    if (mode_ == ImuNodeMode::CALIBRATE) {
        gyro_calibration_samples_.push_back(gyro_z);

        // Po cca 2-3 vteřinách (cca 200 zpráv) kalibraci ukončíme
        if (gyro_calibration_samples_.size() >= 200) {
            planar_integrator_.setCalibration(gyro_calibration_samples_);
            mode_ = ImuNodeMode::INTEGRATE;
            target_yaw_ = 0.0f; // Jakmile je zkalibrován, stanovíme cíl na "drž aktuální úhel 0"
            RCLCPP_INFO(this->get_logger(), "Kalibrace dokoncena. Aktivuji udrzovani uhlu.");
        }
    }
    else if (mode_ == ImuNodeMode::INTEGRATE) {

        // A) Integrace = výpočet aktuálního úhlu
        planar_integrator_.update(gyro_z, dt);
        float current_yaw = planar_integrator_.getYaw();

        // B) Výpočet regulační odchylky
        float yaw_error = target_yaw_ - current_yaw;

        // Normalizace chyby do rozsahu [-PI, PI], aby robot rotoval kratší cestou
        while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

        // C) P-Regulátor (převod úhlové chyby na sílu do motorů)
        float Kp = 60.0f; // Tuto hodnotu musíte poladit! Čím větší, tím silněji se bude vracet.
        int correction = static_cast<int>(yaw_error * Kp);

        // Bezpečnostní omezení - maximální síla zatáčení (např. 60 z 127)
        correction = std::clamp(correction, -60, 60);

        // D) Diferenciální otáčení na místě
        int left_speed = 127 - correction;
        int right_speed = 127 + correction;

        left_speed = std::clamp(left_speed, 0, 255);
        right_speed = std::clamp(right_speed, 0, 255);

        std_msgs::msg::UInt8MultiArray speed_msg;
        speed_msg.data = { static_cast<uint8_t>(left_speed), static_cast<uint8_t>(right_speed) };
        // motor_pub_->publish(speed_msg);

        // Volitelné: Odkomentujte pro vidění dat
        // RCLCPP_INFO(this->get_logger(), "Yaw: %.2f | Chyba: %.2f | Kor: %d", current_yaw, yaw_error, correction);
    }
}

} // namespace nodes
