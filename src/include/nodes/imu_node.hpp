#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "algorithms/planar_imu_integrator.hpp"

namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode();
        ~ImuNode() override = default;

        // Umožní např. z jiného kódu říct robotovi "otoč se o 90 stupňů"
        void set_target_yaw(float target_yaw);

    private:
        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);

        ImuNodeMode mode_ = ImuNodeMode::CALIBRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;

        algorithms::PlanarImuIntegrator planar_integrator_;
        std::vector<float> gyro_calibration_samples_;

        rclcpp::Time last_time_;
        bool first_msg_ = true;

        float target_yaw_ = 0.0f; // Úhel, který se robot snaží udržet
    };

} // namespace nodes
