#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp> // PŘIDÁNO PRO KAMERU
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "algorithms/pid.hpp"
#include "algorithms/planar_imu_integrator.hpp"
#include "algorithms/aruco_detector.hpp"        // PŘIDÁNO PRO KAMERU
#include "helper.hpp"

namespace loops {

enum class MazeState {
    CALIBRATION,
    CORRIDOR_FOLLOWING,
    TURNING
};

class MovementLoop : public rclcpp::Node {
public:
    MovementLoop();
    ~MovementLoop() override = default;

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg); // PŘIDÁNO PRO KAMERU
    void timer_callback();

    void set_speed(int left, int right);

    // Subscribery a Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_; // PŘIDÁNO PRO KAMERU
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    algorithms::PlanarImuIntegrator imu_integrator_;
    std::vector<float> gyro_calibration_samples_;

    // --- PAMĚŤ PRO KAMERU A ZNAČKY ---
    algorithms::ArucoDetector aruco_detector_;
    std::vector<algorithms::ArucoDetector::Aruco> last_markers_;

    // PID regulátory
    algorithms::Pid wall_pid_;
    algorithms::Pid turn_pid_;

    // Proměnné pro stavový automat
    MazeState current_state_ = MazeState::CALIBRATION;
    float target_yaw_ = 0.0f;
    rclcpp::Time last_imu_time_;
    bool first_imu_ = true;
};

} // namespace loops
