#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "pfc_msgs/msg/system_status.hpp"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"

using namespace std::chrono_literals;

class HealthNode : public rclcpp::Node
{
public:
    HealthNode() : Node("health_node")
    {
        status_pub_ = this->create_publisher<pfc_msgs::msg::SystemStatus>("/system_status", 10);

        // 1. Monitoreo de IMU
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr) { last_imu_time_ = this->now(); });

        // 2. Monitoreo de Odometría Filtrada (EKF)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr) { last_odom_time_ = this->now(); });

        // 3. Monitoreo de RoboClaw (Motores/Batería)
        motor_sub_ = this->create_subscription<ros2_roboclaw_driver::msg::RoboClawStatus>(
            "/roboclaw_status", 10, std::bind(&HealthNode::motor_callback, this, std::placeholders::_1));

        // 4. NUEVO: Monitoreo de Flujo Óptico (MTF-02P)
        flow_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "sensor/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr) { last_flow_time_ = this->now(); });

        timer_ = this->create_wall_timer(500ms, std::bind(&HealthNode::check_health, this));

        RCLCPP_INFO(this->get_logger(), "HealthStatus iniciado: Supervisando todos los sensores (IMU, EKF, Motores, Flujo Optico).");
    }

private:
    void motor_callback(const ros2_roboclaw_driver::msg::RoboClawStatus::SharedPtr msg)
    {
        last_motor_time_ = this->now();
        current_battery_voltage_ = msg->main_battery_voltage;
        motor_error_code_ = msg->error_status;
    }

    void check_health()
    {
        auto now = this->now();
        auto status = pfc_msgs::msg::SystemStatus();
        bool all_ok = true;
        std::string error_log = "";

        // Check IMU
        status.imu_ok = (now - last_imu_time_).seconds() < 0.5;
        if (!status.imu_ok) { all_ok = false; error_log += "[IMU LOST] "; }

        // Check EKF
        status.odom_ok = (now - last_odom_time_).seconds() < 0.5;
        if (!status.odom_ok) { all_ok = false; error_log += "[EKF LOST] "; }

        // Check RoboClaw
        status.motors_ok = (now - last_motor_time_).seconds() < 1.0;
        if (!status.motors_ok) { all_ok = false; error_log += "[MOTORS OFFLINE] "; }

        // NUEVO: Check Flujo Óptico
        status.optical_flow_ok = (now - last_flow_time_).seconds() < 0.5;
        if (!status.optical_flow_ok) { all_ok = false; error_log += "[FLOW SENSOR LOST] "; }

        // Check Batería
        status.battery_voltage = current_battery_voltage_;
        if (current_battery_voltage_ < 11.0 && status.motors_ok) {
            all_ok = false;
            error_log += "[LOW BATT] ";
        }

        status.status_message = all_ok ? "NOMINAL" : "FAULT: " + error_log;

        if (!all_ok) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", status.status_message.c_str());
        }

        status_pub_->publish(status);
    }

    // Publicadores y Suscriptores
    rclcpp::Publisher<pfc_msgs::msg::SystemStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr flow_sub_; // Suscriptor de flujo
    rclcpp::Subscription<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr motor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Tiempos
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_motor_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_flow_time_{0, 0, RCL_ROS_TIME}; // Tiempo flujo

    float current_battery_voltage_ = 0.0;
    uint32_t motor_error_code_ = 0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HealthNode>());
    rclcpp::shutdown();
    return 0;
}