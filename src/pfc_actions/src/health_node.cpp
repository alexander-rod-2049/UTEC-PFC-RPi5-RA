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
        // Declarar parámetros de habilitación
        this->declare_parameter("check_imu", true);
        this->declare_parameter("check_motors", true);
        this->declare_parameter("check_odom", true);
        this->declare_parameter("check_flow", true);

        status_pub_ = this->create_publisher<pfc_msgs::msg::SystemStatus>("/system_status", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_topic", 10, [this](const sensor_msgs::msg::Imu::SharedPtr) { last_imu_time_ = this->now(); });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr) { last_odom_time_ = this->now(); });

        motor_sub_ = this->create_subscription<ros2_roboclaw_driver::msg::RoboClawStatus>(
            "/roboclaw_status", 10, std::bind(&HealthNode::motor_callback, this, std::placeholders::_1));

        flow_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sensor/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr) { last_flow_time_ = this->now(); });

        timer_ = this->create_wall_timer(500ms, std::bind(&HealthNode::check_health, this));

        RCLCPP_INFO(this->get_logger(), "HealthStatus con Bypass iniciado.");
    }

private:
    void motor_callback(const ros2_roboclaw_driver::msg::RoboClawStatus::SharedPtr msg)
    {
        last_motor_time_ = this->now();
        current_battery_voltage_ = msg->main_battery_voltage;
    }

    void check_health()
    {
        auto now = this->now();
        auto status = pfc_msgs::msg::SystemStatus();
        bool all_ok = true;
        std::string error_log = "";

        // Obtener parámetros de bypass
        bool check_imu = this->get_parameter("check_imu").as_bool();
        bool check_motors = this->get_parameter("check_motors").as_bool();
        bool check_odom = this->get_parameter("check_odom").as_bool();
        bool check_flow = this->get_parameter("check_flow").as_bool();

        // 1. IMU
        status.imu_ok = (now - last_imu_time_).seconds() < 1.0;
        if (check_imu && !status.imu_ok) { all_ok = false; error_log += "[IMU OFFLINE] "; }

        // 2. EKF
        status.odom_ok = (now - last_odom_time_).seconds() < 0.5;
        if (check_odom && !status.odom_ok) { all_ok = false; error_log += "[EKF LOST] "; }

        // 3. Motores (Tolerancia 1.5s por latencia serial)
        status.motors_ok = (now - last_motor_time_).seconds() < 1.5;
        if (check_motors && !status.motors_ok) { all_ok = false; error_log += "[MOTORS OFFLINE] "; }

        // 4. Flujo Óptico
        status.optical_flow_ok = (now - last_flow_time_).seconds() < 1.0;
        if (check_flow && !status.optical_flow_ok) { all_ok = false; error_log += "[FLOW SENSOR LOST] "; }

        // 5. Batería (Umbral 21V para orugas de 24V)
        status.battery_voltage = current_battery_voltage_;
        if (current_battery_voltage_ < 21.0 && status.motors_ok) {
            all_ok = false;
            error_log += "[LOW BATT] ";
        }

        status.status_message = all_ok ? "NOMINAL" : "FAULT: " + error_log;
        status_pub_->publish(status);
    }

    rclcpp::Publisher<pfc_msgs::msg::SystemStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr flow_sub_;
    rclcpp::Subscription<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr motor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME}, last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_motor_time_{0, 0, RCL_ROS_TIME}, last_flow_time_{0, 0, RCL_ROS_TIME};
    float current_battery_voltage_ = 0.0;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HealthNode>());
    rclcpp::shutdown();
    return 0;
}