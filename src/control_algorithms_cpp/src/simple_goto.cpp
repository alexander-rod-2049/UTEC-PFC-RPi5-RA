#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> // <-- CAMBIADO A STAMPED
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

class SimpleGoTo : public rclcpp::Node
{
public:
    SimpleGoTo() : Node("simple_goto_node")
    {
        // 1. Publicador estricto para Gazebo (TwistStamped)
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        
        // 2. Suscriptor (QoS en 10, que por defecto es RELIABLE, compatible con Gazebo)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, 
            std::bind(&SimpleGoTo::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SimpleGoTo::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🚀 Nodo iniciado. Objetivo: X=%.2f, Y=%.2f", target_x_, target_y_);
    }

private:
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    bool valid_odom_ = false;

    double target_x_ = 1.0;
    double target_y_ = 1.0;

    double k_linear_ = 0.5;
    double k_angular_ = 1.5;
    double max_v_ = 0.2;
    double max_w_ = 1.0;
    double distance_tolerance_ = 0.05;
    bool goal_reached_ = false;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
        
        if (!valid_odom_) {
            RCLCPP_INFO(this->get_logger(), "✅ Primera odometría recibida! X=%.2f", current_x_);
            valid_odom_ = true;
        }
    }

    void control_loop()
    {
        if (!valid_odom_ || goal_reached_) {
            return; 
        }

        // CREAMOS EL MENSAJE STAMPED
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";

        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance_error = std::sqrt(dx*dx + dy*dy);
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = target_yaw - current_yaw_;

        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        if (distance_error < distance_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "🏁 ¡META ALCANZADA! Distancia: %.3fm", distance_error);
            msg.twist.linear.x = 0.0;
            msg.twist.angular.z = 0.0;
            cmd_pub_->publish(msg);
            goal_reached_ = true;
            return;
        }

        // ACCEDEMOS USANDO msg.twist...
        msg.twist.linear.x = std::min(k_linear_ * distance_error, max_v_);
        msg.twist.angular.z = std::max(std::min(k_angular_ * yaw_error, max_w_), -max_w_);

        // Imprimir logs de control
        static int log_counter = 0;
        if (log_counter++ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Dist: %.2f m | YawErr: %.2f rad | Cmd: [v=%.2f, w=%.2f]", 
                distance_error, yaw_error, msg.twist.linear.x, msg.twist.angular.z);
        }

        cmd_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGoTo>());
    rclcpp::shutdown();
    return 0;
}