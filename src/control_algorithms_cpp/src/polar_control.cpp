#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // Nuevo para setpoints
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

class PolarControl : public rclcpp::Node
{
public:
    PolarControl() : Node("polar_control_node")
    {
        // Publicadores
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        // Este tópico servirá para que PlotJuggler vea el objetivo actual
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_setpoint", 10);

        // Suscriptores
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PolarControl::odom_callback, this, std::placeholders::_1));
        
        // Suscribirse a setpoints externos (puedes usar RViz o terminal)
        setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PolarControl::setpoint_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&PolarControl::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🚀 Nodo Polar Control Dinámico Iniciado.");
    }

private:
    // Estado actual
    double current_x_ = 0.0, current_y_ = 0.0, current_theta_ = 0.0;
    bool valid_odom_ = false;

    // Setpoints (Objetivos)
    double target_x_ = 0.0, target_y_ = 0.0, target_theta_ = 0.0;
    bool goal_reached_ = true; // Empezamos en true hasta recibir el primer setpoint

    // Ganancias y límites
    double K_rho_ = 0.3, K_alpha_ = 0.8, K_beta_ = -0.15;
    double max_v_ = 0.22, max_w_ = 1.0;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_theta_ = quaternion_to_yaw(msg->pose.pose.orientation);
        valid_odom_ = true;
    }

    void setpoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_x_ = msg->pose.position.x;
        target_y_ = msg->pose.position.y;
        target_theta_ = quaternion_to_yaw(msg->pose.orientation);
        
        goal_reached_ = false; // ¡Despertar al robot!
        RCLCPP_INFO(this->get_logger(), "📍 Nuevo Setpoint: X=%.2f, Y=%.2f, θ=%.2f", 
                    target_x_, target_y_, target_theta_);
    }

    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q_msg) {
        tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }

    void control_loop() {
        if (!valid_odom_) return;

        // Publicar el setpoint actual para PlotJuggler (útil para ver la línea recta del target)
        auto target_msg = geometry_msgs::msg::PoseStamped();
        target_msg.header.stamp = this->now();
        target_msg.header.frame_id = "odom";
        target_msg.pose.position.x = target_x_;
        target_msg.pose.position.y = target_y_;
        target_pub_->publish(target_msg);

        if (goal_reached_) return;

        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";

        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double rho = std::sqrt(dx*dx + dy*dy);
        double alpha = normalize_angle(std::atan2(dy, dx) - current_theta_);
        double beta = normalize_angle(target_theta_ - current_theta_ - alpha);

        // Control Lógico
        double v = K_rho_ * rho;
        double w = K_alpha_ * alpha + K_beta_ * beta;

        if (std::abs(alpha) > M_PI / 2) v = 0.0; // Solo girar si está de espaldas

        // Condición de parada
        if (rho < 0.05 && std::abs(normalize_angle(target_theta_ - current_theta_)) < 0.1) {
            RCLCPP_INFO(this->get_logger(), "🏁 Objetivo alcanzado.");
            v = 0.0; w = 0.0;
            goal_reached_ = true;
        }

        cmd.twist.linear.x = std::min(v, max_v_);
        cmd.twist.angular.z = std::clamp(w, -max_w_, max_w_);
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolarControl>());
    rclcpp::shutdown();
    return 0;
}