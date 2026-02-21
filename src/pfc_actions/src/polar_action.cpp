#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "pfc_msgs/msg/system_status.hpp" // Mensaje de salud
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

class PolarActionServer : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    PolarActionServer() : Node("polar_action_server")
    {
        // Declarar parámetros
        this->declare_parameter("k_rho", 0.3);
        this->declare_parameter("k_alpha", 0.8);
        this->declare_parameter("k_beta", -0.15);
        this->declare_parameter("max_v", 0.22);
        this->declare_parameter("max_w", 1.0);

        refresh_parameters();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&PolarActionServer::odom_callback, this, std::placeholders::_1));

        // Suscriptor de Seguridad Proactiva
        health_sub_ = this->create_subscription<pfc_msgs::msg::SystemStatus>(
            "/system_status", 10,
            [this](const pfc_msgs::msg::SystemStatus::SharedPtr msg) {
                last_health_status_ = msg;
            });

        this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&PolarActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PolarActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PolarActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Servidor Polar con Seguridad Proactiva iniciado.");
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<pfc_msgs::msg::SystemStatus>::SharedPtr health_sub_;

    double cur_x_ = 0.0, cur_y_ = 0.0, cur_yaw_ = 0.0;
    bool valid_odom_ = false;
    pfc_msgs::msg::SystemStatus::SharedPtr last_health_status_;

    // Ganancias
    double K_rho, K_alpha, K_beta, max_v, max_w;

    void refresh_parameters() {
        K_rho = this->get_parameter("k_rho").as_double();
        K_alpha = this->get_parameter("k_alpha").as_double();
        K_beta = this->get_parameter("k_beta").as_double();
        max_v = this->get_parameter("max_v").as_double();
        max_w = this->get_parameter("max_w").as_double();
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        (void)goal;
        (void)uuid;
        // No aceptar metas si el sistema está en falla desde el inicio
        if (last_health_status_ && last_health_status_->status_message.find("FAULT") != std::string::npos) {
            RCLCPP_ERROR(this->get_logger(), "Meta rechazada: El sistema reporta fallas de seguridad.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        std::thread{std::bind(&PolarActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        auto result = std::make_shared<NavigateToPose::Result>();
        
        tf2::Quaternion q_obj(
            goal->pose.pose.orientation.x, goal->pose.pose.orientation.y,
            goal->pose.pose.orientation.z, goal->pose.pose.orientation.w);
        tf2::Matrix3x3 m_obj(q_obj);
        double r, p, yaw_obj;
        m_obj.getRPY(r, p, yaw_obj);

        rclcpp::Rate loop_rate(20); 

        while (rclcpp::ok()) {
            // VERIFICACIÓN DE SEGURIDAD PROACTIVA
            if (last_health_status_) {
                if (!last_health_status_->imu_ok || !last_health_status_->motors_ok || !last_health_status_->odom_ok) {
                    stop_robot();
                    RCLCPP_ERROR(this->get_logger(), "ABORTO DE EMERGENCIA: %s", last_health_status_->status_message.c_str());
                    goal_handle->abort(result);
                    return;
                }
            }

            if (goal_handle->is_canceling()) {
                stop_robot();
                goal_handle->canceled(result);
                return;
            }

            double dx = goal->pose.pose.position.x - cur_x_;
            double dy = goal->pose.pose.position.y - cur_y_;
            double rho = std::sqrt(dx*dx + dy*dy);
            double alpha = normalize_angle(std::atan2(dy, dx) - cur_yaw_);
            double beta = normalize_angle(yaw_obj - cur_yaw_ - alpha);

            double v = K_rho * rho;
            double w = K_alpha * alpha + K_beta * beta;

            if (std::abs(alpha) > M_PI / 2.0) v = 0.0;

            if (rho < 0.05 && std::abs(normalize_angle(yaw_obj - cur_yaw_)) < 0.1) {
                stop_robot();
                goal_handle->succeed(result);
                break;
            }

            auto cmd = geometry_msgs::msg::Twist();
            //cmd.header.stamp = this->now();
            //cmd.header.frame_id = "base_link";
            cmd.linear.x = std::clamp(v, -max_v, max_v);
            cmd.angular.z = std::clamp(w, -max_w, max_w);
            cmd_pub_->publish(cmd);

            feedback->distance_remaining = rho;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
    }

    void stop_robot() {
        auto cmd = geometry_msgs::msg::Twist(); // CORRECCIÓN: Debe ser Twist, no TwistStamped
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        cur_x_ = msg->pose.pose.position.x;
        cur_y_ = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        cur_yaw_ = y;
        valid_odom_ = true;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolarActionServer>());
    rclcpp::shutdown();
    return 0;
}