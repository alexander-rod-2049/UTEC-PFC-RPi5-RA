#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "pfc_msgs/msg/system_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"

inline double sign(double x) { return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0); }

class SmcDflActionServer : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    SmcDflActionServer() : Node("smc_dfl_action_server") {
        this->declare_parameter("lambda1", 3.5);
        this->declare_parameter("lambda2", 3.5);
        this->declare_parameter("k1_sta", 1.5);
        this->declare_parameter("k2_sta", 0.8);
        this->declare_parameter("eps_v", 0.08);
        this->declare_parameter("max_v", 0.6); // Reducido para hardware real
        this->declare_parameter("max_w", 1.0);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&SmcDflActionServer::odom_callback, this, std::placeholders::_1));

        this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this, "smc_dfl_nav",
            std::bind(&SmcDflActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SmcDflActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SmcDflActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Servidor SMC+DFL (Hardware Ready) iniciado.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double cur_x_ = 0, cur_y_ = 0, cur_yaw_ = 0, cur_v_meas_ = 0;
    double z1_ = 0, z2_ = 0, last_w2_ = 0;

    void execute(const std::shared_ptr<GoalHandleNav> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        auto result = std::make_shared<NavigateToPose::Result>();

        // Parámetros de control
        double L1 = this->get_parameter("lambda1").as_double();
        double K1 = this->get_parameter("k1_sta").as_double();
        double EPS_V = this->get_parameter("eps_v").as_double();
        double MAX_V = this->get_parameter("max_v").as_double();
        double MAX_W = this->get_parameter("max_w").as_double();

        double x_start = cur_x_, y_start = cur_y_;
        double x_end = goal->pose.pose.position.x;
        double y_end = goal->pose.pose.position.y;

        // FSM interna: ALIGN -> MOVE
        enum class InternalState { ALIGN, MOVE };
        InternalState sub_state = InternalState::ALIGN;

        double t = 0.0;
        const double T_total = 15.0; // Duración de la trayectoria
        const double dt = 0.05;
        rclcpp::Rate loop_rate(20);

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                stop_robot();
                goal_handle->canceled(result);
                return;
            }

            double dx = x_end - cur_x_;
            double dy = y_end - cur_y_;
            double dist = std::sqrt(dx*dx + dy*dy);
            double target_yaw = std::atan2(dy, dx);
            double yaw_err = target_yaw - cur_yaw_;
            while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

            auto cmd = geometry_msgs::msg::Twist();

            if (sub_state == InternalState::ALIGN) {
                if (std::abs(yaw_err) < 0.1) {
                    sub_state = InternalState::MOVE;
                    t = 0; // Inicia tiempo de trayectoria
                } else {
                    cmd.angular.z = std::clamp(1.2 * yaw_err, -MAX_W, MAX_W);
                }
            } 
            else {
                // Generación de Trayectoria Quintic (Grado 5)
                double tau = std::min(t / T_total, 1.0);
                double s = 10 * std::pow(tau, 3) - 15 * std::pow(tau, 4) + 6 * std::pow(tau, 5);
                double ds = (30 * std::pow(tau, 2) - 60 * std::pow(tau, 3) + 30 * std::pow(tau, 4)) / T_total;
                double dds = (60 * tau - 180 * std::pow(tau, 2) + 120 * std::pow(tau, 3)) / (T_total * T_total);

                // Referencias Cartesianas
                double xd = x_start + (x_end - x_start) * s;
                double yd = y_start + (y_end - y_start) * s;
                double dxd = (x_end - x_start) * ds;
                double dyd = (y_end - y_start) * ds;
                double ddxd = (x_end - x_start) * dds;
                double ddyd = (y_end - y_start) * dds;

                // Ley SMC + DFL
                double e1 = cur_x_ - xd, e2 = cur_y_ - yd;
                double de1 = cur_v_meas_ * std::cos(cur_yaw_) - dxd;
                double de2 = cur_v_meas_ * std::sin(cur_yaw_) - dyd;
                double s1 = de1 + L1 * e1, s2 = de2 + L1 * e2;

                double term1 = (-ddxd + L1 * de1) + K1 * std::sqrt(std::abs(s1)) * sign(s1);
                double term2 = (-ddyd + L1 * de2) + K1 * std::sqrt(std::abs(s2)) * sign(s2);

                double v_safe = (std::abs(cur_v_meas_) < EPS_V) ? EPS_V : cur_v_meas_;
                double u_v = -(std::cos(cur_yaw_) * term1 + std::sin(cur_yaw_) * term2);
                double u_w = -(-std::sin(cur_yaw_) * term1 + std::cos(cur_yaw_) * term2) / v_safe;

                static double v_integral = 0;
                v_integral += u_v * dt;
                
                cmd.linear.x = std::clamp(v_integral, -MAX_V, MAX_V);
                cmd.angular.z = std::clamp(u_w, -MAX_W, MAX_W);

                if (dist < 0.15 && t > T_total) {
                    stop_robot();
                    goal_handle->succeed(result);
                    return;
                }
                t += dt;
            }

            cmd_pub_->publish(cmd);
            feedback->distance_remaining = dist;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
    }

    // Callbacks de boilerplate omitidos por brevedad...
    void stop_robot() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        cur_x_ = msg->pose.pose.position.x;
        cur_y_ = msg->pose.pose.position.y;
        cur_v_meas_ = msg->twist.twist.linear.x;
        tf2::Quaternion q; tf2::fromMsg(msg->pose.pose.orientation, q);
        double r, p, y; tf2::Matrix3x3(q).getRPY(r, p, y); cur_yaw_ = y;
    }
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNav>) { return rclcpp_action::CancelResponse::ACCEPT; }
    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle) { std::thread{std::bind(&SmcDflActionServer::execute, this, std::placeholders::_1), goal_handle}.detach(); }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmcDflActionServer>());
    rclcpp::shutdown();
    return 0;
}