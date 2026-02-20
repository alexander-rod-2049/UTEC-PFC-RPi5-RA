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
#include "pfc_msgs/msg/system_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

inline double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

class SmcDflActionServer : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    SmcDflActionServer() : Node("smc_dfl_action_server")
    {
        // Declarar parámetros (coincidentes con tu params.yaml)
        this->declare_parameter("lambda1", 4.0);
        this->declare_parameter("lambda2", 4.0);
        this->declare_parameter("k1", 1.1);
        this->declare_parameter("k2", 1.1);
        this->declare_parameter("eps_v", 0.02);
        this->declare_parameter("max_v", 0.22);
        this->declare_parameter("max_w", 2.0);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&SmcDflActionServer::odom_callback, this, std::placeholders::_1));

        health_sub_ = this->create_subscription<pfc_msgs::msg::SystemStatus>(
            "/system_status", 10,
            [this](const pfc_msgs::msg::SystemStatus::SharedPtr msg) { last_health_status_ = msg; });

        this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "smc_dfl_nav",
            std::bind(&SmcDflActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SmcDflActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SmcDflActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Servidor Action SMC+DFL con Seguridad Proactiva iniciado.");
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<pfc_msgs::msg::SystemStatus>::SharedPtr health_sub_;

    double cur_x_ = 0.0, cur_y_ = 0.0, cur_yaw_ = 0.0, cur_v_meas_ = 0.0;
    bool valid_odom_ = false;
    pfc_msgs::msg::SystemStatus::SharedPtr last_health_status_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        (void)uuid; (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        std::thread{std::bind(&SmcDflActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        auto result = std::make_shared<NavigateToPose::Result>();

        // Cargar parámetros dinámicamente al inicio
        double L1 = this->get_parameter("lambda1").as_double();
        double L2 = this->get_parameter("lambda2").as_double();
        double K1 = this->get_parameter("k1").as_double();
        double K2 = this->get_parameter("k2").as_double();
        double EPS_V = this->get_parameter("eps_v").as_double();
        double max_v = this->get_parameter("max_v").as_double();
        double max_w = this->get_parameter("max_w").as_double();

        double v_cmd = 0.0;
        double t = 0.0;
        const double dt = 0.02; // 50Hz
        rclcpp::Rate loop_rate(50);

        while (rclcpp::ok()) {
            // SEGURIDAD PROACTIVA
            if (last_health_status_ && (!last_health_status_->imu_ok || !last_health_status_->motors_ok)) {
                stop_robot();
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "EMERGENCIA: Abortando SMC por fallo de sensores.");
                return;
            }

            if (goal_handle->is_canceling()) {
                stop_robot();
                goal_handle->canceled(result);
                return;
            }

            // 1. Trayectoria Circular
            double R = 1.0; double w_t = 0.15;
            double xd = R * std::sin(w_t * t);
            double yd = R * (1.0 - std::cos(w_t * t));
            double dxd = R * w_t * std::cos(w_t * t);
            double dyd = R * w_t * std::sin(w_t * t);
            double ddxd = -R * w_t * w_t * std::sin(w_t * t);
            double ddyd = R * w_t * w_t * std::cos(w_t * t);

            // 2. Errores
            double e1 = cur_x_ - xd;
            double e2 = cur_y_ - yd;
            double de1 = cur_v_meas_ * std::cos(cur_yaw_) - dxd;
            double de2 = cur_v_meas_ * std::sin(cur_yaw_) - dyd;

            // 3. SMC + DFL
            double s1 = de1 + L1 * e1;
            double s2 = de2 + L2 * e2;
            double Phi1 = -ddxd + L1 * de1;
            double Phi2 = -ddyd + L2 * de2;
            double term1 = Phi1 + K1 * sign(s1);
            double term2 = Phi2 + K2 * sign(s2);

            double w1 = -(std::cos(cur_yaw_) * term1 + std::sin(cur_yaw_) * term2);
            double v_safe = (std::abs(cur_v_meas_) < EPS_V) ? ((cur_v_meas_ < 0) ? -EPS_V : EPS_V) : cur_v_meas_;
            double w2 = -(-std::sin(cur_yaw_) * term1 + std::cos(cur_yaw_) * term2) / v_safe;

            v_cmd += w1 * dt;

            // 4. Publicar
            auto cmd = geometry_msgs::msg::TwistStamped();
            cmd.header.stamp = this->now();
            cmd.header.frame_id = "base_link";
            cmd.twist.linear.x = std::clamp(v_cmd, -max_v, max_v);
            cmd.twist.angular.z = std::clamp(w2, -max_w, max_w);
            cmd_pub_->publish(cmd);

            if (t > 45.0) { // Fin de la trayectoria
                stop_robot();
                goal_handle->succeed(result);
                break;
            }

            feedback->distance_remaining = std::sqrt(e1*e1 + e2*e2);
            goal_handle->publish_feedback(feedback);

            t += dt;
            loop_rate.sleep();
        }
    }

    void stop_robot() {
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->now();
        cmd_pub_->publish(cmd);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        cur_x_ = msg->pose.pose.position.x;
        cur_y_ = msg->pose.pose.position.y;
        cur_v_meas_ = msg->twist.twist.linear.x;
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
    rclcpp::spin(std::make_shared<SmcDflActionServer>());
    rclcpp::shutdown();
    return 0;
}