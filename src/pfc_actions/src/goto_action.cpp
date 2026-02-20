#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class GoToActionServer : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    GoToActionServer() : Node("goto_action_server")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&GoToActionServer::odom_callback, this, std::placeholders::_1));

        this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&GoToActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GoToActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&GoToActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Action Server de Navegacion iniciado.");
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double cur_x_ = 0.0, cur_y_ = 0.0;
    bool valid_odom_ = false;

    // --- Funciones del Servidor ---

    // CAMBIO AQUI: rclcpp_action::GoalUUID en lugar de GoalID
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Nueva meta recibida: X=%.2f", goal->pose.pose.position.x);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancelacion solicitada.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        std::thread{std::bind(&GoToActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Ejecutando movimiento...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        auto result = std::make_shared<NavigateToPose::Result>();
        
        rclcpp::Rate loop_rate(20); 

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling()) {
                stop_robot();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Accion cancelada.");
                return;
            }

            double dx = goal->pose.pose.position.x - cur_x_;
            double dy = goal->pose.pose.position.y - cur_y_;
            double distance = std::sqrt(dx*dx + dy*dy);

            feedback->distance_remaining = distance;
            goal_handle->publish_feedback(feedback);

            if (distance < 0.1) {
                stop_robot();
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Meta alcanzada con exito.");
                break;
            }

            auto cmd = geometry_msgs::msg::TwistStamped();
            cmd.header.stamp = this->now();
            cmd.header.frame_id = "base_link";
            cmd.twist.linear.x = std::min(0.2, distance); 
            cmd_pub_->publish(cmd);

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
        valid_odom_ = true;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}