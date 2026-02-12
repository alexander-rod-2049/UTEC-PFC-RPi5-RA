#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp" // NUEVO
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class PointControlNode : public rclcpp::Node
{
public:
  PointControlNode()
  : Node("point_control_node")
  {
    // ... (Tus parámetros existentes) ...
    this->declare_parameter("target_x", 1.0);
    this->declare_parameter("target_y", 0.0);
    this->declare_parameter("kp_linear", 0.5);
    this->declare_parameter("kp_angular", 2.0);
    this->declare_parameter("max_linear_vel", 0.5);
    this->declare_parameter("max_angular_vel", 1.0);
    this->declare_parameter("stop_tolerance", 0.05);

    target_x_ = this->get_parameter("target_x").as_double();
    target_y_ = this->get_parameter("target_y").as_double();
    kp_linear_ = this->get_parameter("kp_linear").as_double();
    kp_angular_ = this->get_parameter("kp_angular").as_double();
    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    stop_tolerance_ = this->get_parameter("stop_tolerance").as_double();

    // Suscripciones
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/sigyn/wheel_odom", 10, 
      std::bind(&PointControlNode::odom_callback, this, std::placeholders::_1));

    // Publicadores
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // NUEVO: Publicador de la meta actual para el logger
    pub_current_goal_ = this->create_publisher<geometry_msgs::msg::Point>("/current_goal", 10);

    // Timer
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PointControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Control Polar. Meta: (%.2f, %.2f)", target_x_, target_y_);
  }

private:
  // ... (Variables existentes) ...
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  bool odom_received_ = false;

  double target_x_, target_y_;
  double kp_linear_, kp_angular_;
  double max_linear_vel_, max_angular_vel_;
  double stop_tolerance_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_current_goal_; // NUEVO
  rclcpp::TimerBase::SharedPtr timer_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    odom_received_ = true;
  }

  void control_loop()
  {
    // NUEVO: Publicar siempre la meta actual
    geometry_msgs::msg::Point goal_msg;
    goal_msg.x = target_x_;
    goal_msg.y = target_y_;
    goal_msg.z = 0.0;
    pub_current_goal_->publish(goal_msg);

    if (!odom_received_) return;

    // ... (Lógica de control existente: Distancia, Angulo, Twist) ...
    geometry_msgs::msg::Twist cmd;
    double dx = target_x_ - current_x_;
    double dy = target_y_ - current_y_;
    double distance_error = std::sqrt(dx*dx + dy*dy);
    double desired_yaw = std::atan2(dy, dx);
    double yaw_error = desired_yaw - current_yaw_;
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    if (distance_error > stop_tolerance_) {
      double angular_vel = kp_angular_ * yaw_error;
      double linear_vel = 0.0;
      if (std::abs(yaw_error) < (M_PI / 2.0)) {
         linear_vel = kp_linear_ * distance_error;
      }
      cmd.linear.x = std::min(linear_vel, max_linear_vel_);
      cmd.angular.z = std::max(std::min(angular_vel, max_angular_vel_), -max_angular_vel_);
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    pub_cmd_vel_->publish(cmd);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointControlNode>());
  rclcpp::shutdown();
  return 0;
}