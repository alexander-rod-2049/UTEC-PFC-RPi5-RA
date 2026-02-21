#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "pfc_msgs/msg/system_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"

// Función signo estándar
inline double sign(double x) {
    return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);
}

class SmcDflActionServer : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    SmcDflActionServer() : Node("smc_dfl_action_server")
    {
        // 1. Declaración de parámetros
        this->declare_parameter("lambda1", 3.5);
        this->declare_parameter("lambda2", 3.5);
        this->declare_parameter("k1_sta", 1.5);
        this->declare_parameter("k2_sta", 0.8);
        this->declare_parameter("eps_v", 0.08); // Umbral de protección v_safe
        this->declare_parameter("max_v", 1.0);
        this->declare_parameter("max_w", 1.2);

        // 2. Publicadores y Suscriptores
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&SmcDflActionServer::odom_callback, this, std::placeholders::_1));

        health_sub_ = this->create_subscription<pfc_msgs::msg::SystemStatus>(
            "/system_status", 10,
            [this](const pfc_msgs::msg::SystemStatus::SharedPtr msg) { last_health_status_ = msg; });

        // 3. Servidor de Acción
        this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this, "smc_dfl_nav",
            std::bind(&SmcDflActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SmcDflActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SmcDflActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Servidor SMC+DFL (STA + Quintic Path) iniciado.");
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<pfc_msgs::msg::SystemStatus>::SharedPtr health_sub_;

    // Variables de estado del robot
    double cur_x_ = 0.0, cur_y_ = 0.0, cur_yaw_ = 0.0, cur_v_meas_ = 0.0;
    pfc_msgs::msg::SystemStatus::SharedPtr last_health_status_;

    // Estados internos del controlador STA y Filtro
    double z1_ = 0.0, z2_ = 0.0;
    double last_w2_ = 0.0;

    // Callbacks del Servidor de Acción
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal)
    { (void)uuid; (void)goal; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNav> goal_handle)
    { (void)goal_handle; return rclcpp_action::CancelResponse::ACCEPT; }

    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    { std::thread{std::bind(&SmcDflActionServer::execute, this, std::placeholders::_1), goal_handle}.detach(); }

    void execute(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        auto result = std::make_shared<NavigateToPose::Result>();

        // Carga de parámetros al inicio de la misión
        double L1 = this->get_parameter("lambda1").as_double();
        double L2 = this->get_parameter("lambda2").as_double();
        double K1 = this->get_parameter("k1_sta").as_double();
        double K2 = this->get_parameter("k2_sta").as_double();
        double EPS_V = this->get_parameter("eps_v").as_double();
        double MAX_V = this->get_parameter("max_v").as_double();
        double MAX_W = this->get_parameter("max_w").as_double();

        double x_start = cur_x_, y_start = cur_y_;
        double x_end = goal->pose.pose.position.x;
        double y_end = goal->pose.pose.position.y;

        // Reset de integradores y filtro
        z1_ = 0.0; z2_ = 0.0; last_w2_ = 0.0;
        double v_cmd = 0.0;
        double t = 0.0;
        const double T_total = 30.0; // Trayectoria lenta para vencer stiction
        const double dt = 0.05; // 20Hz
        rclcpp::Rate loop_rate(20);

        RCLCPP_INFO(this->get_logger(), "Iniciando seguimiento de trayectoria STA...");

        while (rclcpp::ok()) {
            // SEGURIDAD: Abortar si hay fallos reportados por el HealthNode
            if (last_health_status_ && (!last_health_status_->imu_ok || !last_health_status_->motors_ok)) {
                stop_robot();
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Aborto por seguridad: Fallo en sensores/motores.");
                return;
            }

            if (goal_handle->is_canceling()) {
                stop_robot();
                goal_handle->canceled(result);
                return;
            }

            // 1. Generación de Trayectoria Quintic (Polinomio de 5to Grado)
            double tau = std::min(t / T_total, 1.0);
            double s = 10 * std::pow(tau, 3) - 15 * std::pow(tau, 4) + 6 * std::pow(tau, 5);
            double ds = (30 * std::pow(tau, 2) - 60 * std::pow(tau, 3) + 30 * std::pow(tau, 4)) / T_total;
            double dds = (60 * tau - 180 * std::pow(tau, 2) + 120 * std::pow(tau, 3)) / (T_total * T_total);

            double xd = x_start + (x_end - x_start) * s;
            double yd = y_start + (y_end - y_start) * s;
            double dxd = (x_end - x_start) * ds;
            double dyd = (y_end - y_start) * ds;
            double ddxd = (x_end - x_start) * dds;
            double ddyd = (y_end - y_start) * dds;

            // 2. Errores y Superficies de Deslizamiento (SMC)
            double e1 = cur_x_ - xd;
            double e2 = cur_y_ - yd;
            double de1 = cur_v_meas_ * std::cos(cur_yaw_) - dxd;
            double de2 = cur_v_meas_ * std::sin(cur_yaw_) - dyd;
            double s1 = de1 + L1 * e1;
            double s2 = de2 + L2 * e2;

            // 3. Ley de Control Super-Twisting (Compensador de Perturbaciones)
            z1_ += -K2 * sign(s1) * dt;
            z2_ += -K2 * sign(s2) * dt;
            double Phi1 = -ddxd + L1 * de1;
            double Phi2 = -ddyd + L2 * de2;
            double term1 = Phi1 + K1 * std::sqrt(std::abs(s1)) * sign(s1) - z1_;
            double term2 = Phi2 + K2 * std::sqrt(std::abs(s2)) * sign(s2) - z2_;

            // 4. Dynamic Feedback Linearization (DFL) con Protección
            double v_safe = (std::abs(cur_v_meas_) < EPS_V) ? EPS_V : cur_v_meas_;
            double w1 = -(std::cos(cur_yaw_) * term1 + std::sin(cur_yaw_) * term2);
            double target_w = -(-std::sin(cur_yaw_) * term1 + std::cos(cur_yaw_) * term2) / v_safe;

            // Filtro Paso Bajo para w2 (Suaviza el giro de las orugas)
            double w2 = 0.7 * last_w2_ + 0.3 * target_w; 
            last_w2_ = w2;
            
            v_cmd += w1 * dt;

            // 5. Publicar Mensaje Twist
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = std::clamp(v_cmd, -MAX_V, MAX_V);
            cmd.angular.z = std::clamp(w2, -MAX_W, MAX_W);
            cmd_pub_->publish(cmd);

            // Condición de éxito basada en error de posición
            double error_dist = std::sqrt(e1*e1 + e2*e2);
            if (error_dist < 0.1 && t > T_total) {
                stop_robot();
                goal_handle->succeed(result);
                break;
            }

            feedback->distance_remaining = error_dist;
            goal_handle->publish_feedback(feedback);
            t += dt;
            loop_rate.sleep();
        }
    }

    void stop_robot() {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.0; cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        cur_x_ = msg->pose.pose.position.x;
        cur_y_ = msg->pose.pose.position.y;
        cur_v_meas_ = msg->twist.twist.linear.x;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y; m.getRPY(r, p, y); cur_yaw_ = y;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmcDflActionServer>());
    rclcpp::shutdown();
    return 0;
}