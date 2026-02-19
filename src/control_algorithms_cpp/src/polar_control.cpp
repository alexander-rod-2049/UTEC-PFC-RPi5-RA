#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

// Normalizar ángulos entre -PI y PI (Función auxiliar vital)
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
        // IMPORTANTE: Nos suscribimos al tópico genérico "odom".
        // Para tu robot real, usarás remapping: --ros-args -r odom:=/odometry/filtered
        // Para Gazebo (simulación), el tópico por defecto ya es /odom, así que funcionará directo.
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, // "odom" es un nombre relativo, perfecto para remapping
            std::bind(&PolarControl::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&PolarControl::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🅿️ Nodo Polar Control iniciado. Objetivo: X=%.2f, Y=%.2f, Theta=%.2f", 
            target_x_, target_y_, target_theta_);
    }

private:
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;
    bool valid_odom_ = false;

    // --- LA META (Pose de Estacionamiento) ---
    double target_x_ = -1.0;
    double target_y_ = 0.0;
    double target_theta_ = 1.57; // 90 grados (mirando al "Norte" de Gazebo)

    // --- GANANCIAS (Sintonización Fina) ---
    // K_rho: Velocidad de acercamiento
    // K_alpha: Qué tanto le importa apuntar al objetivo
    // K_beta: Qué tanto le importa la orientación final
    double K_rho_ = 0.3;
    double K_alpha_ = 0.8;
    double K_beta_ = -0.15; // Debe ser negativo usualmente para estabilidad

    double max_v_ = 0.22; // Max velocidad TurtleBot3
    double max_w_ = 1.0;

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
        current_theta_ = yaw;
        
        if (!valid_odom_) valid_odom_ = true;
    }

    void control_loop()
    {
        if (!valid_odom_ || goal_reached_) return;

        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";

        // 1. Calcular diferenciales
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;

        // 2. Transformación a Coordenadas Polares
        double rho = std::sqrt(dx*dx + dy*dy); // Distancia
        double alpha = normalize_angle(std::atan2(dy, dx) - current_theta_); // Ángulo relativo meta
        double beta = normalize_angle(target_theta_ - current_theta_ - alpha); // Ángulo relativo orientación final

        // 3. Chequeo de dirección (¿Ir de frente o de reversa?)
        // Si la meta está atrás (alpha > 90 grados), invertimos la lógica para estacionar de reversa
        // o simplemente para que no gire 180 grados bruscamente.
        // Por simplicidad en este paso A, asumiremos avance frontal siempre (v > 0).
        // PERO, si alpha está entre PI/2 y -PI/2, vamos hacia adelante.
        
        double v = K_rho_ * rho;
        double w = K_alpha_ * alpha + K_beta_ * beta;

        // Si alpha es muy grande (estamos de espaldas), frenamos el avance para que gire en su sitio primero
        if (std::abs(alpha) > M_PI / 2) {
            v = 0.0; // Opcional: permitir solo giro si está muy desalineado
        }

        // 4. Parada
        if (rho < 0.05 && std::abs(normalize_angle(target_theta_ - current_theta_)) < 0.1) {
            RCLCPP_INFO(this->get_logger(), "🏁 ¡ESTACIONADO! Dist: %.3f, AngErr: %.3f", rho, target_theta_ - current_theta_);
            msg.twist.linear.x = 0.0;
            msg.twist.angular.z = 0.0;
            cmd_pub_->publish(msg);
            goal_reached_ = true;
            return;
        }

        // 5. Saturación segura
        msg.twist.linear.x = std::min(std::abs(v), max_v_); // Siempre positivo en este ejemplo simple
        msg.twist.angular.z = std::max(std::min(w, max_w_), -max_w_);

        // Logs
        static int log_counter = 0;
        if (log_counter++ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "ρ: %.2f | α: %.2f | β: %.2f | Cmd: [v=%.2f, w=%.2f]", 
                rho, alpha, beta, msg.twist.linear.x, msg.twist.angular.z);
        }

        cmd_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolarControl>());
    rclcpp::shutdown();
    return 0;
}