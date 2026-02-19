#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

// Función Signo para el SMC
inline double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

class SmcDflControl : public rclcpp::Node
{
public:
    SmcDflControl() : Node("smc_dfl_control_node")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", rclcpp::SensorDataQoS(), 
            std::bind(&SmcDflControl::odom_callback, this, std::placeholders::_1));

        // Bucle a 50Hz (20ms) como en tu ESP32
        dt_ = 0.02; 
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SmcDflControl::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🌀 Nodo SMC+DFL iniciado. Trayectoria: Círculo");
    }

private:
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;
    double current_v_meas_ = 0.0; // Velocidad real medida (x4)
    
    double v_cmd_ = 0.0; // La velocidad lineal que integramos (ul)
    double t_ = 0.0;     // Tiempo de la trayectoria
    double dt_;
    bool valid_odom_ = false;

    // --- PARÁMETROS DEL SMC ---
    double lambda1_ = 4.0;
    double lambda2_ = 4.0;
    double K1_ = 1.1;
    double K2_ = 1.1;
    double EPS_V_ = 0.02;

    double max_v_ = 0.22; // Max velocidad TurtleBot3
    double max_w_ = 2.0;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // La velocidad lineal que el simulador nos reporta
        current_v_meas_ = msg->twist.twist.linear.x;

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
        if (!valid_odom_) return;

        // 1. Generador de Trayectoria (CÍRCULO)
        double R = 1.0;          // Radio de 1 metro
        double w_t = 0.15;      // 2*pi / 20 segundos por vuelta

        double xd   = R * std::sin(w_t * t_);
        double yd   = R * (1.0 - std::cos(w_t * t_));
        
        double dxd  = R * w_t * std::cos(w_t * t_);
        double dyd  = R * w_t * std::sin(w_t * t_);
        
        double ddxd = -R * w_t * w_t * std::sin(w_t * t_);
        double ddyd =  R * w_t * w_t * std::cos(w_t * t_);

        // 2. Errores
        double e1 = current_x_ - xd;
        double e2 = current_y_ - yd;

        // 3. Dinámica del error (con la velocidad real)
        double de1 = current_v_meas_ * std::cos(current_theta_) - dxd;
        double de2 = current_v_meas_ * std::sin(current_theta_) - dyd;

        // 4. Superficies Deslizantes (Sliding Surfaces)
        double s1 = de1 + lambda1_ * e1;
        double s2 = de2 + lambda2_ * e2;

        // 5. Ley de Control (DFL + SMC)
        double Phi1 = -ddxd + lambda1_ * de1;
        double Phi2 = -ddyd + lambda2_ * de2;

        double term1 = Phi1 + K1_ * sign(s1);
        double term2 = Phi2 + K2_ * sign(s2);

        // w1 es la derivada de la velocidad lineal (aceleración)
        double w1 = -( std::cos(current_theta_) * term1 + std::sin(current_theta_) * term2 );
        
        // Safe Division para evitar singularidad en DFL
        double v_safe = current_v_meas_;
        if (std::abs(v_safe) < EPS_V_) {
            v_safe = (v_safe < 0.0) ? -EPS_V_ : EPS_V_;
        }

        // w2 es la velocidad angular
        double w2 = -( -std::sin(current_theta_) * term1 + std::cos(current_theta_) * term2 ) / v_safe;

        // 6. Integración del estado extendido (v_cmd = integral(w1))
        v_cmd_ += w1 * dt_;

        // 7. Saturaciones y publicación
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";

        msg.twist.linear.x = std::clamp(v_cmd_, -max_v_, max_v_);
        msg.twist.angular.z = std::clamp(w2, -max_w_, max_w_);

        cmd_pub_->publish(msg);

        // Actualizamos el tiempo para el siguiente ciclo
        t_ += dt_;

        // Logs
        static int log_counter = 0;
        if (log_counter++ % 25 == 0) { // 25 ciclos = 0.5s
            RCLCPP_INFO(this->get_logger(), 
                "T: %.1fs | Err: (%.2f, %.2f) | cmd_v: %.2f | cmd_w: %.2f", 
                t_, e1, e2, msg.twist.linear.x, msg.twist.angular.z);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmcDflControl>());
    rclcpp::shutdown();
    return 0;
}