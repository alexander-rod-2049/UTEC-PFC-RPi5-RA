#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

class SlipEstimatorNode : public rclcpp::Node {
public:
    SlipEstimatorNode() : Node("slip_estimator_node") {
        this->declare_parameter("track_width", 0.55); 
        width_ = this->get_parameter("track_width").as_double();

        sub_enc_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/encoders", 10, std::bind(&SlipEstimatorNode::encCb, this, std::placeholders::_1));
        sub_ekf_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&SlipEstimatorNode::ekfCb, this, std::placeholders::_1));
        sub_lat_L_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/track/velocity/left", 10, std::bind(&SlipEstimatorNode::latLCb, this, std::placeholders::_1));
        sub_lat_R_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/track/velocity/right", 10, std::bind(&SlipEstimatorNode::latRCb, this, std::placeholders::_1));

        pub_slip_L_ = this->create_publisher<std_msgs::msg::Float64>("/slip/left", 10);
        pub_slip_R_ = this->create_publisher<std_msgs::msg::Float64>("/slip/right", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SlipEstimatorNode::loop, this));
        RCLCPP_INFO(this->get_logger(), "Estimador Slip OK.");
    }

private:
    double v_enc_L=0, v_enc_R=0, v_lat_L=0, v_lat_R=0;
    double width_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_enc_, sub_ekf_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_lat_L_, sub_lat_R_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_slip_L_, pub_slip_R_;
    rclcpp::TimerBase::SharedPtr timer_;

    void encCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double v = msg->twist.twist.linear.x;
        double w = msg->twist.twist.angular.z;
        v_enc_L = v - (w * width_/2.0);
        v_enc_R = v + (w * width_/2.0);
    }
    void ekfCb(const nav_msgs::msg::Odometry::SharedPtr msg) {} // Solo referencia si se necesita
    void latLCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { v_lat_L = msg->twist.linear.x; }
    void latRCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { v_lat_R = msg->twist.linear.x; }

    void loop() {
        double v_thresh = 0.05; // Umbral mínimo de velocidad (5 cm/s)

        // SLIP IZQUIERDA
        double slip_L = 0.0;
        if (std::abs(v_enc_L) > v_thresh) {
            slip_L = 1.0 - (v_lat_L / v_enc_L);
        }
        
        // SLIP DERECHA
        double slip_R = 0.0;
        if (std::abs(v_enc_R) > v_thresh) {
            slip_R = 1.0 - (v_lat_R / v_enc_R);
        }
        
        // SATURACIÓN (Para que no salga -500 o NaN en gráficas)
        // Slip físico real está entre 0 y 1. Permitimos un margen por ruido.
        if (slip_L > 1.0) slip_L = 1.0; 
        if (slip_L < -0.5) slip_L = -0.5;

        if (slip_R > 1.0) slip_R = 1.0; 
        if (slip_R < -0.5) slip_R = -0.5;

        // Publicar
        std_msgs::msg::Float64 m;
        m.data = slip_L; pub_slip_L_->publish(m);
        m.data = slip_R; pub_slip_R_->publish(m);
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlipEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}