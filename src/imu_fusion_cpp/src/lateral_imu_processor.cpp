#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <numeric>

class LateralIMUProcessor : public rclcpp::Node {
public:
    LateralIMUProcessor() : Node("lateral_imu_processor") {
        this->declare_parameter("side", "left");
        side_ = this->get_parameter("side").as_string();
        
        std::string imu_topic = (side_ == "left") ? "/imu/left/data" : "/imu/right/data";
        std::string out_topic = (side_ == "left") ? "/track/velocity/left" : "/track/velocity/right";

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&LateralIMUProcessor::imuCallback, this, std::placeholders::_1));
        
        central_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&LateralIMUProcessor::odomCallback, this, std::placeholders::_1));

        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(out_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Procesador (%s) Iniciado. MANTEN EL ROBOT QUIETO PARA CALIBRAR.", side_.c_str());
    }

private:
    double velocity_ = 0.0;
    double ref_vel_ = 0.0;
    rclcpp::Time last_time_;
    bool first_msg_ = true;
    std::string side_;
    
    // Calibración
    bool is_calibrated_ = false;
    std::vector<double> calib_buffer_;
    double accel_bias_ = 0.0;
    const size_t SAMPLES = 150; // ~3 Segundos

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr central_odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double v = msg->twist.twist.linear.x;
        double w = msg->twist.twist.angular.z;
        // ZUPT: Reset integral si estamos parados
        if (std::abs(v) < 0.01 && std::abs(w) < 0.01) velocity_ = 0.0;
        ref_vel_ = (side_ == "left") ? v - (w * 0.25) : v + (w * 0.25);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (first_msg_) { last_time_ = msg->header.stamp; first_msg_ = false; return; }
        double dt = (rclcpp::Time(msg->header.stamp) - last_time_).seconds();
        last_time_ = msg->header.stamp;

        // 1. Orientación
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 2. Compensación Gravedad
        double ax_no_g = msg->linear_acceleration.x - (9.81 * std::sin(pitch));

        // --- CALIBRACIÓN ---
        if (!is_calibrated_) {
            calib_buffer_.push_back(ax_no_g);
            if (calib_buffer_.size() >= SAMPLES) {
                double sum = std::accumulate(calib_buffer_.begin(), calib_buffer_.end(), 0.0);
                accel_bias_ = sum / SAMPLES;
                is_calibrated_ = true;
                RCLCPP_INFO(this->get_logger(), "CALIBRADO %s. Bias restado: %.3f", side_.c_str(), accel_bias_);
            }
            return;
        }

        // 3. Filtrado
        double ax_clean = ax_no_g - accel_bias_;
        if (std::abs(ax_clean) < 0.08) ax_clean = 0.0; // Dead-zone

        // 4. Integración Leaky
        velocity_ += ax_clean * dt;
        velocity_ = (0.98 * velocity_) + (0.02 * ref_vel_);

        // 5. Publicar
        auto out = geometry_msgs::msg::TwistStamped();
        out.header = msg->header;
        out.header.frame_id = "base_link";
        out.twist.linear.x = velocity_;
        out.twist.angular.z = msg->angular_velocity.z; 
        vel_pub_->publish(out);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LateralIMUProcessor>());
    rclcpp::shutdown();
    return 0;
}