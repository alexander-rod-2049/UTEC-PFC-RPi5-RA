#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include <cstring>
#include <cmath>
#include <iostream>
#include <algorithm>

class MTF02POdometry : public rclcpp::Node
{
public:
    MTF02POdometry() : Node("mtf02p_odometry_node")
    {
        // Declarar parámetros
        this->declare_parameter<std::string>("port", "/dev/flow_mtf02p");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("offset_mm", 41);
        this->declare_parameter<std::string>("frame_id", "odom");
        this->declare_parameter<std::string>("child_frame_id", "base_link");

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        offset_mm_ = this->get_parameter("offset_mm").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();

        // Abrir puerto serie
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el puerto %s", port_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Configurar termios
        struct termios options;
        tcgetattr(fd_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1;
        tcsetattr(fd_, TCSANOW, &options);
        tcflush(fd_, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(), "Conectado a %s a %d baudios", port_.c_str(), baudrate_);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("sensor/odom", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MTF02POdometry::readSerial, this));
        last_time_ = this->now();

        // Inicializar variables de velocidad previa
        vel_x_prev_ = 0.0;
        vel_y_prev_ = 0.0;
    }

    ~MTF02POdometry()
    {
        if (fd_ != -1) close(fd_);
    }

private:
    void readSerial()
    {
        uint8_t buf[256];
        int n = read(fd_, buf, sizeof(buf));
        if (n > 0) {
            buffer_.insert(buffer_.end(), buf, buf + n);
        }

        while (buffer_.size() >= 7) {
            auto it = std::find(buffer_.begin(), buffer_.end(), 0xEF);
            if (it == buffer_.end()) {
                buffer_.clear();
                break;
            }
            if (it != buffer_.begin()) {
                buffer_.erase(buffer_.begin(), it);
            }
            if (buffer_.size() < 7) break;
            uint8_t payload_len = buffer_[5];
            uint16_t total_len = payload_len + 7;
            if (buffer_.size() < total_len) break;

            std::vector<uint8_t> payload(buffer_.begin() + 6, buffer_.begin() + 6 + payload_len);
            decodePayload(payload);
            buffer_.erase(buffer_.begin(), buffer_.begin() + total_len);
        }
    }

    void decodePayload(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 20) return;

        uint32_t contador = *reinterpret_cast<const uint32_t*>(&payload[0]);
        uint32_t dist_medida = *reinterpret_cast<const uint32_t*>(&payload[4]);
        int16_t flow_raw_x = *reinterpret_cast<const int16_t*>(&payload[12]);
        int16_t flow_raw_y = *reinterpret_cast<const int16_t*>(&payload[14]);
        uint8_t calidad = payload[16];

        double altura_real_mm = static_cast<double>(dist_medida) + offset_mm_;
        double altura_m = altura_real_mm / 1000.0;

        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double vel_x, vel_y;
        if (calidad > 50) {
            double vel_x_cm = flow_raw_x * altura_m;
            double vel_y_cm = flow_raw_y * altura_m;
            vel_x = vel_x_cm / 100.0;
            vel_y = -(vel_y_cm / 100.0);
            // Actualizar valores previos solo cuando la calidad es buena
            vel_x_prev_ = vel_x;
            vel_y_prev_ = vel_y;
        } else {
            // Usar el último valor válido
            vel_x = vel_x_prev_;
            vel_y = vel_y_prev_;
            // No actualizamos prev_ con estos valores porque son repeticiones
        }

        // Crear mensaje de odometría
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = frame_id_;
        odom_msg.child_frame_id = child_frame_id_;

        // Asignar velocidades
        odom_msg.twist.twist.linear.x = vel_x;
        odom_msg.twist.twist.linear.y = vel_y;

        // Configurar covarianza del twist
        // Inicializar toda la covarianza a cero
        for (int i = 0; i < 36; ++i) {
            odom_msg.twist.covariance[i] = 0.0;
        }
        // Asignar varianzas para las velocidades lineales
        odom_msg.twist.covariance[0] = 0.08;   // vx (menor confianza: 0.08 ≈ 0.283 m/s de desviación)
        odom_msg.twist.covariance[1] = 0.04;   // vy (0.2 m/s de desviación, más confianza)
        // Velocidades angulares y lineal z (no medidas) con varianza muy alta
        odom_msg.twist.covariance[2] = 1e6;    // vz
        odom_msg.twist.covariance[3] = 1e6;    // wx
        odom_msg.twist.covariance[4] = 1e6;    // wy
        odom_msg.twist.covariance[5] = 1e6;    // wz

        // (Opcional) Covarianza de pose: si no se usa, poner valores altos
        for (int i = 0; i < 36; ++i) {
            odom_msg.pose.covariance[i] = 1e6;
        }

        odom_pub_->publish(odom_msg);

        // Para depuración (opcional):
        // RCLCPP_INFO(this->get_logger(), "H: %.1f mm, Vx: %.3f m/s, Vy: %.3f m/s, Qual: %d",
        //             altura_real_mm, vel_x, vel_y, calidad);
    }

    int fd_;
    std::vector<uint8_t> buffer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string port_;
    int baudrate_;
    int offset_mm_;
    std::string frame_id_;
    std::string child_frame_id_;
    rclcpp::Time last_time_;

    // Nuevas variables para guardar velocidades previas
    double vel_x_prev_;
    double vel_y_prev_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MTF02POdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}