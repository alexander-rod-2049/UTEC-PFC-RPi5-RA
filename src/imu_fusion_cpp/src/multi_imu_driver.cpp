#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cmath>

#define TCA_ADDR 0x70
#define MPU_ADDR 0x68
#define MAG_ADDR 0x1E

class MultiIMUDriver : public rclcpp::Node {
public:
    MultiIMUDriver() : Node("multi_imu_driver") {
        pub_right_   = this->create_publisher<sensor_msgs::msg::Imu>("/imu/right/data", 10);
        pub_left_    = this->create_publisher<sensor_msgs::msg::Imu>("/imu/left/data", 10);
        pub_central_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/central/data", 10);
        pub_mag_     = this->create_publisher<sensor_msgs::msg::MagneticField>("/mag/gps/data", 10);

        char filename[] = "/dev/i2c-1"; 
        file_i2c_ = open(filename, O_RDWR);
        if (file_i2c_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "ERROR FATAL: No se abre I2C-1");
            return; // No iniciamos timer si no hay bus
        }

        // Intentamos inicializar, pero no bloqueamos si falla uno
        RCLCPP_INFO(this->get_logger(), "Inicializando sensores...");
        if(!initMPU(0)) RCLCPP_WARN(this->get_logger(), "Fallo init MPU Canal 0 (Der)");
        if(!initMPU(1)) RCLCPP_WARN(this->get_logger(), "Fallo init MPU Canal 1 (Izq)");
        if(!initMPU(2)) RCLCPP_WARN(this->get_logger(), "Fallo init MPU Canal 2 (Cen)");
        if(!initMag(3)) RCLCPP_WARN(this->get_logger(), "Fallo init MAG Canal 3");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MultiIMUDriver::readAllSensors, this));
        RCLCPP_INFO(this->get_logger(), "Driver Iniciado. Entrando al bucle...");
    }

private:
    int file_i2c_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_right_, pub_left_, pub_central_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Contador para no saturar el log
    int debug_counter_ = 0;

    bool selectChannel(int channel) {
        if (ioctl(file_i2c_, I2C_SLAVE, TCA_ADDR) < 0) return false;
        uint8_t data = (1 << channel);
        return (write(file_i2c_, &data, 1) == 1);
    }

    bool initMPU(int channel) {
        if(!selectChannel(channel)) return false;
        if (ioctl(file_i2c_, I2C_SLAVE, MPU_ADDR) < 0) return false;
        
        uint8_t wake[] = {0x6B, 0x00}; 
        if(write(file_i2c_, wake, 2) != 2) return false;
        
        uint8_t dlpf[] = {0x1A, 0x03}; 
        write(file_i2c_, dlpf, 2);
        return true;
    }

    bool initMag(int channel) {
        if(!selectChannel(channel)) return false;
        if (ioctl(file_i2c_, I2C_SLAVE, MAG_ADDR) < 0) return false;
        uint8_t buf[] = {0x02, 0x00}; 
        return (write(file_i2c_, buf, 2) == 2);
    }

    int16_t readWord2C(int reg) {
        uint8_t buf[] = {(uint8_t)reg};
        if(write(file_i2c_, buf, 1) != 1) return 0;
        if(read(file_i2c_, buf, 2) != 2) return 0;
        return (int16_t)((buf[0] << 8) + buf[1]);
    }

    void readAndPublishMPU(int channel, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub, std::string frame) {
        if(!selectChannel(channel)) return;
        if (ioctl(file_i2c_, I2C_SLAVE, MPU_ADDR) < 0) return;

        // Verificacion simple de conexion leyendo WHO_AM_I o similar (opcional)
        // Aquí leemos directo por velocidad
        
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = frame;

        double a_s = 16384.0; double g_s = 131.0;
        double G = 9.81; double RAD = M_PI/180.0;

        // Leemos todo el bloque de una vez sería mejor, pero mantenemos estructura simple
        // Si readWord2C devuelve 0 constante, el sensor está muerto o desconectado
        msg.linear_acceleration.x = (readWord2C(0x3B) / a_s) * G;
        msg.linear_acceleration.y = (readWord2C(0x3D) / a_s) * G;
        msg.linear_acceleration.z = (readWord2C(0x3F) / a_s) * G;

        msg.angular_velocity.x = (readWord2C(0x43) / g_s) * RAD;
        msg.angular_velocity.y = (readWord2C(0x45) / g_s) * RAD;
        msg.angular_velocity.z = (readWord2C(0x47) / g_s) * RAD;

        msg.linear_acceleration_covariance[0] = 0.05; 
        msg.angular_velocity_covariance[8] = 0.001; 

        pub->publish(msg);
    }

    void readAndPublishMag(int channel) {
        if(!selectChannel(channel)) return;
        if (ioctl(file_i2c_, I2C_SLAVE, MAG_ADDR) < 0) return;

        uint8_t reg = 0x03; 
        if(write(file_i2c_, &reg, 1) != 1) return;
        
        uint8_t buf[6];
        if (read(file_i2c_, buf, 6) != 6) return;

        int16_t x = (buf[0] << 8) | buf[1];
        int16_t z = (buf[2] << 8) | buf[3];
        int16_t y = (buf[4] << 8) | buf[5];

        auto msg = sensor_msgs::msg::MagneticField();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link_mag";
        
        double scale = 0.92e-7; 
        msg.magnetic_field.x = x * scale;
        msg.magnetic_field.y = y * scale;
        msg.magnetic_field.z = z * scale;
        msg.magnetic_field_covariance[0] = 0.01;

        pub_mag_->publish(msg);
    }

    void readAllSensors() {
        // Ejecutar lecturas
        readAndPublishMPU(0, pub_right_, "imu_link_right");
        readAndPublishMPU(1, pub_left_, "imu_link_left");
        readAndPublishMPU(2, pub_central_, "imu_link_central");
        readAndPublishMag(3);

        // DEBUG: Imprimir cada 50 ciclos (aprox 1 segundo) para saber que sigue vivo
        debug_counter_++;
        if (debug_counter_ >= 50) {
            // RCLCPP_INFO(this->get_logger(), "Heartbeat: Leyendo sensores OK..."); 
            // Descomenta la linea de arriba si quieres ver spam en la terminal para confirmar vida
            debug_counter_ = 0;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiIMUDriver>());
    rclcpp::shutdown();
    return 0;
}