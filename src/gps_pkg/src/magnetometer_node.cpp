#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cmath>
#include <iostream>

#define TCA9548A_ADDR 0x70
#define MAG_ADDR_HMC 0x1E
#define MAG_ADDR_QMC 0x0D
#define CHANNEL 3

enum MagType { UNKNOWN, HMC5883L, QMC5883L };

class MagnetometerNode : public rclcpp::Node
{
public:
    MagnetometerNode() : Node("magnetometer_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("gps/mag", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MagnetometerNode::timer_callback, this));
            
        init_i2c();
    }

    ~MagnetometerNode()
    {
        if (i2c_fd_ >= 0) close(i2c_fd_);
    }

private:
    void init_i2c()
    {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-1");
        i2c_fd_ = open(filename, O_RDWR);
        
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error abriendo bus I2C: %s", filename);
            return;
        }

        // 1. Configurar Multiplexor
        select_multiplexer_channel(CHANNEL);
        
        // 2. Detectar y Configurar Magnetómetro
        detect_and_configure();
    }

    void select_multiplexer_channel(uint8_t channel)
    {
        if (ioctl(i2c_fd_, I2C_SLAVE, TCA9548A_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error conectando con Multiplexor 0x70");
            return;
        }
        uint8_t data = (1 << channel);
        write(i2c_fd_, &data, 1);
    }

    void detect_and_configure()
    {
        // Intento 1: QMC5883L (0x0D)
        if (check_device(MAG_ADDR_QMC)) {
            // Leer ID (Reg 0x0D)
            uint8_t id = read_reg(0x0D);
            if (id == 0xFF) {
                mag_type_ = QMC5883L;
                current_addr_ = MAG_ADDR_QMC;
                RCLCPP_INFO(this->get_logger(), "✅ Detectado QMC5883L en 0x0D (ID: 0xFF)");
                
                // Config QMC: Reg 0x09 (Control 1) -> 0x1D (Mode Continuous, 200Hz, 8G, OSR 512)
                // Reg 0x0B (SET/RESET) -> 0x01
                write_reg(0x0B, 0x01);
                write_reg(0x09, 0x1D); 
                return;
            }
        }

        // Intento 2: HMC5883L (0x1E)
        if (check_device(MAG_ADDR_HMC)) {
            mag_type_ = HMC5883L;
            current_addr_ = MAG_ADDR_HMC;
            
            // Leer ID A (0x0A) - Debería ser 'H' (0x48)
            uint8_t id = read_reg(0x0A);
            RCLCPP_INFO(this->get_logger(), "✅ Detectado dispositivo en 0x1E (ID Reg A: 0x%02X). Configurando como HMC5883L.", id);
            
            // Configurar HMC Estándar
            write_reg(0x00, 0x70); // 8-average, 15 Hz
            write_reg(0x01, 0xA0); // Gain 5
            write_reg(0x02, 0x00); // Continuous
            return;
        }
        
        RCLCPP_ERROR(this->get_logger(), "❌ No se detectó ningún magnetómetro conocido (HMC5883L o QMC5883L)");
    }
    
    bool check_device(uint8_t addr) {
        if (ioctl(i2c_fd_, I2C_SLAVE, addr) < 0) return false;
        // Intentar leer registro 0x00 para ver si responde (ACK)
        uint8_t reg = 0x00;
        if (write(i2c_fd_, &reg, 1) < 0) return false;
        return true;
    }
    
    void write_reg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = {reg, val};
        write(i2c_fd_, buf, 2);
    }
    
    uint8_t read_reg(uint8_t reg) {
        write(i2c_fd_, &reg, 1);
        uint8_t val = 0;
        read(i2c_fd_, &val, 1);
        return val;
    }

    void timer_callback()
    {
        if (i2c_fd_ < 0 || mag_type_ == UNKNOWN) return;
        
        if (ioctl(i2c_fd_, I2C_SLAVE, current_addr_) < 0) return;
        
        int16_t x=0, y=0, z=0;
        uint8_t data[6];
        
        if (mag_type_ == HMC5883L) {
            // HMC: X Z Y (MSB first)
            // Regs 0x03..0x08
            uint8_t reg = 0x03;
            write(i2c_fd_, &reg, 1);
            if (read(i2c_fd_, data, 6) == 6) {
                x = (int16_t)((data[0] << 8) | data[1]);
                z = (int16_t)((data[2] << 8) | data[3]);
                y = (int16_t)((data[4] << 8) | data[5]);
            } else {
                 return; // Error lectura silencioso para no spammear
            }
        } else if (mag_type_ == QMC5883L) {
            // QMC: X Y Z (LSB first)
            // Regs 0x00..0x05
            uint8_t reg = 0x00;
            write(i2c_fd_, &reg, 1);
            if (read(i2c_fd_, data, 6) == 6) {
                x = (int16_t)((data[1] << 8) | data[0]);
                y = (int16_t)((data[3] << 8) | data[2]);
                z = (int16_t)((data[5] << 8) | data[4]);
            } else {
                return;
            }
        }
        
        // Publicar
        float scale = 0.92e-7; // Escala aprox Tesla
        auto msg = sensor_msgs::msg::MagneticField();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        msg.magnetic_field.x = x * scale;
        msg.magnetic_field.y = y * scale;
        msg.magnetic_field.z = z * scale;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_ = -1;
    MagType mag_type_ = UNKNOWN;
    uint8_t current_addr_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagnetometerNode>());
    rclcpp::shutdown();
    return 0;
}
