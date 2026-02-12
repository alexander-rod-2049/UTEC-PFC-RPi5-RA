#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <cstdlib>
#include <Eigen/Dense>

#define TCA9548A_ADDR 0x70
#define MAG_ADDR_HMC 0x1E
#define MAG_ADDR_QMC 0x0D
#define MPU_ADDR 0x68
#define MAG_CHANNEL 3
#define MPU_CHANNEL 2 // Central IMU

enum MagType { UNKNOWN, HMC5883L, QMC5883L };

struct MagDataPoint {
    double timestamp;
    double mag_x, mag_y, mag_z;         // Tesla
    double accel_x, accel_y, accel_z;   // m/s2 (Raw)
    double gyro_x, gyro_y, gyro_z;      // Rad/s
    double roll, pitch, yaw;            // Radians
    double roll_deg, pitch_deg, yaw_deg;// Degrees
    double lin_accel_x, lin_accel_y, lin_accel_z; // m/s^2 (World Frame, No Gravity)
    double vel_x, vel_y, vel_z;         // m/s (World Frame)
};

// Low Pass Filter Class
class LowPassFilter {
public:
    LowPassFilter(double alpha) : alpha_(alpha), initialized_(false) {}
    Eigen::Vector3d update(const Eigen::Vector3d& input) {
        if (!initialized_) {
            output_ = input;
            initialized_ = true;
        } else {
            output_ = alpha_ * output_ + (1.0 - alpha_) * input;
        }
        return output_;
    }
private:
    double alpha_;
    bool initialized_;
    Eigen::Vector3d output_;
};

// Extended Kalman Filter for AHRS (Quaternion)
class EKF {
public:
    EKF() {
        // Init State (Quaternion [w, x, y, z])
        x_ = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
        P_ = Eigen::Matrix4d::Identity() * 0.01;
        Q_ = Eigen::Matrix4d::Identity() * 0.001;
        R_acc_ = Eigen::Matrix3d::Identity() * 0.1; // Trust accel more for static tilt
    }
    
    // Initialize Quaternion from Accelerometer (Gravity Vector)
    void initFromAccel(const Eigen::Vector3d& accel) {
        if(accel.norm() < 0.1) return;
        Eigen::Vector3d a = accel.normalized();
        // We want to find q such that R(q)^T * [0,0,1] = a
        // i.e., rotating world Z (gravity) into body frame gives 'a'
        // Pitch (theta) = asin(-ax)
        // Roll (phi) = atan2(ay, az)
        double pitch = asin(-a.x());
        double roll = atan2(a.y(), a.z());
        double yaw = 0; // Assume 0 initially

        // Convert Euler to Quaternion
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        x_(0) = cr * cp * cy + sr * sp * sy; // w
        x_(1) = sr * cp * cy - cr * sp * sy; // x
        x_(2) = cr * sp * cy + sr * cp * sy; // y
        x_(3) = cr * cp * sy - sr * sp * cy; // z
        x_.normalize();
    }

    void predict(const Eigen::Vector3d& gyro, double dt) {
        // q_dot = 0.5 * Omega * q
        Eigen::Matrix4d Omega;
        Omega << 0, -gyro.x(), -gyro.y(), -gyro.z(),
                 gyro.x(), 0, gyro.z(), -gyro.y(),
                 gyro.y(), -gyro.z(), 0, gyro.x(),
                 gyro.z(), gyro.y(), -gyro.x(), 0;
                 
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity() + 0.5 * Omega * dt;
        x_ = F * x_;
        x_.normalize(); 
        P_ = F * P_ * F.transpose() + Q_;
    }
    
    void updateAccel(const Eigen::Vector3d& accel) {
        if (accel.norm() < 0.01) return;
        Eigen::Vector3d a_norm = accel.normalized();
        
        double qw = x_(0), qx = x_(1), qy = x_(2), qz = x_(3);
        // Predicted Gravity in Body Frame
        Eigen::Vector3d h;
        h(0) = 2.0 * (qx*qz - qw*qy);
        h(1) = 2.0 * (qw*qx + qy*qz);
        h(2) = qw*qw - qx*qx - qy*qy + qz*qz;
        
        // Jacobian H
        Eigen::Matrix<double, 3, 4> H;
        H(0,0) = -2*qy; H(1,0) = 2*qx; H(2,0) = 2*qw;
        H(0,1) = 2*qz; H(1,1) = 2*qw; H(2,1) = -2*qx;
        H(0,2) = -2*qw; H(1,2) = 2*qz; H(2,2) = -2*qy;
        H(0,3) = 2*qx; H(1,3) = 2*qy; H(2,3) = 2*qz;
        
        // EKF Update
        Eigen::Vector3d y = a_norm - h;
        Eigen::Matrix3d S = H * P_ * H.transpose() + R_acc_;
        Eigen::Matrix<double, 4, 3> K = P_ * H.transpose() * S.inverse();
        
        x_ = x_ + K * y;
        x_.normalize();
        P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
    }
    
    Eigen::Vector4d getState() { return x_; }
    
    Eigen::Matrix3d getRotationMatrix() {
        double qw = x_(0), qx = x_(1), qy = x_(2), qz = x_(3);
        Eigen::Matrix3d R;
        R(0,0) = 1 - 2*qy*qy - 2*qz*qz; R(0,1) = 2*qx*qy - 2*qz*qw; R(0,2) = 2*qx*qz + 2*qy*qw;
        R(1,0) = 2*qx*qy + 2*qz*qw;     R(1,1) = 1 - 2*qx*qx - 2*qz*qz; R(1,2) = 2*qy*qz - 2*qx*qw;
        R(2,0) = 2*qx*qz - 2*qy*qw;     R(2,1) = 2*qy*qz + 2*qx*qw;     R(2,2) = 1 - 2*qx*qx - 2*qy*qy;
        return R;
    }

private:
    Eigen::Vector4d x_; // State
    Eigen::Matrix4d P_; // Covariance
    Eigen::Matrix4d Q_; // Process Noise
    Eigen::Matrix3d R_acc_; // Measurement Noise
};

class MagnetometerNode : public rclcpp::Node
{
public:
    MagnetometerNode() : Node("magnetometer_node"), ekf_(), lpf_lin_accel_(0.8), is_calibrated_(false)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("gps/mag", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&MagnetometerNode::timer_callback, this));
            
        init_i2c();
        RCLCPP_INFO(this->get_logger(), "🚀 Magnetometer Node Started. Calibrating...");
    }

    ~MagnetometerNode()
    {
        if (i2c_fd_ >= 0) close(i2c_fd_);
        save_data_to_csv();
        plot_data();
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
        select_multiplexer_channel(MPU_CHANNEL);
        init_mpu();
        select_multiplexer_channel(MAG_CHANNEL);
        detect_and_configure();
    }

    void select_multiplexer_channel(uint8_t channel)
    {
        if (ioctl(i2c_fd_, I2C_SLAVE, TCA9548A_ADDR) < 0) return;
        uint8_t data = (1 << channel);
        write(i2c_fd_, &data, 1);
    }

    void init_mpu()
    {
        if (ioctl(i2c_fd_, I2C_SLAVE, MPU_ADDR) < 0) return;
        write_reg(0x6B, 0x00); 
        write_reg(0x1A, 0x03); 
    }

    void detect_and_configure()
    {
        if (check_device(MAG_ADDR_QMC)) {
            uint8_t id = read_reg(0x0D);
            if (id == 0xFF) {
                mag_type_ = QMC5883L; current_addr_ = MAG_ADDR_QMC;
                write_reg(0x0B, 0x01); write_reg(0x09, 0x1D); return;
            }
        }
        if (check_device(MAG_ADDR_HMC)) {
            mag_type_ = HMC5883L; current_addr_ = MAG_ADDR_HMC;
            write_reg(0x00, 0x70); write_reg(0x01, 0xA0); write_reg(0x02, 0x00); return;
        }
    }
    
    bool check_device(uint8_t addr) {
        if (ioctl(i2c_fd_, I2C_SLAVE, addr) < 0) return false;
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

    int16_t read_word_2c(int reg) {
        uint8_t buf[2];
        buf[0] = (uint8_t)reg;
        if(write(i2c_fd_, buf, 1) != 1) return 0;
        if(read(i2c_fd_, buf, 2) != 2) return 0;
        return (int16_t)((buf[0] << 8) + buf[1]);
    }

    void read_mpu() {
        select_multiplexer_channel(MPU_CHANNEL);
        if (ioctl(i2c_fd_, I2C_SLAVE, MPU_ADDR) < 0) return;

        double a_s = 16384.0; 
        double g_s = 131.0;
        double G = 9.81; 
        double RAD = M_PI/180.0;

        current_ax_ = (read_word_2c(0x3B) / a_s) * G;
        current_ay_ = (read_word_2c(0x3D) / a_s) * G;
        current_az_ = (read_word_2c(0x3F) / a_s) * G;
        
        // Apply Manual Bias Correction
        current_ax_ -= ACCEL_BIAS_X;
        current_ay_ -= ACCEL_BIAS_Y;
        current_az_ -= ACCEL_BIAS_Z;

        current_gx_ = (read_word_2c(0x43) / g_s) * RAD;
        current_gy_ = (read_word_2c(0x45) / g_s) * RAD;
        current_gz_ = (read_word_2c(0x47) / g_s) * RAD;
    }

    void read_mag() {
        select_multiplexer_channel(MAG_CHANNEL);
        if (ioctl(i2c_fd_, I2C_SLAVE, current_addr_) < 0) return;

        int16_t raw_x=0, raw_y=0, raw_z=0;
        uint8_t data[6];
        bool success = false;
        if (mag_type_ == HMC5883L) {
            uint8_t reg = 0x03;
            if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, data, 6) == 6) {
                raw_x = (int16_t)((data[0] << 8) | data[1]);
                raw_z = (int16_t)((data[2] << 8) | data[3]);
                raw_y = (int16_t)((data[4] << 8) | data[5]);
                success = true;
            }
        } else if (mag_type_ == QMC5883L) {
            uint8_t reg = 0x00;
            if(write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, data, 6) == 6) { 
                raw_x = (int16_t)((data[1] << 8) | data[0]);
                raw_y = (int16_t)((data[3] << 8) | data[2]);
                raw_z = (int16_t)((data[5] << 8) | data[4]);
                success = true;
            }
        }

        if (success) {
            float scale = 0.92e-7; 
            current_mag_x_ = raw_x * scale;
            current_mag_y_ = raw_y * scale;
            current_mag_z_ = raw_z * scale;
        }
    }

    void calibrate() {
        static int sample_count = 0;
        static Eigen::Vector3d gyro_sum(0,0,0);
        static Eigen::Vector3d accel_sum(0,0,0);
        const int N_SAMPLES = 100; // Increased samples

        gyro_sum += Eigen::Vector3d(current_gx_, current_gy_, current_gz_);
        accel_sum += Eigen::Vector3d(current_ax_, current_ay_, current_az_);
        sample_count++;

        if (sample_count >= N_SAMPLES) {
            gyro_bias_ = gyro_sum / N_SAMPLES;
            Eigen::Vector3d init_accel = accel_sum / N_SAMPLES;
            
            // Capture measured gravity magnitude (e.g. 10.5 vs 9.81)
            gravity_mag_ = init_accel.norm();

            // Initialize EKF with initial tilt
            ekf_.initFromAccel(init_accel);
            
            // Calculate Display Offsets (Tare)
            Eigen::Vector3d a = init_accel.normalized();
            double pitch_init = asin(-a.x());
            double roll_init = atan2(a.y(), a.z());
            double yaw_init = 0.0; // EKF starts at 0 yaw
            
            // Tare: Output = Measured - Offset
            // Target: Roll=0, Pitch=0, Yaw=90 (PI/2)
            roll_offset_ = roll_init - 0;
            pitch_offset_ = pitch_init - 0;
            yaw_offset_ = yaw_init - (M_PI / 2.0);
            
            is_calibrated_ = true;
            start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "✅ Calibration Complete. G-Mag: %.3f. Offsets -> R: %.1f, P: %.1f, Y: %.1f", 
                gravity_mag_, roll_offset_*180.0/M_PI, pitch_offset_*180.0/M_PI, yaw_offset_*180.0/M_PI);
        }
    }

    void timer_callback()
    {
        if (i2c_fd_ < 0) { init_i2c(); return; }
        
        rclcpp::Time now = this->now();
        double dt = (last_time_.nanoseconds() == 0) ? 0.01 : (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0 || dt > 0.5) dt = 0.01; 

        // 1. Read Sensors
        read_mpu();
        if (mag_type_ != UNKNOWN) read_mag();

        if (!is_calibrated_) {
            calibrate();
            return;
        }

        // 2. EKF Prediction (Gyro - Bias)
        Eigen::Vector3d gyro(current_gx_, current_gy_, current_gz_);
        ekf_.predict(gyro - gyro_bias_, dt);
        
        // 3. EKF Correction (Accel)
        Eigen::Vector3d accel(current_ax_, current_ay_, current_az_);
        ekf_.updateAccel(accel);
        
        // 4. Get Estimated Orientation
        Eigen::Matrix3d R = ekf_.getRotationMatrix();
        
        // Calculate Roll/Pitch/Yaw from R (Physical)
        double roll = atan2(R(2,1), R(2,2));
        double pitch = asin(-R(2,0));
        if (std::isnan(pitch)) pitch = 0; // Safety
        double yaw = atan2(R(1,0), R(0,0));
        
        // 5. Calculate Linear Acceleration (World Frame)
        // a_world = R * a_body
        Eigen::Vector3d a_world = R * accel;
        // Subtract Measured Gravity Magnitude
        Eigen::Vector3d gravity(0.0, 0.0, gravity_mag_); 
        Eigen::Vector3d lin_accel = a_world - gravity;
        
        // 6. Low Pass Filter (Linear Acceleration)
        Eigen::Vector3d lin_filtered = lpf_lin_accel_.update(lin_accel);
        
        // 7. Integrate Velocity
        // Deadband (Very small to allow some data)
        double deadband = 0.01; 
        if (fabs(lin_filtered.x()) < deadband) lin_filtered.x() = 0;
        if (fabs(lin_filtered.y()) < deadband) lin_filtered.y() = 0;
        if (fabs(lin_filtered.z()) < deadband) lin_filtered.z() = 0;
        
        vel_ += lin_filtered * dt;
        
        // NaN Check
        if (std::isnan(vel_.x())) vel_.x() = 0;
        if (std::isnan(vel_.y())) vel_.y() = 0;
        if (std::isnan(vel_.z())) vel_.z() = 0;
        
        // Decay to prevent drift explosion
        if (lin_filtered.norm() < 0.2) {
            vel_ *= 0.98; 
        }

        // Store
        MagDataPoint dp;
        dp.timestamp = (now - start_time_).seconds();
        dp.mag_x = current_mag_x_; dp.mag_y = current_mag_y_; dp.mag_z = current_mag_z_;
        dp.accel_x = current_ax_; dp.accel_y = current_ay_; dp.accel_z = current_az_;
        dp.gyro_x = current_gx_; dp.gyro_y = current_gy_; dp.gyro_z = current_gz_;
        
        // Apply Tare Offsets and Declination for Display
        dp.roll = roll - roll_offset_;
        dp.pitch = pitch - pitch_offset_;
        
        // Correct Yaw for Declination (Geographic North)
        double yaw_corrected = yaw + MAG_DECLINATION;
        dp.yaw = yaw_corrected - yaw_offset_;
        
        dp.roll_deg = dp.roll * 180.0/M_PI;
        dp.pitch_deg = dp.pitch * 180.0/M_PI;
        dp.yaw_deg = dp.yaw * 180.0/M_PI;
        
        dp.lin_accel_x = lin_filtered.x(); 
        dp.lin_accel_y = lin_filtered.y(); 
        dp.lin_accel_z = lin_filtered.z();
        
        dp.vel_x = vel_.x(); dp.vel_y = vel_.y(); dp.vel_z = vel_.z();
        
        data_log_.push_back(dp);
    }

    void save_data_to_csv()
    {
        std::string path = "/home/rmf209/PFCII/pfc2_ws/data/magnetometer_data.csv";
        std::ofstream file(path);
        if (file.is_open()) {
            file << "timestamp,mag_x,mag_y,mag_z,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll_rad,pitch_rad,yaw_rad,roll_deg,pitch_deg,yaw_deg,lin_accel_x,lin_accel_y,lin_accel_z,vel_x,vel_y,vel_z\n";
            for (const auto& dp : data_log_) {
                file << dp.timestamp << ","
                     << dp.mag_x << "," << dp.mag_y << "," << dp.mag_z << ","
                     << dp.accel_x << "," << dp.accel_y << "," << dp.accel_z << ","
                     << dp.gyro_x << "," << dp.gyro_y << "," << dp.gyro_z << ","
                     << dp.roll << "," << dp.pitch << "," << dp.yaw << ","
                     << dp.roll_deg << "," << dp.pitch_deg << "," << dp.yaw_deg << ","
                     << dp.lin_accel_x << "," << dp.lin_accel_y << "," << dp.lin_accel_z << ","
                     << dp.vel_x << "," << dp.vel_y << "," << dp.vel_z << "\n";
            }
            file.close();
            RCLCPP_INFO(this->get_logger(), "💾 Data saved to %s", path.c_str());
        }
    }
    
    void plot_data()
    {
        std::string data_path = "/home/rmf209/PFCII/pfc2_ws/data/magnetometer_data.csv";
        RCLCPP_INFO(this->get_logger(), "📈 Opening Plotter...");
        std::string cmd = "ros2 run gps_pkg plotter.py mag " + data_path + " &";
        system(cmd.c_str());
    }

    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_ = -1;
    MagType mag_type_ = UNKNOWN;
    uint8_t current_addr_ = 0;
    
    rclcpp::Time start_time_;
    rclcpp::Time last_time_;
    std::vector<MagDataPoint> data_log_;
    
    // Sensor data
    double current_mag_x_=0, current_mag_y_=0, current_mag_z_=0;
    double current_ax_=0, current_ay_=0, current_az_=0;
    double current_gx_=0, current_gy_=0, current_gz_=0;
    
    // Manual Accel Bias (Offsets to subtract)
    // X: 1.30289, Y: -0.27423, Z: 0.8105
    const double ACCEL_BIAS_X = 1.30289;
    const double ACCEL_BIAS_Y = -0.27423;
    const double ACCEL_BIAS_Z = 0.8105;
    
    // Magnetic Declination for Lima, Peru (~ -1.73 degrees)
    const double MAG_DECLINATION = -1.73 * (M_PI / 180.0);

    // Calibration
    bool is_calibrated_;
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    double roll_offset_ = 0;
    double pitch_offset_ = 0;
    double yaw_offset_ = 0;
    double gravity_mag_ = 9.81;
    
    EKF ekf_;
    LowPassFilter lpf_lin_accel_;
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagnetometerNode>());
    rclcpp::shutdown();
    return 0;
}
