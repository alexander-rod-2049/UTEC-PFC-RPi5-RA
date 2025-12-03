#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <cmath>
#include <ctime>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
// Tipos de mensaje
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SystemIdLogger : public rclcpp::Node
{
public:
  SystemIdLogger()
  : Node("system_id_logger")
  {
    // --- PARAMETROS ---
    this->declare_parameter("wheel_separation", 0.585);
    this->declare_parameter("wheel_radius", 0.1);
    this->declare_parameter("log_frequency", 50.0); // 50 Hz solicitado
    
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    double freq = this->get_parameter("log_frequency").as_double();

    // QoS Best Effort para sensores rápidos
    auto qos = rclcpp::SensorDataQoS();

    // --- SUSCRIPCIONES ---

    // 1. Comandos y Estado General
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&SystemIdLogger::cmd_vel_callback, this, _1));

    sub_status_ = this->create_subscription<ros2_roboclaw_driver::msg::RoboClawStatus>(
      "/roboclaw_status", 10, std::bind(&SystemIdLogger::status_callback, this, _1));

    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&SystemIdLogger::joint_callback, this, _1));

    // 2. Odometría (Raw Encoders vs Filtered EKF)
    sub_odom_raw_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/encoders", 10, std::bind(&SystemIdLogger::odom_raw_callback, this, _1));

    sub_odom_filt_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, std::bind(&SystemIdLogger::odom_filt_callback, this, _1));

    // 3. IMU Central y Magnetómetro
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/central/data", qos, std::bind(&SystemIdLogger::imu_callback, this, _1));
    
    sub_mag_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "/mag/gps/data", qos, std::bind(&SystemIdLogger::mag_callback, this, _1));

    // 4. Estimación de Deslizamiento (Slip) y Velocidad de Tracks (IMU based)
    sub_slip_l_ = this->create_subscription<std_msgs::msg::Float64>(
      "/slip/left", 10, std::bind(&SystemIdLogger::slip_l_callback, this, _1));
    sub_slip_r_ = this->create_subscription<std_msgs::msg::Float64>(
      "/slip/right", 10, std::bind(&SystemIdLogger::slip_r_callback, this, _1));

    sub_track_vel_l_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/track/velocity/left", qos, std::bind(&SystemIdLogger::track_l_callback, this, _1));
    sub_track_vel_r_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/track/velocity/right", qos, std::bind(&SystemIdLogger::track_r_callback, this, _1));

    // 5. GPS (Opcional si tienes fix)
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&SystemIdLogger::gps_callback, this, _1));


    // --- LOG FILE INIT ---
    create_log_file();

    // --- TIMER (50 Hz) ---
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / freq), 
      std::bind(&SystemIdLogger::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "LOGGER HARMONIZADO: Corriendo a %.1f Hz.", freq);
  }

  ~SystemIdLogger() {
    if (csv_file_.is_open()) csv_file_.close();
  }

private:
  // --- VARIABLES (NAN init) ---
  double cmd_lin_x_ = NAN, cmd_ang_z_ = NAN;
  double voltage_ = NAN, current_L_ = NAN, current_R_ = NAN;
  
  // Odom Raw (Encoders)
  double odom_raw_x_ = NAN, odom_raw_y_ = NAN, odom_raw_yaw_ = NAN;
  double odom_raw_vx_ = NAN, odom_raw_wz_ = NAN;

  // Odom Filtered (EKF)
  double odom_filt_x_ = NAN, odom_filt_y_ = NAN, odom_filt_yaw_ = NAN;
  double odom_filt_vx_ = NAN, odom_filt_wz_ = NAN;

  // Joints
  double joint_vel_L_ = NAN, joint_vel_R_ = NAN; 
  double real_vel_L_ms_ = NAN, real_vel_R_ms_ = NAN;

  // IMU & Mag
  double imu_acc_x_ = NAN, imu_acc_y_ = NAN, imu_gyro_z_ = NAN;
  double mag_x_ = NAN, mag_y_ = NAN, mag_z_ = NAN;

  // Slip & Track Velocity
  double slip_L_ = NAN, slip_R_ = NAN;
  double track_vel_L_ = NAN, track_vel_R_ = NAN;

  // GPS
  double gps_lat_ = NAN, gps_lon_ = NAN;

  double wheel_separation_, wheel_radius_;
  std::string filename_;
  std::ofstream csv_file_;

  // --- SUBSCRIBERS ---
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_raw_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_filt_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_slip_l_, sub_slip_r_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_track_vel_l_, sub_track_vel_r_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  // --- UTILS ---
  double get_yaw(const double qx, const double qy, const double qz, const double qw) {
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  }

  void create_log_file() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "full_robot_log_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    filename_ = oss.str();
    csv_file_.open(filename_);
    
    // CABECERA PARA MATLAB
    csv_file_ << "timestamp_ms,"
              // Comandos
              << "cmd_lin_x,cmd_ang_z,"
              // Electrico
              << "voltage,current_L,current_R,"
              // Velocidades Referencia Calculadas
              << "ref_vel_L_ms,ref_vel_R_ms,"
              // Joints (Real Encoders Speed)
              << "joint_vel_L_rad,joint_vel_R_rad,real_vel_L_ms,real_vel_R_ms,"
              // Track Velocities (IMU Based)
              << "track_vel_L_ms,track_vel_R_ms,"
              // Slip
              << "slip_L,slip_R,"
              // Odometry Raw (Encoders only)
              << "odom_raw_x,odom_raw_y,odom_raw_yaw,odom_raw_vx,odom_raw_wz,"
              // Odometry Filtered (EKF Output)
              << "ekf_x,ekf_y,ekf_yaw,ekf_vx,ekf_wz,"
              // IMU Central
              << "imu_acc_x,imu_acc_y,imu_gyro_z,"
              // Magnetometer
              << "mag_x,mag_y,mag_z,"
              // GPS
              << "gps_lat,gps_lon\n";
  }

  // --- CALLBACKS ---
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_lin_x_ = msg->linear.x; cmd_ang_z_ = msg->angular.z;
  }
  void status_callback(const ros2_roboclaw_driver::msg::RoboClawStatus::SharedPtr msg) {
    voltage_ = msg->main_battery_voltage; 
    current_L_ = msg->m1_motor_current; current_R_ = msg->m2_motor_current;
  }
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if(msg->velocity.size() >= 2) {
        joint_vel_L_ = msg->velocity[0]; joint_vel_R_ = msg->velocity[1];
        real_vel_L_ms_ = joint_vel_L_ * wheel_radius_; 
        real_vel_R_ms_ = joint_vel_R_ * wheel_radius_;
    }
  }
  void odom_raw_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_raw_x_ = msg->pose.pose.position.x; odom_raw_y_ = msg->pose.pose.position.y;
    odom_raw_vx_ = msg->twist.twist.linear.x; odom_raw_wz_ = msg->twist.twist.angular.z;
    odom_raw_yaw_ = get_yaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  }
  void odom_filt_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_filt_x_ = msg->pose.pose.position.x; odom_filt_y_ = msg->pose.pose.position.y;
    odom_filt_vx_ = msg->twist.twist.linear.x; odom_filt_wz_ = msg->twist.twist.angular.z;
    odom_filt_yaw_ = get_yaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  }
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_acc_x_ = msg->linear_acceleration.x; imu_acc_y_ = msg->linear_acceleration.y;
    imu_gyro_z_ = msg->angular_velocity.z;
  }
  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    mag_x_ = msg->magnetic_field.x; mag_y_ = msg->magnetic_field.y; mag_z_ = msg->magnetic_field.z;
  }
  // callbacks simples para std_msgs
  void slip_l_callback(const std_msgs::msg::Float64::SharedPtr msg) { slip_L_ = msg->data; }
  void slip_r_callback(const std_msgs::msg::Float64::SharedPtr msg) { slip_R_ = msg->data; }
  // callbacks twist stamped
  void track_l_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { track_vel_L_ = msg->twist.linear.x; }
  void track_r_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { track_vel_R_ = msg->twist.linear.x; }
  
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    gps_lat_ = msg->latitude; gps_lon_ = msg->longitude;
  }

  // --- WRITE LOOP ---
  void timer_callback() {
    if (!csv_file_.is_open()) return;
    
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    // Calcular referencias esperadas (Teóricas)
    double ref_vel_L = cmd_lin_x_ - (cmd_ang_z_ * wheel_separation_ / 2.0);
    double ref_vel_R = cmd_lin_x_ + (cmd_ang_z_ * wheel_separation_ / 2.0);

    csv_file_ << ms << ","
              << cmd_lin_x_ << "," << cmd_ang_z_ << ","
              << voltage_ << "," << current_L_ << "," << current_R_ << ","
              << ref_vel_L << "," << ref_vel_R << ","
              << joint_vel_L_ << "," << joint_vel_R_ << "," << real_vel_L_ms_ << "," << real_vel_R_ms_ << ","
              << track_vel_L_ << "," << track_vel_R_ << ","
              << slip_L_ << "," << slip_R_ << ","
              << odom_raw_x_ << "," << odom_raw_y_ << "," << odom_raw_yaw_ << "," << odom_raw_vx_ << "," << odom_raw_wz_ << ","
              << odom_filt_x_ << "," << odom_filt_y_ << "," << odom_filt_yaw_ << "," << odom_filt_vx_ << "," << odom_filt_wz_ << ","
              << imu_acc_x_ << "," << imu_acc_y_ << "," << imu_gyro_z_ << ","
              << mag_x_ << "," << mag_y_ << "," << mag_z_ << ","
              << std::setprecision(9) << gps_lat_ << "," << gps_lon_ << "\n";
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemIdLogger>());
  rclcpp::shutdown();
  return 0;
}