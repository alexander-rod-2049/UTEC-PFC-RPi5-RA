#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// Importamos el mensaje del driver (CamelCase -> snake_case header)
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp" 

using std::placeholders::_1;

class SystemIdLogger : public rclcpp::Node
{
public:
  SystemIdLogger()
  : Node("system_id_logger")
  {
    // PARAMETROS
    this->declare_parameter("wheel_separation", 0.585);
    this->declare_parameter("wheel_radius", 0.1);
    
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();

    // SUSCRIPCIONES
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&SystemIdLogger::cmd_vel_callback, this, _1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/sigyn/wheel_odom", 10, std::bind(&SystemIdLogger::odom_callback, this, _1));

    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&SystemIdLogger::joint_callback, this, _1));

    // NUEVO: Suscripción a estado de batería y corriente
    sub_status_ = this->create_subscription<ros2_roboclaw_driver::msg::RoboClawStatus>(
      "/roboclaw_status", 10, std::bind(&SystemIdLogger::status_callback, this, _1));

    // ARCHIVO CSV
    csv_file_.open("robot_data_log.csv");
    // Cabecera del CSV actualizada
    csv_file_ << "timestamp,"
              << "cmd_lin_x,cmd_ang_z,"         
              << "ref_wheel_L,ref_wheel_R,"     
              << "real_wheel_L,real_wheel_R,"   
              << "odom_x,odom_y,odom_yaw,"      
              << "odom_vx,odom_wz,"
              << "voltage_main,current_L,current_R\n"; // Nuevas columnas

    RCLCPP_INFO(this->get_logger(), "Logger V2 iniciado. Esperando datos...");
  }

  ~SystemIdLogger() {
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(this->get_logger(), "Archivo CSV cerrado.");
    }
  }

private:
  double current_cmd_lin_ = 0.0;
  double current_cmd_ang_ = 0.0;
  double wheel_separation_;
  double wheel_radius_;

  // Variables para guardar el último estado de energía conocido
  float last_voltage_ = 0.0;
  float last_current_L_ = 0.0;
  float last_current_R_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr sub_status_;
  
  nav_msgs::msg::Odometry latest_odom_;
  std::ofstream csv_file_;

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    current_cmd_lin_ = msg->linear.x;
    current_cmd_ang_ = msg->angular.z;
  }

  // Callback para leer datos eléctricos (aprox 1Hz según yaml)
  void status_callback(const ros2_roboclaw_driver::msg::RoboClawStatus::SharedPtr msg)
  {
    last_voltage_ = msg->main_battery_voltage;
    last_current_L_ = msg->m1_motor_current;
    last_current_R_ = msg->m2_motor_current;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
  }

  // Usamos el joint_callback como trigger principal para escribir (20Hz)
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double vel_L_rads = 0.0;
    double vel_R_rads = 0.0;

    // Asumimos orden [0]=Left, [1]=Right. Si está al revés, intercambiar índices.
    if(msg->velocity.size() >= 2) {
        vel_L_rads = msg->velocity[0];
        vel_R_rads = msg->velocity[1];
    }

    // Conversión Rad/s -> m/s
    double real_vel_L_ms = vel_L_rads * wheel_radius_; 
    double real_vel_R_ms = vel_R_rads * wheel_radius_;

    // Calculamos referencia esperada
    double ref_vel_L = current_cmd_lin_ - (current_cmd_ang_ * wheel_separation_ / 2.0);
    double ref_vel_R = current_cmd_lin_ + (current_cmd_ang_ * wheel_separation_ / 2.0);

    write_to_csv(ref_vel_L, ref_vel_R, real_vel_L_ms, real_vel_R_ms);
  }

  void write_to_csv(double ref_L, double ref_R, double real_L, double real_R)
  {
    if (!csv_file_.is_open()) return;

    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    // Yaw desde cuaternión
    double qx = latest_odom_.pose.pose.orientation.x;
    double qy = latest_odom_.pose.pose.orientation.y;
    double qz = latest_odom_.pose.pose.orientation.z;
    double qw = latest_odom_.pose.pose.orientation.w;
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    csv_file_ << timestamp << ","
              << current_cmd_lin_ << "," << current_cmd_ang_ << ","
              << ref_L << "," << ref_R << ","
              << real_L << "," << real_R << ","
              << latest_odom_.pose.pose.position.x << ","
              << latest_odom_.pose.pose.position.y << ","
              << yaw << ","
              << latest_odom_.twist.twist.linear.x << ","
              << latest_odom_.twist.twist.angular.z << ","
              << last_voltage_ << "," 
              << last_current_L_ << "," 
              << last_current_R_ << "\n";
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemIdLogger>());
  rclcpp::shutdown();
  return 0;
}