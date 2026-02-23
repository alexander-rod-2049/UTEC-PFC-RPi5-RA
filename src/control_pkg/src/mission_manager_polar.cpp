#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h" // <--- AGREGA ESTA LÍNEA

class MissionManagerPolar : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MissionManagerPolar() : Node("mission_manager_polar") {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&MissionManagerPolar::run_mission, this));
        
        // Waypoints (x, y, yaw) - Máximo abs 1.5m
        //waypoints_ = {
        //   {1.2, 0.5, 0.0},
        //    {1.2, 1.2, 1.57},
        //    {0.0, 1.2, 3.14},
        //    {0.0, 0.0, 0.0}
        //};
waypoints_ = {
    {1.30,  0.00,  0.0000},
    {2.90,  1.20,  0.0000},
    {5.10,  1.10, -1.5708},
    {5.40, -0.45, -3.1416},
    {3.60, -0.90, -1.5708},
    {3.30, -2.00, -1.5708}
};
    }

private:
    void run_mission() {
        timer_->cancel();
        if (current_idx_ < waypoints_.size()) {
            send_goal(waypoints_[current_idx_]);
        } else {
            RCLCPP_INFO(this->get_logger(), "Misión Polar completada con éxito.");
        }
    }

    void send_goal(const std::vector<double>& point) {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 'navigate_to_pose' no disponible.");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = point[0];
        goal_msg.pose.pose.position.y = point[1];
        
        // Convertir Yaw a Quaternio
        tf2::Quaternion q;
        q.setRPY(0, 0, point[2]);
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();
        goal_msg.pose.header.frame_id = "odom";

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Punto alcanzado.");
                current_idx_++;
                run_mission();
            }
        };
        client_->async_send_goal(goal_msg, options);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> waypoints_;
    size_t current_idx_ = 0;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManagerPolar>());
    rclcpp::shutdown();
    return 0;
}