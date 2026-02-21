#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class MissionManager : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MissionManager() : Node("mission_manager_smc_dfl") {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "smc_dfl_nav");
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&MissionManager::run_fsm, this));
        
        // Waypoints de ejemplo para limpieza (x, y)
        waypoints_ = {{1.0, 0.0}, {1.0, 0.5}, {0.0, 1.0}, {0.0, 0.0}};
    }

private:
    void run_fsm() {
        timer_->cancel();
        if (current_idx_ < waypoints_.size()) {
            send_next_goal();
        } else {
            RCLCPP_INFO(this->get_logger(), "¡Misión de limpieza completada!");
        }
    }

    void send_next_goal() {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server no disponible");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = waypoints_[current_idx_].first;
        goal_msg.pose.pose.position.y = waypoints_[current_idx_].second;
        goal_msg.pose.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Enviando punto %d: (%.2f, %.2f)", 
                    current_idx_, waypoints_[current_idx_].first, waypoints_[current_idx_].second);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                current_idx_++;
                run_fsm();
            }
        };
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_idx_ = 0;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManager>());
    rclcpp::shutdown();
    return 0;
}