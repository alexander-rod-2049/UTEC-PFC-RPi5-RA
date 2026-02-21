#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <stdexcept>
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

class FollowSMCPath : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  FollowSMCPath(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {}

  static PortsList providedPorts() {
    return { 
      InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
      InputPort<double>("T_total"),
      InputPort<double>("max_v")
    };
  }

  bool setGoal(RosActionNode::Goal& goal) override {
    auto res = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    if (!res) throw std::runtime_error("Falta target_pose en FollowSMCPath");
    goal.pose = res.value();
    return true;
  }

  NodeStatus onResultReceived(const WrappedResult& wr) override {
    return (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? 
           NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual NodeStatus onFailure(ActionNodeErrorCode error) override { 
    (void)error; 
    return NodeStatus::FAILURE; 
  }
};

// ESTA ES LA ÚNICA FORMA DE REGISTRO QUE DEBE EXISTIR EN ESTE ARCHIVO
BT_REGISTER_NODES(factory)
{
  RosNodeParams params;
  params.default_port_value = "smc_dfl_nav";
  
  NodeBuilder builder = [params](const std::string& name, const NodeConfig& config) {
    return std::make_unique<FollowSMCPath>(name, config, params);
  };

  factory.registerBuilder<FollowSMCPath>("FollowSMCPath", builder);
}