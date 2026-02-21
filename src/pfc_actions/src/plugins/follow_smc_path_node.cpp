#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace BT;

class FollowSMCPath : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  FollowSMCPath(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {}

  // Definimos qué puertos (entradas/salidas) tiene este nodo en el XML
  static PortsList providedPorts()
  {
    return {
      InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
      InputPort<double>("T_total"),
      InputPort<double>("max_v")
    };
  }

  // Esta función se llama una vez para construir la meta (Goal) de la acción
  bool setGoal(RosActionNode::Goal& goal) override
  {
    auto res = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    if (!res)
    {
      throw RuntimeException("Falta el puerto [target_pose] en FollowSMCPath");
    }
    
    goal.pose = res.value();
    // Aquí podrías pasar parámetros extra si tu acción los soportara en el mensaje,
    // pero NavigateToPose es estándar. Los parámetros max_v y T_total 
    // usualmente se leen directamente del params.yaml del servidor.
    
    return true;
  }

  // Se llama cuando la acción termina con éxito
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }

  // Se llama si la acción es cancelada por el árbol (ej. por una falla de seguridad)
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    return NodeStatus::FAILURE;
  }
};

// Esto permite que el manager cargue el plugin dinámicamente
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  RosNodeParams params;
  params.nh = std::make_shared<rclcpp::Node>("follow_smc_path_node");
  params.default_port_value = "smc_dfl_nav"; // Nombre de tu Action Server
  factory.registerNodeType<FollowSMCPath>("FollowSMCPath", params);
}