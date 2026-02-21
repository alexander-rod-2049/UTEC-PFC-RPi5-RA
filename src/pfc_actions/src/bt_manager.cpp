#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp" 
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

// --- 1. Definición del Nodo de Acción ---
class FollowSMCPath : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  FollowSMCPath(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return { 
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
      BT::InputPort<double>("T_total"),
      BT::InputPort<double>("max_v")
    };
  }

  bool setGoal(RosActionNode::Goal& goal) override {
    auto res = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    
    if (!res) {
      // En lugar de matar el programa, avisamos y fallamos la meta
      RCLCPP_WARN(rclcpp::get_logger("bt_manager"), "Esperando meta en Blackboard puerto [target_pose]...");
      return false; 
    }
    
    goal.pose = res.value();
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override {
    return (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? 
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// --- 2. Función Principal ---
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_manager_node");
  BT::BehaviorTreeFactory factory;

  // Registro de Dummies de seguridad con sus puertos
  factory.registerSimpleCondition("IsBatteryOk", [&](BT::TreeNode&){ return BT::NodeStatus::SUCCESS; }, {BT::InputPort<double>("min_voltage")});
  factory.registerSimpleCondition("IsSystemHealthy", [&](BT::TreeNode&){ return BT::NodeStatus::SUCCESS; });
  factory.registerSimpleAction("AlignOrientation", [&](BT::TreeNode&){ return BT::NodeStatus::SUCCESS; }, {BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"), BT::InputPort<double>("tolerance")});
  factory.registerSimpleAction("SaySuccess", [&](BT::TreeNode&){ return BT::NodeStatus::SUCCESS; }, {BT::InputPort<std::string>("message")});

  // --- SOLUCIÓN DEFINITIVA: REGISTRO VÍA BUILDER ---
  BT::RosNodeParams params;
  params.nh = node;
  params.default_port_value = "smc_dfl_nav";
  
  // Creamos un lambda que captura los params y construye el nodo
  BT::NodeBuilder builder = [params](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<FollowSMCPath>(name, config, params);
  };

  // Registramos el builder directamente en la fábrica (evita el error de 'registerRosAction')
  factory.registerBuilder<FollowSMCPath>("FollowSMCPath", builder);

  // Cargar Árbol desde el share del paquete
  std::string pkg_dir = ament_index_cpp::get_package_share_directory("pfc_actions");
  auto tree = factory.createTreeFromFile(pkg_dir + "/behavior_trees/beach_logic.xml");
  auto blackboard = tree.rootBlackboard();
geometry_msgs::msg::PoseStamped default_goal;
default_goal.header.frame_id = "map";
default_goal.pose.position.x = 1.0; // Mover 1 metro al iniciar
blackboard->set("goal_pose", default_goal); // Asegúrate que el nombre coincida con tu XML
  // Publicador para Groot2 (Puerto 1667 por defecto)
  BT::Groot2Publisher publisher(tree);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  RCLCPP_INFO(node->get_logger(), "Tito BT Manager online (Builder Mode).");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Quitamos la condición de status del while para que el nodo no muera
  while (rclcpp::ok()) {
    status = tree.tickOnce();
    
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, 
                           "Árbol terminado con estado: %s. Esperando nueva meta...", 
                           (status == BT::NodeStatus::SUCCESS ? "ÉXITO" : "FALLO"));
      // Opcional: podrías resetear el árbol aquí si quieres reintentar
    }

    executor.spin_some(); 
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  rclcpp::shutdown();
  return 0;
}