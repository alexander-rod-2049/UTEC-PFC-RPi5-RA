#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_manager_node");

  // 1. Instanciar la fábrica de árboles
  BT::BehaviorTreeFactory factory;
    
  // 2. Cargar y registrar los plugins (nuestros Action Nodes)
  // Nota: Asegúrate de que el nombre coincida con el CMakeLists.txt
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("pfc_actions");
  std::string plugin_lib_path = package_share_dir + "/../../lib/libfollow_smc_path_plugin.so";
  
  factory.registerFromPlugin(plugin_lib_path);

  // 3. Cargar el archivo XML
  std::string xml_path = package_share_dir + "/behavior_trees/beach_logic.xml";
  auto tree = factory.createTreeFromFile(xml_path);
  BT::Groot2Publisher publisher(tree);
  // 4. Logger para ver el estado del árbol en la terminal
  BT::StdCoutLogger logger_cout(tree);

  // 5. El Bucle de Ejecución (Tick)
  rclcpp::Rate loop_rate(10); // El árbol "piensa" a 10Hz
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  RCLCPP_INFO(node->get_logger(), "Behavior Tree iniciado. Esperando condiciones de seguridad...");

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    // Tick: El árbol evalúa todas las ramas
    status = tree.tickOnce();
    
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "¡Misión finalizada con éxito!");
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "Misión fallida o abortada por seguridad.");
  }

  rclcpp::shutdown();
  return 0;
}