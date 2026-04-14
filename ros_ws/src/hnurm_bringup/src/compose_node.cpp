//
// Created by Kai Wang on 24-5-22.
//
#include <armor_solver/armor_solver_node.hpp>
#include <hnurm_camera/camera_node.hpp>
#include <armor_detector/armor_detector_node.hpp>
#include <hnurm_uart/uart_node.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto uart_node   = std::make_shared<hnurm::UartNode>(options);
  auto camera_node = std::make_shared<hnurm::CameraNode>(options);
  auto armor_detector_node = std::make_shared<hnurm::ArmorDetectorNode>(options);
  auto armor_solver_node  = std::make_shared<hnurm::ArmorSolverNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(uart_node);
  executor.add_node(camera_node);
  executor.add_node(armor_detector_node);
  executor.add_node(armor_solver_node);

  uart_node->run();
  // camera_node->run();

  executor.spin();

  rclcpp::shutdown();
  return 0;
}