#include <rclcpp/rclcpp.hpp>
#include "hnurm_uart/uart_node.hpp"

int main(int argc, char** argv)
{
  // 初始化 ROS 2 节点环境
  rclcpp::init(argc, argv);
  
  // 实例化 UartNode，此时构造函数会自动打开串口并启动 read_loop 线程
  auto node = std::make_shared<hnurm::UartNode>();
  
  // 阻塞主线程，保持节点活跃，并处理 ROS 2 话题的收发回调 (如 write_callback)
  rclcpp::spin(node);
  
  // 收到中断信号 (Ctrl+C) 后安全退出
  rclcpp::shutdown();
  return 0;
}