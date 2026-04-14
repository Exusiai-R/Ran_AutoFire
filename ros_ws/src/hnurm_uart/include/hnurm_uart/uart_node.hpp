#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>
#include <string>

// 引入强大的编解码器
#include "hnurm_uart/Serialcodec.h"
#include <hnurm_interfaces/msg/serial2_vision.hpp>
#include <hnurm_interfaces/msg/vision2_serial.hpp>

namespace hnurm
{

class UartNode : public rclcpp::Node
{
public:
  explicit UartNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~UartNode() override;

private:
  void write_callback(const hnurm_interfaces::msg::Vision2Serial::SharedPtr msg);
  void read_loop();

  rclcpp::Publisher<hnurm_interfaces::msg::Serial2Vision>::SharedPtr publisher_;
  rclcpp::Subscription<hnurm_interfaces::msg::Vision2Serial>::SharedPtr subscriber_;

  // 【核心修改】接管带环形缓冲的编解码器，而不是裸串口
  std::unique_ptr<SerialCodec> codec_;
  
  std::thread read_thread_;
  bool rclcpp_ok_;
};

}  // namespace hnurm