#include "hnurm_uart/uart_node.hpp"

namespace hnurm
{

UartNode::UartNode(const rclcpp::NodeOptions& options) : Node("uart_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting UartNode with Anti-Sticky Packet Codec...");

  this->declare_parameter<std::string>("device_name", "/dev/ttyACM0");
  std::string device_name;
  this->get_parameter("device_name", device_name);

  try {
    // 1. 正确的底层串口初始化流程 (适配你的 serial.h)
    auto serial = std::make_unique<Serial>();
    if (!serial->open_port(device_name)) {
      throw std::runtime_error("Failed to open port: " + device_name);
    }
    if (!serial->init()) {
      throw std::runtime_error("Failed to init port settings");
    }
    
    // 2. 将初始化的串口移交给 SerialCodec，由它来处理粘包和校验
    codec_ = std::make_unique<SerialCodec>(std::move(serial));
    RCLCPP_INFO(this->get_logger(), "Serial port opened & Codec successfully mounted.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Serial setup error: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  // 3. 初始化发布器与订阅器
  publisher_ = this->create_publisher<hnurm_interfaces::msg::Serial2Vision>(
    "/uart/serial_to_vision", 10);
  
  subscriber_ = this->create_subscription<hnurm_interfaces::msg::Vision2Serial>(
    "/uart/vision_to_serial", 10, 
    std::bind(&UartNode::write_callback, this, std::placeholders::_1));

  rclcpp_ok_ = true;
  read_thread_ = std::thread(&UartNode::read_loop, this);
}

UartNode::~UartNode()
{
  rclcpp_ok_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

// ==========================================
// 串口读取线程 (Read Loop)
// ==========================================
void UartNode::read_loop()
{
  while (rclcpp_ok_ && rclcpp::ok()) {
    hnurm_interfaces::msg::Serial2Vision recv_msg;
    
    // 【核心】调用 Codec 的安全提取方法。它自带缓冲，最多阻塞 5ms
    if (codec_->try_get_recv_data_for(recv_msg, 5)) {
        // 只要能解出一个完整的包，立刻赋予绝对时间戳并发布
        recv_msg.header.stamp = this->now();
        publisher_->publish(recv_msg);
    }
  }
}

// ==========================================
// 写入回调函数 (Write Callback)
// ==========================================
void UartNode::write_callback(const hnurm_interfaces::msg::Vision2Serial::SharedPtr msg)
{
  // 【核心】直接调用 Codec 的封装发送方法，零额外开销
  if (!codec_->send_data(*msg)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to send data to STM32");
  }
}

}  // namespace hnurm