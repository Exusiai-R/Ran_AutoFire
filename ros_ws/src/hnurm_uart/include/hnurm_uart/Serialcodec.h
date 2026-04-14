#pragma once
#include <mutex>
#include <string>
#include <memory>

// 引入新协议消息
#include <hnurm_interfaces/msg/serial2_vision.hpp>
#include <hnurm_interfaces/msg/vision2_serial.hpp>

// 保留底层物理串口
#include "hnurm_uart/serial.h"

namespace hnurm
{

class SerialCodec
{
public:
  // 构造函数，接管底层 serial 指针
  explicit SerialCodec(std::unique_ptr<Serial> serial) : serial_(std::move(serial)) {}

  // 极简收发接口
  bool send_data(const hnurm_interfaces::msg::Vision2Serial& data);
  bool try_get_recv_data_for(hnurm_interfaces::msg::Serial2Vision& recv_data, int timeout_ms);

private:
  // 这三个变量就是你报错里 "was not declared in this scope" 的元凶，现在补上了
  std::unique_ptr<Serial> serial_;
  std::string recv_buf_;
  std::mutex mutex_;
};

}  // namespace hnurm