#include "hnurm_uart/Serialcodec.h"
#include "hnurm_uart/protocol.h"
#include <chrono>
#include <thread>

namespace hnurm
{

// ==========================================
// 1. 发送数据 (视觉 -> 电控)
// ==========================================
bool SerialCodec::send_data(const hnurm_interfaces::msg::Vision2Serial& data)
{
  try {
    // 调用 O(1) 序列化方法
    auto data_str = Protocol::encode(data);
    
    // 【修改点】：使用你底层类实际存在的 send 方法
    serial_->send(data_str); 
    
    return true;
  } catch (const std::exception& e) {
    return false;
  }
}

// ==========================================
// 2. 接收数据 (电控 -> 视觉) 解决粘包与半包
// ==========================================
bool SerialCodec::try_get_recv_data_for(hnurm_interfaces::msg::Serial2Vision& recv_data, int timeout_ms)
{
  auto start_time = std::chrono::steady_clock::now();
  
  // 新协议的包长是绝对固定的
  constexpr size_t EXPECTED_PACKAGE_LENGTH = sizeof(Serial2VisionPacket);

  while (true) {
    std::string new_data;
    
    // 【修改点】：使用你底层类实际存在的 try_recv_for 方法，尝试读取(最多阻塞1ms)
    if (serial_->try_recv_for(new_data, 1)) {
      std::lock_guard<std::mutex> lock(mutex_);
      recv_buf_ += new_data;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    
    // 核心解包：只要缓冲区足够长，就尝试解析
    while (recv_buf_.length() >= EXPECTED_PACKAGE_LENGTH) {
      
      // 0xA5 是我们在 protocol.h 里定义的 PROTOCOL_CMD_ID 帧头
      if (static_cast<uint8_t>(recv_buf_[0]) != 0xA5) { 
        recv_buf_.erase(0, 1);
        continue; 
      }

      std::string candidate_packet = recv_buf_.substr(0, EXPECTED_PACKAGE_LENGTH);

      if (Protocol::decode(candidate_packet, recv_data)) {
        recv_buf_.erase(0, EXPECTED_PACKAGE_LENGTH);
        return true; 
      } else {
        recv_buf_.erase(0, 1); // 校验失败，剔除假帧头
      }
    }

    // 3. 超时控制
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    if (elapsed_ms >= timeout_ms) {
      break; 
    }
  }

  return false;
}

}  // namespace hnurm