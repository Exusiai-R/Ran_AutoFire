/* Refactored for LMTD Low-Latency Architecture */

#pragma once
#include <stdint.h>
#include <string>

// 1. 替换为全新的精简版 ROS 2 消息头文件
#include <hnurm_interfaces/msg/serial2_vision.hpp>
#include <hnurm_interfaces/msg/vision2_serial.hpp>

namespace hnurm
{

// 2. 强制 1 字节对齐，这是上位机(Linux)和下位机(STM32)内存完美映射的物理契约
#pragma pack(push, 1)

/**
 * @brief 电控发给视觉的底层数据包 (严格对应 Serial2Vision.msg)
 */
struct Serial2VisionPacket {
  uint8_t  sof;              // 帧头 (如 0xA5)
  float    pitch;            // 陀螺仪绝对 Pitch
  float    yaw;              // 陀螺仪绝对 Yaw
  float    roll;             // 陀螺仪绝对 Roll
  float    bullet_speed;     // 当前实际弹速
  uint8_t  enemy_color;      // 目标颜色
  uint8_t  program_mode;     // 运行模式
  uint16_t ack_seq_id;       // LMTD 延迟闭环确认号
  uint8_t  shoot_feedback;   // 击发反馈
  float    vel_x;            // 预留底盘 x 速度
  float    vel_y;            // 预留底盘 y 速度
  float    vel_z;            // 预留底盘 z 速度
  uint16_t crc16;            // 整包 CRC16 校验码
};

/**
 * @brief 视觉发给电控的底层数据包 (严格对应 Vision2Serial.msg)
 */
struct Vision2SerialPacket {
  uint8_t  sof;              // 帧头 (如 0xA5)
  float    target_yaw;       // 预测后目标绝对 Yaw
  float    target_pitch;     // 预测后目标绝对 Pitch
  uint8_t  shoot_flag;       // 0: 丢失/巡逻, 1: 跟踪, 2: 允许开火
  uint16_t seq_id;           // 视觉自增序列号 (Ping)
  uint16_t crc16;            // 整包 CRC16 校验码
};

#pragma pack(pop)

class Protocol
{
protected:
  // 如果电控依然坚持复杂的定长帧头 (如包含 data_length 和 crc8)，可保留此宏。
  // 但强烈建议与电控协商，直接按上方 Packet 结构体整体发送和接收。
  static short constexpr PROTOCOL_CMD_ID = 0XA5; 

public:
  // 3. 接口更新：使用最新的消息类型
  static std::string encode(const hnurm_interfaces::msg::Vision2Serial& data);
  static bool decode(const std::string& s, hnurm_interfaces::msg::Serial2Vision& decoded_data);

private:
  // 4. 清理历史包袱：删除了臃肿的 get_protocol_info_vision 和 get_protocol_send_data_vision
  // 因为现在可以直接使用 reinterpret_cast<Vision2SerialPacket*> 进行 O(1) 内存拷贝

  // 5. 保留官方的 CRC 校验函数供 encode/decode 调用
  static uint8_t Get_CRC8_Check(uint8_t* pchMessage, uint16_t dwLength);
  static uint8_t CRC8_Check_Sum(uint8_t* pchMessage, uint16_t dwLength);
  static uint16_t Get_CRC16_Check(uint8_t* pchMessage, uint32_t dwLength);
  static uint16_t CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
};

}  // namespace hnurm