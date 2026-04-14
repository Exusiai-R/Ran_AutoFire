/*
    @SEASKY---2020/09/05
*/

/*
    kiko@idiospace.com --- 2021.01
*/

#include "hnurm_uart/protocol.h"

#include <cstring>

#include "hnurm_uart/bsp_crc16.h"
#include "hnurm_uart/bsp_crc8.h"

namespace hnurm
{

// ==========================================
// 视觉发给电控 (Encode: ROS 2 Msg -> String bytes)
// ==========================================
std::string Protocol::encode(const hnurm_interfaces::msg::Vision2Serial& data) {
    Vision2SerialPacket packet{}; // 初始化清零
    
    // 1. 填充数据
    packet.sof = PROTOCOL_CMD_ID; // 0xA5
    packet.target_yaw = data.target_yaw;
    packet.target_pitch = data.target_pitch;
    packet.shoot_flag = data.shoot_flag;
    packet.seq_id = data.seq_id;
    
    // 2. 计算 CRC16 (校验除去 crc16 字段本身的所有前置字节)
    packet.crc16 = Get_CRC16_Check(reinterpret_cast<uint8_t*>(&packet), sizeof(Vision2SerialPacket) - 2);
    
    // 3. 直接将结构体内存块转为 std::string 发送
    return std::string(reinterpret_cast<const char*>(&packet), sizeof(Vision2SerialPacket));
}

// ==========================================
// 电控发给视觉 (Decode: String bytes -> ROS 2 Msg)
// ==========================================
bool Protocol::decode(const std::string& s, hnurm_interfaces::msg::Serial2Vision& decoded_data) {
    // 1. 长度与帧头校验
    if (s.length() != sizeof(Serial2VisionPacket)) {
        return false;
    }
    
    // 2. O(1) 零拷贝强转
    const auto* packet = reinterpret_cast<const Serial2VisionPacket*>(s.data());
    
    if (packet->sof != PROTOCOL_CMD_ID) {
        return false;
    }
    
    // 3. CRC16 校验 (可选，强烈建议保留以防止串口误码)
    // uint16_t expected_crc = Get_CRC16_Check((uint8_t*)packet, sizeof(Serial2VisionPacket) - 2);
    // if (packet->crc16 != expected_crc) return false;

    // 4. 数据映射到 ROS 2 Message
    decoded_data.pitch = packet->pitch;
    decoded_data.yaw = packet->yaw;
    decoded_data.roll = packet->roll;
    decoded_data.bullet_speed = packet->bullet_speed;
    decoded_data.enemy_color = packet->enemy_color;
    decoded_data.program_mode = packet->program_mode;
    decoded_data.ack_seq_id = packet->ack_seq_id;
    decoded_data.shoot_feedback = packet->shoot_feedback;
    decoded_data.vel_x = packet->vel_x;
    decoded_data.vel_y = packet->vel_y;
    decoded_data.vel_z = packet->vel_z;
    
    return true;
}

uint8_t Protocol::Get_CRC8_Check(uint8_t* pchMessage, uint16_t dwLength)
{
  return crc_8(pchMessage, dwLength);
}

uint8_t Protocol::CRC8_Check_Sum(uint8_t* pchMessage, uint16_t dwLength)
{
  uint8_t ucExpected = 0;
  if ((pchMessage == 0) || (dwLength <= 2)) return 0;
  ucExpected = Get_CRC8_Check(pchMessage, dwLength - 1);
  return (ucExpected == pchMessage[dwLength - 1]);
}

uint16_t Protocol::Get_CRC16_Check(uint8_t* pchMessage, uint32_t dwLength)
{
  return crc_16(pchMessage, dwLength);
}

uint16_t Protocol::CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength)
{
  uint16_t wExpected = 0;
  if ((pchMessage == 0) || (dwLength <= 2)) {
    return 0;
  }
  wExpected = Get_CRC16_Check(pchMessage, dwLength - 2);
  return (
    ((wExpected & 0xff) == pchMessage[dwLength - 2]) &&
    (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1])
  );
}


}  // namespace hnurm
