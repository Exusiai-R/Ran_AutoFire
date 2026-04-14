//
// Created by rm on 24-4-8.
//

#ifndef PROJECT_BRINGUPNODE_H
#define PROJECT_BRINGUPNODE_H

#include <deque>
#include <rclcpp/rclcpp.hpp>

#include "hnurm_interfaces/msg/vision_send_data.hpp"

namespace hnurm
{
class SentryManageNode : public rclcpp::Node
{
public:
  explicit SentryManageNode(const rclcpp::NodeOptions& options);

  void init_params();

  void master_res_callback(hnurm_interfaces::msg::VisionSendData::ConstSharedPtr msg);
  void slave_res_callback(hnurm_interfaces::msg::VisionSendData::ConstSharedPtr msg);

private:
  // subs
  rclcpp::Subscription<hnurm_interfaces::msg::VisionSendData>::SharedPtr master_res_sub_;
  rclcpp::Subscription<hnurm_interfaces::msg::VisionSendData>::SharedPtr slave_res_sub_;

  // pubs
  rclcpp::Publisher<hnurm_interfaces::msg::VisionSendData>::SharedPtr master_res_pub_;
  rclcpp::Publisher<hnurm_interfaces::msg::VisionSendData>::SharedPtr slave_res_pub_;

  // topics
  std::string master_res_sub_topic_;
  std::string slave_res_sub_topic_;

  std::string master_res_pub_topic_;
  std::string slave_res_pub_topic_;

  // buffers
  std::mutex                                        master_res_buffer_mutex_;
  std::deque<hnurm_interfaces::msg::VisionSendData> master_res_buffer_;

  // params
  bool   use_cross_fire         = false;
  double master_to_slave_offset = 0.335;
};
}  // namespace hnurm

#endif  // PROJECT_BRINGUPNODE_H