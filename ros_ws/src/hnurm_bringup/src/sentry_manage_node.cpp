//
// Created by rm on 24-4-8.
//

#include "hnurm_bringup/sentry_manage_node.h"

#include <angles/angles.h>

namespace hnurm
{

SentryManageNode::SentryManageNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("SentryManageNode", options)
{
  init_params();

  master_res_sub_ = this->create_subscription<hnurm_interfaces::msg::VisionSendData>(
    master_res_sub_topic_, 10,
    std::bind(&SentryManageNode::master_res_callback, this, std::placeholders::_1));
  slave_res_sub_ = this->create_subscription<hnurm_interfaces::msg::VisionSendData>(
    slave_res_sub_topic_, 10,
    std::bind(&SentryManageNode::slave_res_callback, this, std::placeholders::_1));

  master_res_pub_ =
    this->create_publisher<hnurm_interfaces::msg::VisionSendData>(master_res_pub_topic_, 10);
  slave_res_pub_ =
    this->create_publisher<hnurm_interfaces::msg::VisionSendData>(slave_res_pub_topic_, 10);
}

void SentryManageNode::init_params()
{
  master_res_sub_topic_ = this->declare_parameter("/master_res", "/left/vision_send_res");
  slave_res_sub_topic_  = this->declare_parameter("/slave_res", "/right/vision_send_res");

  master_res_pub_topic_ = this->declare_parameter("/master_send_res", "/left/vision_send_data");
  slave_res_pub_topic_  = this->declare_parameter("/slave_send_res", "/right/vision_send_data");

  use_cross_fire         = this->declare_parameter("/use_cross_fire", true);
  master_to_slave_offset = this->declare_parameter("/master_to_slave_offset", 0.335);
}

void SentryManageNode::master_res_callback(
  hnurm_interfaces::msg::VisionSendData::ConstSharedPtr msg)
{
  if (use_cross_fire) {
    if (msg->target_state.data == hnurm_interfaces::msg::TargetState::FIRE)
      master_res_buffer_.push_front(*msg);
    else
      master_res_buffer_.clear();
  }
  master_res_pub_->publish(*msg);
}

void SentryManageNode::slave_res_callback(hnurm_interfaces::msg::VisionSendData::ConstSharedPtr msg)
{
  if (!use_cross_fire) {
    slave_res_pub_->publish(*msg);
    return;
  }

  if (msg->target_state.data == hnurm_interfaces::msg::TargetState::FIRE) {
    // this means slave head has target
    slave_res_pub_->publish(*msg);
    return;
  }

  if (msg->target_state.data == hnurm_interfaces::msg::TargetState::CONVERGING) {
    // this means slave head has target
    slave_res_pub_->publish(*msg);
    return;
  }

  if (master_res_buffer_.empty()) {
    // no target in master
    slave_res_pub_->publish(*msg);
    return;
  }

  // cross fire
  auto master_res = master_res_buffer_.front();
  // auto master_time = rclcpp::Time(master_res.header.stamp);
  // auto slave_time  = rclcpp::Time(msg->header.stamp);
  // auto diff        = (master_time - slave_time).seconds();
  // if(diff > 1)
  // {
  //     // master target is too old
  //     master_res_buffer_.clear();
  //     slave_res_pub_->publish(*msg);
  //     return;
  // }
  RCLCPP_INFO(this->get_logger(), "crossing fire");

  // get target
  const auto& d   = master_to_slave_offset;
  const auto& r   = master_res.target_distance;
  const auto  phi = angles::from_degrees(master_res.yaw);
  // theta is target
  auto theta     = static_cast<float>(atan(r * sin(phi) / (r * cos(phi) - d)));
  auto total_yaw = angles::from_degrees(msg->yaw);
  // get angular distance in degs
  auto da = angles::to_degrees(angles::shortest_angular_distance(total_yaw, theta));
  // calculate final target in degs
  auto yaw_target = msg->yaw + da + 12;
  bool status_    = true;

  // if(master_res.yaw > 180)
  // {
  //     yaw_target = yaw_target - 180;
  // }
  if (master_res.yaw > 90) {
    yaw_target = yaw_target - 195;
  }

  if (master_res.yaw > 80 && master_res.yaw < 110) {
    status_ = false;
  }

  if (status_) {
    hnurm_interfaces::msg::VisionSendData res;
    res.header            = msg->header;
    res.target_state.data = hnurm_interfaces::msg::TargetState::CONVERGING;
    res.target_type       = master_res.target_type;
    res.pitch             = master_res.pitch;
    res.yaw               = yaw_target;
    res.control_id        = 1.000;
    slave_res_pub_->publish(res);
    master_res_buffer_.pop_front();
    if (master_res_buffer_.size() > 10'000) {
      master_res_buffer_.clear();
    }
  }
}
}  // namespace hnurm

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto                node = std::make_shared<hnurm::SentryManageNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}