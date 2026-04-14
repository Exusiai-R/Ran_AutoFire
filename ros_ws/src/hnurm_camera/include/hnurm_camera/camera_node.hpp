#pragma once
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hnurm_camera/Camera.h"

namespace hnurm
{
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions& options);
  ~CameraNode();

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      pub_img_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info_;
  std::unique_ptr<camera_info_manager::CameraInfoManager>    camera_info_manager_;
  // msgs
  sensor_msgs::msg::Image::UniquePtr img_msg_;
  sensor_msgs::msg::CameraInfo       cam_info_msg_;

  std::string camera_img_topic_;
  std::string camera_info_topic_;
  std::string camera_name_;
  std::string camera_id_;
  std::string camera_info_url_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<HKcam>       cam_;
  std::thread                  capture_thread_;
  std::atomic<bool>            stop_thread_;

private:
  void timer_callback();

protected:
  std::shared_ptr<CameraNode> shared_from_this()
  {
    RCLCPP_INFO(this->get_logger(), "init camera");
    return std::static_pointer_cast<CameraNode>(std::shared_ptr<rclcpp::Node>(this));
    RCLCPP_INFO(this->get_logger(), "init camera over");
  }
};
}  // namespace hnurm
