#include "hnurm_camera/camera_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

namespace hnurm
{
CameraNode::CameraNode(const rclcpp::NodeOptions& options)
: Node("camera_node", options)
, stop_thread_(false)
{
  RCLCPP_INFO(this->get_logger(), "CameraNode is running");
  camera_img_topic_ = this->declare_parameter("camera_img_topic", "image");
  camera_id_        = this->declare_parameter("camera_id", "2BDFA7447491");
  camera_name_      = this->declare_parameter("camera_name", camera_id_);
  camera_info_url_  = this->declare_parameter(
    "camera_info_url", "package://hnurm_bringup/config/camera_info/" + camera_id_ + ".yaml"
  );

  cam_ = std::make_shared<HKcam>(shared_from_this());

  // 配置发布者选项以启用进程内通信和零拷贝
  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  // 创建图像发布者
  pub_img_ = this->create_publisher<sensor_msgs::msg::Image>(
    camera_img_topic_, rclcpp::SensorDataQoS(), pub_options
  );

  // 创建相机信息发布者
  camera_info_topic_ = "/camera_info";
  pub_cam_info_      = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS()
  );

  // 初始化相机信息管理器
  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);

  // 加载相机标定参数
  if (!camera_info_url_.empty()) {
    if (camera_info_manager_->loadCameraInfo(camera_info_url_)) {
      RCLCPP_INFO(
        this->get_logger(), "Loaded camera calibration from: %s", camera_info_url_.c_str()
      );
    }
    else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to load camera calibration from: %s", camera_info_url_.c_str()
      );
    }
  }
  else {
    RCLCPP_INFO(
      this->get_logger(), "No camera calibration file specified, using default parameters"
    );
  }

  // 获取相机信息消息
  cam_info_msg_ = camera_info_manager_->getCameraInfo();

  // 启动采集线程
  capture_thread_ = std::thread([this]() {
    while (rclcpp::ok() && !stop_thread_) {
      timer_callback();
    }
  });

  RCLCPP_INFO(this->get_logger(), "CameraNode 组件初始化完成");
}

CameraNode::~CameraNode()
{
  stop_thread_ = true;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
    RCLCPP_INFO(this->get_logger(), "CameraNode capture thread joined");
  }
}

void CameraNode::timer_callback()
{
  // static auto          prev    = this->now();

  // 初始化图像
  img_msg_ = std::make_unique<sensor_msgs::msg::Image>();

  if (cam_->GetFrame(img_msg_->data, img_msg_->header.stamp)) {
    img_msg_->header.frame_id = "camera";
    img_msg_->encoding        = sensor_msgs::image_encodings::BGR8;
    img_msg_->height          = cam_->_nHeight;
    img_msg_->width           = cam_->_nWidth;
    img_msg_->is_bigendian    = 0;
    img_msg_->step            = cam_->_nWidth * 3;
    // 更新相机信息消息的头信息（与图像消息同步）
    cam_info_msg_.header.stamp    = img_msg_->header.stamp;
    cam_info_msg_.header.frame_id = img_msg_->header.frame_id;

    // 调试信息(图像)
    // cv::Mat frame_(img_msg_->height, img_msg_->width, CV_8UC3, img_msg_->data.data());
    // std::stringstream ss;
    // ss << "pid: " << getpid() << ", ptr: " << img_msg_.get();
    // cv::putText(
    //   frame_, ss.str(), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0)
    // );

    // 发布图像消息
    pub_img_->publish(std::move(img_msg_));

    // 发布相机信息消息
    pub_cam_info_->publish(cam_info_msg_);

    // auto now = this->now();
    // RCLCPP_INFO(this->get_logger(), "Capture FPS: %f", 1.0 / (now - prev).seconds());
    // prev = now;
  }
}
}  // namespace hnurm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hnurm::CameraNode)
