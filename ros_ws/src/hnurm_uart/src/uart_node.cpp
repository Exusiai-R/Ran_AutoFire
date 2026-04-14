#include "hnurm_uart/uart_node.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <filesystem>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

namespace hnurm
{
void UartNode::run()
{
  // check if /dev/serial/by-id/ is created
  while (!std::filesystem::exists("/dev/serial/by-id/")) {
    RCLCPP_WARN(logger, "Waiting for /dev/serial/by-id/ to be created");
    std::this_thread::sleep_for(1s);
  }

  recv_topic_ = this->declare_parameter("recv_topic", "vision_recv_data");
  send_topic_ = this->declare_parameter("send_topic", "vision_send_data");

  use_control_id_ = this->declare_parameter("use_control_id", false);
  control_id_     = static_cast<float>(this->declare_parameter("control_id", 1.0f));

  serial_codec_    = new SerialCodec(shared_from_this());
  callback_group1_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callback_group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  use_distribution_ = this->declare_parameter("use_distribution", false);
  master_ns_        = this->declare_parameter("master_ns", "main");
  slave_ns_         = this->declare_parameter("slave_ns", "right");
  decision_send_topic_ =
    this->declare_parameter("decision_send_topic", "/decision/vision_send_data");
  if (use_distribution_) {
    // master_pub_ = create_publisher<hnurm_interfaces::msg::VisionRecvData>(
    //     master_ns_ + "/" + recv_topic_, rclcpp::SensorDataQoS()
    // );
    master_pub_ =
      create_publisher<hnurm_interfaces::msg::VisionRecvData>(recv_topic_, rclcpp::SensorDataQoS());
    //defalt main thread do not use namespace
    slave_pub = create_publisher<hnurm_interfaces::msg::VisionRecvData>(
      slave_ns_ + "/" + recv_topic_, rclcpp::SensorDataQoS()
    );
    RCLCPP_INFO_STREAM(logger, "using distribution");
    RCLCPP_INFO_STREAM(logger, "master ns: " << master_ns_ << ", slave ns: " << slave_ns_);
  }
  else {
    master_pub_ =
      create_publisher<hnurm_interfaces::msg::VisionRecvData>(recv_topic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO_STREAM(logger, "not using distribution");
  }

  auto sub_option           = rclcpp::SubscriptionOptions();
  sub_option.callback_group = callback_group2_;
  sub_                      = create_subscription<hnurm_interfaces::msg::VisionSendData>(
    send_topic_,
    rclcpp::ServicesQoS(),
    std::bind(&UartNode::sub_callback, shared_from_this(), std::placeholders::_1),
    sub_option
  );

  decision_sub_ = create_subscription<hnurm_interfaces::msg::VisionSendData>(
    decision_send_topic_,
    rclcpp::ServicesQoS(),
    std::bind(&UartNode::decision_sub_callback, shared_from_this(), std::placeholders::_1),
    sub_option
  );

  sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::ServicesQoS(),
    std::bind(&UartNode::sub_twist_callback, shared_from_this(), std::placeholders::_1),
    sub_option
  );

  // tf
  tf_broadcaster_     = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

  // 创建服务客户端
  set_self_color_client_ =
    create_client<hnurm_interfaces::srv::SetSelfColor>("/armor_detector_node/set_self_color");
  set_work_mode_client_ =
    create_client<hnurm_interfaces::srv::SetWorkMode>("/armor_detector_node/set_work_mode");

  uart_thread_ = std::thread([this]() {
    while (rclcpp::ok() && !stop_flag_) {
      timer_callback();
    }
    RCLCPP_WARN(logger, "uart thread exit");
  });
}

void UartNode::decision_sub_callback(hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
{
  msg->vel_x   = 2000.0;
  msg->vel_y   = 2000.0;
  msg->vel_yaw = 2000.0;
  //  RCLCPP_INFO(logger, "send decision data");
  if (serial_codec_->send_data(*msg)) RCLCPP_INFO(logger, "send decision data");
}

void UartNode::sub_callback(hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
{
  msg->vel_x   = 2000.0;
  msg->vel_y   = 2000.0;
  msg->vel_yaw = 2000.0;
  if (serial_codec_->send_data(*msg)) RCLCPP_INFO(logger, "send data");
}

void UartNode::sub_twist_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  hnurm_interfaces::msg::VisionSendData send_data;
  send_data.vel_x   = static_cast<float>(msg->linear.x);
  send_data.vel_y   = static_cast<float>(msg->linear.y);
  send_data.vel_yaw = static_cast<float>(msg->angular.z);
  if (serial_codec_->send_data(send_data)) RCLCPP_INFO(logger, "send data");
}

int counter = 0;

void UartNode::timer_callback()
{
  geometry_msgs::msg::TransformStamped  static_transform;
  geometry_msgs::msg::TransformStamped  transformStamped;
  tf2::Quaternion                       q;
  hnurm_interfaces::msg::VisionRecvData recv_data;
  if (serial_codec_->try_get_recv_data_for(recv_data)) {
    static auto last_time = this->now();
    auto        this_time = this->now();
    auto        call_dt   = (this_time - last_time).seconds();
    RCLCPP_INFO(this->get_logger(), "Callback FPS: %f", 1.0 / call_dt);
    last_time = this_time;

    // early return
    if (recv_data.self_color.data == hnurm_interfaces::msg::SelfColor::COLOR_NONE) {
      RCLCPP_WARN(logger, "self color not set, ignoring this msg");
      return;
    }

    // 检测配置变化并调用服务（只有成功后才更新 last_xxx_）
    if (recv_data.self_color.data != last_self_color_) {
      auto request             = std::make_shared<hnurm_interfaces::srv::SetSelfColor::Request>();
      request->self_color.data = recv_data.self_color.data;

      set_self_color_client_->async_send_request(
        request,
        [this, color = recv_data.self_color.data](
          rclcpp::Client<hnurm_interfaces::srv::SetSelfColor>::SharedFuture future
        ) {
          try {
            auto response = future.get();
            if (response->success) {
              last_self_color_ = color;  // 只在成功时更新
              RCLCPP_INFO(logger, "Updated self_color to %d: %s", color, response->message.c_str());
            }
            else {
              RCLCPP_ERROR(
                logger, "Failed to update self_color: %s (will retry)", response->message.c_str()
              );
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "Service call exception: %s (will retry)", e.what());
          }
        }
      );
    }

    if (recv_data.work_mode.data != last_work_mode_) {
      auto request            = std::make_shared<hnurm_interfaces::srv::SetWorkMode::Request>();
      request->work_mode.data = recv_data.work_mode.data;

      set_work_mode_client_->async_send_request(
        request,
        [this, mode = recv_data.work_mode.data](
          rclcpp::Client<hnurm_interfaces::srv::SetWorkMode>::SharedFuture future
        ) {
          try {
            auto response = future.get();
            if (response->success) {
              last_work_mode_ = mode;  // 只在成功时更新
              RCLCPP_INFO(logger, "Updated work_mode to %d: %s", mode, response->message.c_str());
            }
            else {
              RCLCPP_ERROR(
                logger, "Failed to update work_mode: %s (will retry)", response->message.c_str()
              );
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "Service call exception: %s (will retry)", e.what());
          }
        }
      );
    }

    // if(use_distribution_)
    if (use_distribution_ && use_control_id_) {
      // use control id to distribute data
      if (recv_data.control_id < 0) {
        recv_data.header.stamp    = now();
        recv_data.header.frame_id = "serial";
        master_pub_->publish(recv_data);
      }
      else if (recv_data.control_id > 0) {
        recv_data.header.stamp    = now();
        recv_data.header.frame_id = "serial";
        slave_pub->publish(recv_data);
      }
      else {
        RCLCPP_WARN_ONCE(logger, "control_id is illegal[0], ignoring further msg");
      }
    }
    else if (use_control_id_ && (control_id_ != recv_data.control_id)) {
      return;
    }
    else {
      recv_data.header.stamp    = now();
      recv_data.header.frame_id = "serial";
      master_pub_->publish(recv_data);
      /*************全向感知相机只做识别，发布slave话题用于触发detect node的callback*******************/
      if (use_distribution_) slave_pub->publish(recv_data);
      /*******************************/
      if (init_imu_.is_init) {
        static_transform.header.stamp            = now();
        static_transform.header.frame_id         = "imu_init";
        static_transform.child_frame_id          = "camera_init";
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        q.setRPY(0.0, 0.0, init_imu_.yaw * M_PI / 180.0);  // 旋转角度（单位：弧度）
        // RCLCPP_INFO(logger, "imu_init to camera_init pitch:%f yaw:%f", init_imu_.pitch, init_imu_.yaw);
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();
        static_broadcaster_->sendTransform(static_transform);
      }
      else {
        // counter++;
        // if(counter>1000)
        // {
        init_imu_.yaw     = recv_data.yaw;
        init_imu_.pitch   = recv_data.pitch;
        init_imu_.is_init = true;
        // }
      }
      //发布imu坐标系的动态坐标变换
      transformStamped.header.stamp            = now();
      transformStamped.header.frame_id         = "imu_init";
      transformStamped.child_frame_id          = "imu_link";
      transformStamped.transform.translation.x = 0.0;
      transformStamped.transform.translation.y = 0.0;
      transformStamped.transform.translation.z = 0.0;
      q.setRPY(0, -recv_data.pitch * M_PI / 180.0, recv_data.yaw * M_PI / 180.0);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
      // tf_broadcaster_->sendTransform(transformStamped);
    }
    //        RCLCPP_INFO(logger, "recv data: %f, %f, %f", recv_data.pitch, recv_data.yaw, recv_data.roll);
  }
  else {
    if (error_cnt_++ > 100) {
      std::thread([this]() { re_launch(); }).detach();
    }
    std::this_thread::sleep_for(10ms);
  }
}

void UartNode::re_launch()
{
  stop_flag_ = true;
  uart_thread_.join();
  stop_flag_ = false;

  // check if /dev/serial/by-id/ is created
  while (!std::filesystem::exists("/dev/serial/by-id/")) {
    RCLCPP_WARN(logger, "Waiting for /dev/serial/by-id/ to be created");
    std::this_thread::sleep_for(1s);
  }

  error_cnt_ = 0;
  serial_codec_->init_port();
  uart_thread_ = std::thread([this]() {
    while (rclcpp::ok() && !stop_flag_) {
      timer_callback();
    }
    RCLCPP_WARN(logger, "uart thread exit");
  });
}

}  // namespace hnurm
