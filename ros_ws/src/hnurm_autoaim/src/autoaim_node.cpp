#include <rclcpp/rclcpp.hpp>
#include <hnurm_interfaces/msg/serial2_vision.hpp>
#include <hnurm_interfaces/msg/vision2_serial.hpp>
#include "aimer/auto_aim/predictor/enemy_predictor/enemy_predictor.hpp"
#include "aimer/auto_aim/base/defs.hpp"
#include "core_io/robot.hpp"
#include "UltraMultiThread/include/umt/ObjManager.hpp"
#include <thread>

namespace base {
    void parameter_run(const std::string& param_file_path);
    void wait_for_param(const std::string& name);
}

using namespace std::chrono_literals;

class AutoAimMockNode : public rclcpp::Node {
public:
    AutoAimMockNode() : Node("autoaim_mock_node") {
        std::thread([]() {
            base::parameter_run("assets/base.param.toml");
        }).detach();

        base::wait_for_param("ok");
        RCLCPP_INFO(this->get_logger(), "参数表加载完毕！");

        predictor_ = std::make_unique<aimer::EnemyPredictor>();
        
        cmd_pub_ = this->create_publisher<hnurm_interfaces::msg::Vision2Serial>("/uart/vision_to_serial", 10);
        uart_sub_ = this->create_subscription<hnurm_interfaces::msg::Serial2Vision>(
            "/uart/serial_to_vision", 10,
            std::bind(&AutoAimMockNode::uart_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "AutoAim 仿真节点完美启动，等待数据输入！");
    }
private:
    void uart_callback(const hnurm_interfaces::msg::Serial2Vision::SharedPtr msg) {
        auto status = umt::ObjManager<RobotStatus>::find_or_create("robot_status");
        status->enemy_color = msg->enemy_color;
        status->bullet_speed = 28.0f;
        status->program_mode = ProgramMode::AUTOAIM;

        aimer::DetectionResult mock_result;
        
        // 【核心修复：强制单调递增的时间戳，彻底杜绝 dt=0】
        static double last_t = 0.0;
        double current_t = this->now().seconds();
        if (current_t <= last_t) {
            current_t = last_t + 0.01; // 强制至少增加 10ms
        }
        last_t = current_t;
        mock_result.timestamp = current_t; 
        
        mock_result.img = cv::Mat::zeros(480, 640, CV_8UC3);
        mock_result.q = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
        
        aimer::DetectedArmor fake_armor;
        fake_armor.color = msg->enemy_color;
        fake_armor.number = 3; 
        fake_armor.conf = 1.0f;
        fake_armor.conf_class = 1.0f;
        
        static float offset = 0.0f;
        offset += 1.5f; 
        if (offset > 150.0f) offset = -150.0f;

        fake_armor.pts[0] = cv::Point2f(300 + offset, 220);
        fake_armor.pts[1] = cv::Point2f(300 + offset, 260);
        fake_armor.pts[2] = cv::Point2f(340 + offset, 260);
        fake_armor.pts[3] = cv::Point2f(340 + offset, 220);

        mock_result.armors.push_back(fake_armor);

        try {
            RobotCmd cmd = predictor_->predict(mock_result);

            hnurm_interfaces::msg::Vision2Serial out_cmd;
            out_cmd.target_yaw = cmd.yaw;
            out_cmd.target_pitch = cmd.pitch;
            out_cmd.seq_id = msg->ack_seq_id + 1;
            
            if (cmd.shoot == ShootMode::SHOOT_NOW) {
                out_cmd.shoot_flag = 2;
                RCLCPP_INFO(this->get_logger(), "🔥 [开火!] 提前量 Y/P: [%.2f, %.2f]", cmd.yaw, cmd.pitch);
            } else if (cmd.shoot == ShootMode::TRACKING) {
                out_cmd.shoot_flag = 1;
                RCLCPP_INFO(this->get_logger(), "👀 [追踪中...] 目标移动中");
            }
            cmd_pub_->publish(out_cmd);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "C++ Exception 捕获: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "未知异常发生！");
        }
    }
    std::unique_ptr<aimer::EnemyPredictor> predictor_;
    rclcpp::Publisher<hnurm_interfaces::msg::Vision2Serial>::SharedPtr cmd_pub_;
    rclcpp::Subscription<hnurm_interfaces::msg::Serial2Vision>::SharedPtr uart_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoAimMockNode>());
    rclcpp::shutdown();
    return 0;
}