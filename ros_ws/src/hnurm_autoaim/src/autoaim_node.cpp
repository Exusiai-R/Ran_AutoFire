#include <rclcpp/rclcpp.hpp>
#include <hnurm_interfaces/msg/serial2_vision.hpp>
#include <hnurm_interfaces/msg/vision2_serial.hpp>
#include "aimer/auto_aim/predictor/enemy_predictor/enemy_predictor.hpp"
#include "aimer/auto_aim/base/defs.hpp"
#include <thread>

// 提前声明 LMTD 祖传的参数加载函数
namespace base {
    void parameter_run(const std::string& param_file_path);
    void wait_for_param(const std::string& name);
}

using namespace std::chrono_literals;

class AutoAimMockNode : public rclcpp::Node {
public:
    AutoAimMockNode() : Node("autoaim_mock_node") {
        RCLCPP_INFO(this->get_logger(), "Starting parameter hot-reload thread...");
        
        // 1. 在后台分离线程中启动 TOML 参数热重载
        std::thread([]() {
            base::parameter_run("assets/base.param.toml");
        }).detach();

        // 2. 阻塞等待，直到参数系统发出 "ok" 信号，防止拿到一堆 0 导致段错误
        base::wait_for_param("ok");
        RCLCPP_INFO(this->get_logger(), "Parameters loaded perfectly!");

        // 3. 参数安全后，再安全地实例化核心预测器
        predictor_ = std::make_unique<aimer::EnemyPredictor>();
        
        cmd_pub_ = this->create_publisher<hnurm_interfaces::msg::Vision2Serial>("/uart/vision_to_serial", 10);
        uart_sub_ = this->create_subscription<hnurm_interfaces::msg::Serial2Vision>(
            "/uart/serial_to_vision", 10,
            std::bind(&AutoAimMockNode::uart_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "AutoAim Predictor Simulator Started (Bypassing YOLO)!");
    }
private:
    void uart_callback(const hnurm_interfaces::msg::Serial2Vision::SharedPtr msg) {
        aimer::DetectionResult mock_result;
        mock_result.timestamp = this->now().nanoseconds();
        mock_result.img = cv::Mat::zeros(480, 640, CV_8UC3);
        // 初始化四元数为单位阵，防止矩阵运算出 NaN
        mock_result.q = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
        
        // 伪造完美的步兵装甲板数据
        aimer::DetectedArmor fake_armor;
        fake_armor.color = msg->enemy_color;
        fake_armor.number = 3; 
        fake_armor.conf = 1.0f;
        fake_armor.conf_class = 1.0f;
        
        // 装甲板坐标放在图像绝对中心
        fake_armor.pts[0] = cv::Point2f(300, 220);
        fake_armor.pts[1] = cv::Point2f(300, 260);
        fake_armor.pts[2] = cv::Point2f(340, 260);
        fake_armor.pts[3] = cv::Point2f(340, 220);

        mock_result.armors.push_back(fake_armor);

        // 送入 LMTD 进行预测！
        RobotCmd cmd = predictor_->predict(mock_result);

        hnurm_interfaces::msg::Vision2Serial out_cmd;
        out_cmd.target_yaw = cmd.yaw;
        out_cmd.target_pitch = cmd.pitch;
        out_cmd.seq_id = msg->ack_seq_id + 1;
        
        if (cmd.shoot == ShootMode::SHOOT_NOW) {
            out_cmd.shoot_flag = 2;
            RCLCPP_INFO(this->get_logger(), "Fire! Target Y/P: [%.2f, %.2f]", cmd.yaw, cmd.pitch);
        } else if (cmd.shoot == ShootMode::TRACKING) {
            out_cmd.shoot_flag = 1;
        } else {
            out_cmd.shoot_flag = 0;
        }
        cmd_pub_->publish(out_cmd);
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