#include "rclcpp/rclcpp.hpp"
#include "ksenos_ground_msgs/msg/control_input.hpp"
#include <memory>

class UnityControlInputNode : public rclcpp::Node
{
public:
    UnityControlInputNode() : Node("unity_control_input")
    {
        // 初期値設定
        latest_aileron_ = 0.0;
        latest_elevator_ = 0.0;
        latest_rudder_ = 0.0;
        latest_throttle_ = 0.0;

        // サブスクライバーの作成
        aileron_sub_ = this->create_subscription<ksenos_ground_msgs::msg::ControlInput>(
            "/aileron_input", 10,
            std::bind(&UnityControlInputNode::aileronCallback, this, std::placeholders::_1));

        elevator_sub_ = this->create_subscription<ksenos_ground_msgs::msg::ControlInput>(
            "/elevator_input", 10,
            std::bind(&UnityControlInputNode::elevatorCallback, this, std::placeholders::_1));

        rudder_sub_ = this->create_subscription<ksenos_ground_msgs::msg::ControlInput>(
            "/rudder_input", 10,
            std::bind(&UnityControlInputNode::rudderCallback, this, std::placeholders::_1));

        throttle_sub_ = this->create_subscription<ksenos_ground_msgs::msg::ControlInput>(
            "/throttle_input", 10,
            std::bind(&UnityControlInputNode::throttleCallback, this, std::placeholders::_1));

        // パブリッシャーの作成
        control_input_pub_ = this->create_publisher<ksenos_ground_msgs::msg::ControlInput>(
            "/control_input", 10);

        // 20Hzタイマーの作成
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 50ms = 20Hz
            std::bind(&UnityControlInputNode::publishControlInput, this));

        RCLCPP_INFO(this->get_logger(), "Unity Control Input Node started");
    }

private:
    void aileronCallback(const ksenos_ground_msgs::msg::ControlInput::SharedPtr msg)
    {
        latest_aileron_ = msg->aileron;
        RCLCPP_DEBUG(this->get_logger(), "Received aileron input: %.3f", latest_aileron_);
    }

    void elevatorCallback(const ksenos_ground_msgs::msg::ControlInput::SharedPtr msg)
    {
        latest_elevator_ = msg->elevator;
        RCLCPP_DEBUG(this->get_logger(), "Received elevator input: %.3f", latest_elevator_);
    }

    void rudderCallback(const ksenos_ground_msgs::msg::ControlInput::SharedPtr msg)
    {
        latest_rudder_ = msg->rudder;
        RCLCPP_DEBUG(this->get_logger(), "Received rudder input: %.3f", latest_rudder_);
    }

    void throttleCallback(const ksenos_ground_msgs::msg::ControlInput::SharedPtr msg)
    {
        latest_throttle_ = msg->throttle;
        RCLCPP_DEBUG(this->get_logger(), "Received throttle input: %.3f", latest_throttle_);
    }

    void publishControlInput()
    {
        auto control_msg = ksenos_ground_msgs::msg::ControlInput();

        // ヘッダー情報を設定
        control_msg.header.stamp = this->get_clock()->now();
        control_msg.header.frame_id = "base_link";

        // 最新の制御値を設定（データが揃っていなくても過去の値を使用）
        control_msg.aileron = latest_aileron_;
        control_msg.elevator = latest_elevator_;
        control_msg.rudder = latest_rudder_;
        control_msg.throttle = latest_throttle_;

        // メッセージをパブリッシュ
        control_input_pub_->publish(control_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Published control input - Aileron: %.3f, Elevator: %.3f, Rudder: %.3f, Throttle: %.3f",
                     control_msg.aileron, control_msg.elevator, control_msg.rudder, control_msg.throttle);
    }

    // サブスクライバー
    rclcpp::Subscription<ksenos_ground_msgs::msg::ControlInput>::SharedPtr aileron_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::ControlInput>::SharedPtr elevator_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::ControlInput>::SharedPtr rudder_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::ControlInput>::SharedPtr throttle_sub_;

    // パブリッシャー
    rclcpp::Publisher<ksenos_ground_msgs::msg::ControlInput>::SharedPtr control_input_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;

    // 最新の制御値（過去のデータを保持）
    float latest_aileron_;
    float latest_elevator_;
    float latest_rudder_;
    float latest_throttle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnityControlInputNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
