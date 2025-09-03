#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include "sensor_msgs/msg/imu.hpp"

#include "ksenos_ground_msgs/msg/rpy.hpp"
#include "std_msgs/msg/float32.hpp"

class RotationCounterNode : public rclcpp::Node
{
public:
    RotationCounterNode()
        : Node("yaw_angle_serialization_node"),
          previous_yaw_(0.0),
          total_rotation_(0.0),
          is_first_measurement_(true)
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_angular_topic_name", "/rpy");
        this->declare_parameter<std::string>("input_counter_reset_topic_name", "/counter_reset");
        this->declare_parameter<std::string>("output_serialized_yaw_angle_topic_name", "/yaw_angle_serialization");

        // パラメータの取得
        std::string input_angular_topic_name;
        this->get_parameter("input_angular_topic_name", input_angular_topic_name);
        std::string input_counter_reset_topic_name;
        this->get_parameter("input_counter_reset_topic_name", input_counter_reset_topic_name);
        std::string output_serialized_yaw_angle_topic_name;
        this->get_parameter("output_serialized_yaw_angle_topic_name", output_serialized_yaw_angle_topic_name);

        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        rpy_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::Rpy>(
            input_angular_topic_name, qos,
            std::bind(&RotationCounterNode::rpy_callback, this, std::placeholders::_1));

        rclcpp::QoS qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        counter_reset_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            input_counter_reset_topic_name, qos_reliable,
            std::bind(&RotationCounterNode::counter_reset_callback, this, std::placeholders::_1));

        yaw_angle_serialization_pub_ = this->create_publisher<std_msgs::msg::Float32>(output_serialized_yaw_angle_topic_name, qos_reliable);

        RCLCPP_INFO(this->get_logger(), "yaw_angle_serialization_node has been started.");
    }

private:
    void rpy_callback(const ksenos_ground_msgs::msg::Rpy::SharedPtr msg)
    {
        double current_yaw = msg->yaw;

        if (is_first_measurement_)
        {
            // 最初の測定値の場合、そのまま記録
            previous_yaw_ = current_yaw;
            total_rotation_ = current_yaw;
            is_first_measurement_ = false;
        }
        else
        {
            // yaw角の差分を計算
            double delta_yaw = current_yaw - previous_yaw_;

            // ラップアラウンドを検出して補正
            if (delta_yaw > M_PI)
            {
                // +π → -π のジャンプ（右回転でπを超えた）
                delta_yaw -= 2.0 * M_PI;
            }
            else if (delta_yaw < -M_PI)
            {
                // -π → +π のジャンプ（左回転で-πを超えた）
                delta_yaw += 2.0 * M_PI;
            }

            // 累積回転量を更新
            total_rotation_ += delta_yaw;
            previous_yaw_ = current_yaw;
        }

        // 連続化されたyaw角を配信
        publishSerializedYaw();
    }
    void publishSerializedYaw()
    {
        std_msgs::msg::Float32 serialized_yaw_angle_msg;
        serialized_yaw_angle_msg.data = total_rotation_;
        yaw_angle_serialization_pub_->publish(serialized_yaw_angle_msg);
    }

    void counter_reset_callback(const std_msgs::msg::Float32::SharedPtr /* msg */)
    {
        is_first_measurement_ = true;
        total_rotation_ = 0.0;
        previous_yaw_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Yaw serialization counter has been reset.");
    }

    // サブスクライバー
    rclcpp::Subscription<ksenos_ground_msgs::msg::Rpy>::SharedPtr rpy_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr counter_reset_subscriber_;

    // パブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_angle_serialization_pub_;

    // 角度関連の状態変数
    double previous_yaw_;       // 前回のyaw角（-π～π）
    double total_rotation_;     // 累積回転量（連続値）
    bool is_first_measurement_; // 初回測定フラグ
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotationCounterNode>());
    rclcpp::shutdown();
    return 0;
}