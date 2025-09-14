#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

class LatAccToTargetRollAngle : public rclcpp::Node
{
public:
    LatAccToTargetRollAngle(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("lat_acc_to_target_roll_angle", options)
    {
        // 重力加速度の定義 (m/s^2)
        g_ = 9.81;

        // パラメータの宣言と取得
        this->declare_parameter("max_roll_angle", 0.524); // デフォルト値: 30度 (0.524 rad)
        max_roll_angle_ = this->get_parameter("max_roll_angle").as_double();

        // Subscriber: 横加速度トピック
        lateral_acceleration_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "lateral_acceleration",
            10,
            std::bind(&LatAccToTargetRollAngle::lateral_acceleration_callback, this, std::placeholders::_1));

        // Publisher: 目標ロール角トピック
        target_roll_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_roll_angle", 10);

        RCLCPP_INFO(this->get_logger(), "LatAccToTargetRollAngle node has been started");
        RCLCPP_INFO(this->get_logger(), "Max roll angle: %.3f rad (%.1f deg)",
                    max_roll_angle_, max_roll_angle_ * 180.0 / M_PI);
    }

private:
    void lateral_acceleration_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 横加速度からロール角を計算
        // target_roll_angle = atan2(lateral_acceleration, g)
        float lateral_acceleration = msg->data;
        float target_roll_angle = std::atan2(lateral_acceleration, g_);

        // max_roll_angleでサチュレーション
        if (target_roll_angle > max_roll_angle_)
        {
            target_roll_angle = max_roll_angle_;
        }
        else if (target_roll_angle < -max_roll_angle_)
        {
            target_roll_angle = -max_roll_angle_;
        }

        // 結果をパブリッシュ
        auto target_roll_msg = std_msgs::msg::Float32();
        target_roll_msg.data = target_roll_angle;
        target_roll_angle_pub_->publish(target_roll_msg);

        // ログ出力（デバッグ用）
        RCLCPP_DEBUG(this->get_logger(),
                     "Lateral acceleration: %.3f m/s^2, Target roll angle: %.3f rad (%.3f deg)",
                     lateral_acceleration, target_roll_angle, target_roll_angle * 180.0 / M_PI);
    }

    // メンバ変数
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lateral_acceleration_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_roll_angle_pub_;
    float g_;               // 重力加速度
    double max_roll_angle_; // 最大ロール角制限値
};

// コンポーネント登録
RCLCPP_COMPONENTS_REGISTER_NODE(LatAccToTargetRollAngle)