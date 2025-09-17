#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ksenos_ground_msgs/msg/sbus_data.hpp>
#include <cmath>

class ModeEightTurningAngle : public rclcpp::Node
{
public:
    ModeEightTurningAngle(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("mode_eight_turning_angle", options)
    {
        // パラメータ宣言
        this->declare_parameter<double>("max_turn_radius", 15.0);
        this->declare_parameter<double>("min_turn_radius", 4.5);

        // パラメータ取得
        max_radius_ = this->get_parameter("max_turn_radius").as_double();
        min_radius_ = this->get_parameter("min_turn_radius").as_double();

        turn_mode_ = TurnMode::LEFT; // 左旋回から開始
        reference_yaw_ = 0.0;
        is_initialized_ = false;
        is_eight_turning_mode_ = false;

        // サブスクライバーとパブリッシャーの設定
        yaw_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "serialized_yaw", 10,
            std::bind(&ModeEightTurningAngle::yaw_callback, this, std::placeholders::_1));

        sbus_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&ModeEightTurningAngle::sbus_callback, this, std::placeholders::_1));

        turn_radius_publisher_ = this->create_publisher<std_msgs::msg::Float32>("turn_radius", 10);
        mode_publisher_ = this->create_publisher<std_msgs::msg::Int32>("turn_mode", 10);

        RCLCPP_INFO(this->get_logger(), "Mode Eight Turning Angle Node has been started");
        RCLCPP_INFO(this->get_logger(), "max_turn_radius: %.2f, min_turn_radius: %.2f", max_radius_, min_radius_);
    }

private:
    enum class TurnMode
    {
        LEFT = 0,
        RIGHT = 1
    };

    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // 新しいモード状態
        bool new_mode = (msg->autopilot_mode == "eight_turning");

        // モード離脱のエッジでのみリセット
        if (is_eight_turning_mode_ && !new_mode)
        {
            reset_state_();
            RCLCPP_INFO(this->get_logger(), "EightTurning mode exited; state reset");
        }

        // モード更新
        is_eight_turning_mode_ = new_mode;

        if (is_eight_turning_mode_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Eight turning mode active.");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Not in eight turning mode (current: %s).",
                         msg->autopilot_mode.c_str());
        }
    }

    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 8の字旋回モードでない場合は処理しない
        if (!is_eight_turning_mode_)
        {
            return;
        }

        float current_yaw = msg->data;

        // 初回の場合、基準角度を設定
        if (!is_initialized_)
        {
            reference_yaw_ = current_yaw;
            is_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Reference yaw initialized: %.2f deg", reference_yaw_ * 180.0 / M_PI);
        }

        // 現在のヨー角度から基準角度との差分を計算（normalize_angleは使わない）
        float angle_diff = current_yaw - reference_yaw_;

        // 現在のモードに応じて旋回半径を計算
        float target_turn_radius = calculate_turn_radius(angle_diff);

        // モード切り替えの判定
        check_mode_switch(angle_diff, current_yaw);

        // 結果をパブリッシュ
        auto radius_msg = std_msgs::msg::Float32();
        radius_msg.data = target_turn_radius;
        turn_radius_publisher_->publish(radius_msg);

        auto mode_msg = std_msgs::msg::Int32();
        mode_msg.data = static_cast<int32_t>(turn_mode_);
        mode_publisher_->publish(mode_msg);

        // デバッグ情報
        RCLCPP_DEBUG(this->get_logger(),
                     "Mode: %s, Angle diff: %.2f deg, Turn radius: %.2f",
                     (turn_mode_ == TurnMode::LEFT) ? "LEFT" : "RIGHT",
                     angle_diff * 180.0 / M_PI,
                     target_turn_radius);
    }

    float calculate_turn_radius(float angle_diff)
    {
        float turn_radius = 0.0;
        float angle_deg = angle_diff * 180.0 / M_PI;

        if (turn_mode_ == TurnMode::LEFT)
        {
            // 左旋回: 0° → +135° → +270°
            if (angle_deg >= 0.0 && angle_deg <= 135.0)
            {
                // 最大半径から最小半径へ減少
                float progress = angle_deg / 135.0;
                turn_radius = max_radius_ - (max_radius_ - min_radius_) * progress;
            }
            else if (angle_deg > 135.0 && angle_deg <= 270.0)
            {
                // 最小半径から最大半径へ増加
                float progress = (angle_deg - 135.0) / 135.0;
                turn_radius = min_radius_ + (max_radius_ - min_radius_) * progress;
            }
            else
            {
                // 想定外の角度の場合は最大半径
                turn_radius = max_radius_;
            }
        }
        else // RIGHT
        {
            // 右旋回: 0° → -135° → -270°
            if (angle_deg <= 0.0 && angle_deg >= -135.0)
            {
                // 負の最大半径から負の最小半径へ
                float progress = (-angle_deg) / 135.0;
                turn_radius = -max_radius_ + (max_radius_ - min_radius_) * progress;
            }
            else if (angle_deg < -135.0 && angle_deg >= -270.0)
            {
                // 負の最小半径から負の最大半径へ
                float progress = (-angle_deg - 135.0) / 135.0;
                turn_radius = -min_radius_ - (max_radius_ - min_radius_) * progress;
            }
            else
            {
                // 想定外の角度の場合は負の最大半径
                turn_radius = -max_radius_;
            }
        }

        return turn_radius;
    }

    void check_mode_switch(float angle_diff, float current_yaw)
    {
        float angle_deg = angle_diff * 180.0 / M_PI;

        if (turn_mode_ == TurnMode::LEFT)
        {
            // 左旋回が270°完了したら右旋回に切り替え
            if (angle_deg >= 270.0)
            {
                turn_mode_ = TurnMode::RIGHT;
                reference_yaw_ = current_yaw; // 現在のヨー角を新しい基準にする
                RCLCPP_INFO(this->get_logger(), "Switched to RIGHT turn mode, new reference: %.2f deg",
                            reference_yaw_ * 180.0 / M_PI);
            }
        }
        else // RIGHT
        {
            // 右旋回が-270°完了したら左旋回に切り替え
            if (angle_deg <= -270.0)
            {
                turn_mode_ = TurnMode::LEFT;
                reference_yaw_ = current_yaw; // 現在のヨー角を新しい基準にする
                RCLCPP_INFO(this->get_logger(), "Switched to LEFT turn mode, new reference: %.2f deg",
                            reference_yaw_ * 180.0 / M_PI);
            }
        }
    }

    void reset_state_()
    {
        is_initialized_ = false;
        reference_yaw_ = 0.0;
        turn_mode_ = TurnMode::LEFT; // 左旋回から再開
        RCLCPP_INFO(this->get_logger(), "State reset - ready for new eight turning cycle");
    }

    // メンバ変数
    TurnMode turn_mode_;
    float reference_yaw_;
    bool is_initialized_;
    bool is_eight_turning_mode_;
    float max_radius_;
    float min_radius_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_subscriber_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turn_radius_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_publisher_;
};

// コンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(ModeEightTurningAngle)
