#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ksenos_ground_msgs/msg/sbus_data.hpp"
// for parameter callback result
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class ModeRiseTurning : public rclcpp::Node
{
public:
    ModeRiseTurning() : Node("mode_rise_turning")
    {
        // パラメータ宣言（周回判定用）
        this->declare_parameter<float>("yaw_per_lap", 6.2831853f); // 1周=2πラジアン
        this->declare_parameter<float>("altitude_offset", 3.5f);
        this->declare_parameter<int>("lap_count_initial", 3);
        this->declare_parameter<int>("lap_count_transition", 2);
        // 旋回半径パラメータ（検証条件に合わせて正の初期値）
        this->declare_parameter<float>("target_radius", 5.0f);

        // パラメータコールバック
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ModeRiseTurning::parameter_callback, this, std::placeholders::_1));

        // サブスクライバー
        sbus_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&ModeRiseTurning::sbus_callback, this, std::placeholders::_1));
        serialized_yaw_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "serialized_yaw", 10,
            std::bind(&ModeRiseTurning::yaw_callback, this, std::placeholders::_1));
        average_altitude_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "average_altitude", 10,
            std::bind(&ModeRiseTurning::altitude_callback, this, std::placeholders::_1));

        // パブリッシャー
        target_altitude_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "target_altitude_rise_turning", 10);
        target_radius_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "target_turning_radius", 10);

        // 初期化
        is_rise_turning_mode_ = false;
        lap_count_ = 0;
        last_yaw_ = 0.0f;
        total_yaw_ = 0.0f;
        average_altitude_ = 0.0f;
        target_altitude_ = 0.0f;
        transition_start_lap_ = 0;
        transition_end_lap_ = 0;
        received_yaw_ = false;
        received_altitude_ = false;

        // 初期値取得
        target_radius_ = this->get_parameter("target_radius").as_double();

        // パラメータ取得
        yaw_per_lap_ = this->get_parameter("yaw_per_lap").as_double();
        altitude_offset_ = this->get_parameter("altitude_offset").as_double();
        lap_count_initial_ = this->get_parameter("lap_count_initial").as_int();
        lap_count_transition_ = this->get_parameter("lap_count_transition").as_int();
        transition_start_lap_ = lap_count_initial_;
        transition_end_lap_ = lap_count_initial_ + lap_count_transition_;

        RCLCPP_INFO(this->get_logger(), "Mode Rise Turning node initialized");
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // 新しいモード状態を取得
        bool new_mode = (msg->autopilot_mode == "rise_turning");

        // モード離脱のエッジでのみリセット（毎回の不要リセットを防止）
        if (is_rise_turning_mode_ && !new_mode)
        {
            reset_state_();
            RCLCPP_DEBUG(this->get_logger(), "RiseTurning mode exited; state reset");
        }

        // 現在のモード状態を更新
        is_rise_turning_mode_ = new_mode;

        if (is_rise_turning_mode_)
        {
            // 現在のパラメータ値を取得してパブリッシュ
            target_radius_ = this->get_parameter("target_radius").as_double();
            std_msgs::msg::Float32 radius_msg;
            radius_msg.data = target_radius_;
            target_radius_publisher_->publish(radius_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published target turning radius: %.2f meters", target_radius_);
        }
    }

    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!is_rise_turning_mode_)
            return;

        float yaw = msg->data;
        if (!received_yaw_)
        {
            last_yaw_ = yaw;
            received_yaw_ = true;
            return;
        }
        // 周回判定
        float delta_yaw = yaw - last_yaw_;
        // -π〜πの範囲でラップ
        if (delta_yaw > 3.1415926f)
            delta_yaw -= 2 * 3.1415926f;
        if (delta_yaw < -3.1415926f)
            delta_yaw += 2 * 3.1415926f;
        total_yaw_ += std::abs(delta_yaw);
        last_yaw_ = yaw;
        // 周回数更新
        int new_lap_count = static_cast<int>(total_yaw_ / yaw_per_lap_);
        if (new_lap_count > lap_count_)
        {
            lap_count_ = new_lap_count;
            RCLCPP_INFO(this->get_logger(), "Lap count updated: %d", lap_count_);
        }
        publish_target_altitude();
    }

    void altitude_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        average_altitude_ = msg->data;
        received_altitude_ = true;
    }

    void publish_target_altitude()
    {
        if (!is_rise_turning_mode_ || !received_yaw_ || !received_altitude_)
            return;

        // 連続的な周回数を計算
        float continuous_lap = total_yaw_ / yaw_per_lap_;
        float target = average_altitude_;

        if (continuous_lap < transition_start_lap_)
        {
            // 最初の3周はそのまま
            target = average_altitude_;
        }
        else if (continuous_lap < transition_end_lap_)
        {
            // 2周かけて+3.5mへスムーズに遷移（2階微分連続）
            float t = (continuous_lap - transition_start_lap_) / lap_count_transition_;
            // 2階微分連続な補間: 3t^2 - 2t^3 (Hermite)
            float smooth = 3 * t * t - 2 * t * t * t;
            target = average_altitude_ + altitude_offset_ * smooth;
        }
        else
        {
            // 以降は+3.5m
            target = average_altitude_ + altitude_offset_;
        }
        target_altitude_ = target;
        auto msg = std_msgs::msg::Float32();
        msg.data = target_altitude_;
        target_altitude_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published target altitude: %.2f (lap: %d)", target_altitude_, lap_count_);
    }

    // モード離脱時の状態リセットを一箇所に集約
    void reset_state_()
    {
        received_yaw_ = false;
        total_yaw_ = 0.0f;
        lap_count_ = 0;
        last_yaw_ = 0.0f;
        average_altitude_ = 0.0f;
        received_altitude_ = false; // 古い高度でのpublishを防止
        received_yaw_ = false;
    }

    // パラメータ更新時の検証
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "target_radius")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    float new_radius = param.as_double();
                    if (new_radius > 0.0f)
                    {
                        target_radius_ = new_radius;
                        RCLCPP_INFO(this->get_logger(), "Target radius updated to: %.2f meters", target_radius_);
                    }
                    else
                    {
                        result.successful = false;
                        result.reason = "Target radius must be greater than 0";
                        RCLCPP_WARN(this->get_logger(), "Invalid target radius: %.2f. Must be > 0", new_radius);
                    }
                }
                else
                {
                    result.successful = false;
                    result.reason = "Target radius parameter must be a float/double";
                }
            }
        }
        return result;
    }

    // メンバ変数
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_altitude_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_radius_publisher_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr serialized_yaw_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr average_altitude_subscriber_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    bool is_rise_turning_mode_;
    int lap_count_;
    float last_yaw_;
    float total_yaw_;
    float average_altitude_;
    float target_altitude_;
    float target_radius_;
    int transition_start_lap_;
    int transition_end_lap_;
    float yaw_per_lap_;
    float altitude_offset_;
    int lap_count_initial_;
    int lap_count_transition_;
    bool received_yaw_;
    bool received_altitude_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeRiseTurning>());
    rclcpp::shutdown();
    return 0;
}
