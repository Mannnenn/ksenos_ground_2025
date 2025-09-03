#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ksenos_ground_msgs/msg/sbus_data.hpp>
#include <cmath>

class ModeEightTurningNode : public rclcpp::Node
{
public:
    ModeEightTurningNode() : Node("mode_eight_turning_node")
    {
        // パブリッシャーの初期化
        turn_radius_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "turn_radius", 10);

        // サブスクライバーの初期化
        sbus_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&ModeEightTurningNode::sbus_callback, this, std::placeholders::_1));

        flight_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "flight_distance", 10,
            std::bind(&ModeEightTurningNode::flight_distance_callback, this, std::placeholders::_1));

        // タイマーの初期化（10Hz）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ModeEightTurningNode::timer_callback, this));

        // パラメータの初期化
        is_eight_turning_mode_ = false;
        current_flight_distance_ = 0.0f;

        // 8の字軌道のパラメータ
        max_curvature_ = 1.0f / 4.1f; // 最大曲率 [1/m] (Pythonと同じ値)
        min_radius_ = 1000.0f;        // 曲率が0に近い時の最小半径 [m]
        cycle_length_ = 55.0f;        // 8の字1周期の距離 [m] (Pythonと同じ値)

        // 軌道セグメントの割合パラメータ
        straight_ratio_ = 0.00f;    // 直線部の割合（0-1、デフォルト0.00）
        transition_ratio_ = 0.125f; // 変化部の割合（0-1、デフォルト0.125）
        max_curve_ratio_ = 0.25f;   // 最大曲率部の割合（0-1、デフォルト0.25）

        RCLCPP_INFO(this->get_logger(), "Mode Eight Turning Node initialized");
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // autopilot_modeが"eight_turning"の場合のみ動作
        is_eight_turning_mode_ = (msg->autopilot_mode == "eight_turning");

        if (is_eight_turning_mode_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Eight turning mode detected. Publishing turn radius.");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Not in eight turning mode (current: %s).",
                         msg->autopilot_mode.c_str());
        }
    }

    void flight_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_flight_distance_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Flight distance updated: %.2f", current_flight_distance_);
    }

    void timer_callback()
    {
        if (!is_eight_turning_mode_)
        {
            return;
        }

        // 8の字軌道の曲率を計算
        float curvature = calculate_eight_trajectory_curvature(current_flight_distance_);

        // 旋回半径を計算（曲率の逆数）
        float turn_radius;
        if (std::abs(curvature) < 1e-6f) // 曲率が0に近い場合
        {
            turn_radius = min_radius_;
        }
        else
        {
            turn_radius = 1.0f / std::abs(curvature);
        }

        // 旋回半径をパブリッシュ
        auto msg = std_msgs::msg::Float32();
        msg.data = turn_radius;
        turn_radius_publisher_->publish(msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Distance: %.2f, Curvature: %.6f, Turn radius: %.2f",
                     current_flight_distance_, curvature, turn_radius);
    }

    float calculate_eight_trajectory_curvature(float distance)
    {
        // パラメータの正規化（合計が0.5を超えないよう調整）
        float straight_ratio = straight_ratio_;
        float transition_ratio = transition_ratio_;
        float max_curve_ratio = max_curve_ratio_;

        float total_ratio = straight_ratio + transition_ratio + max_curve_ratio;
        if (total_ratio > 0.5f)
        {
            // 0.5になるよう比例縮小
            float scale = 0.5f / total_ratio;
            straight_ratio *= scale;
            transition_ratio *= scale;
            max_curve_ratio *= scale;
        }

        // 8の字軌道の1周期内での位置を正規化（0-1）
        float normalized_pos = std::fmod(distance, cycle_length_) / cycle_length_;

        // 半周期ごとに処理（前半と後半）
        float half_cycle_pos = std::fmod(normalized_pos * 2.0f, 1.0f); // 0-1の範囲で半周期
        bool is_second_half = normalized_pos >= 0.5f;                  // 後半かどうか

        // セグメント境界を計算
        float transition1_end = transition_ratio * 2.0f;                 // 変化部1の終了位置
        float max_curve_end = transition1_end + max_curve_ratio * 2.0f;  // 最大曲率部の終了位置
        float transition2_end = max_curve_end + transition_ratio * 2.0f; // 変化部2の終了位置
        // float straight_end = transition2_end + straight_ratio * 2.0f;    // 直線部の終了位置

        float curvature = 0.0f;

        if (half_cycle_pos <= transition1_end)
        { // 変化部1（0から最大曲率へ）
            if (transition_ratio > 0)
            {
                float progress = half_cycle_pos / (transition_ratio * 2.0f);
                curvature = max_curvature_ * progress;
            }
        }
        else if (half_cycle_pos <= max_curve_end)
        { // 最大曲率部
            curvature = max_curvature_;
        }
        else if (half_cycle_pos <= transition2_end)
        { // 変化部2（最大曲率から0へ）
            if (transition_ratio > 0)
            {
                float progress = (half_cycle_pos - max_curve_end) / (transition_ratio * 2.0f);
                curvature = max_curvature_ * (1.0f - progress);
            }
        }
        else
        { // 直線部
            curvature = 0.0f;
        }

        // 後半は曲率の符号を反転
        if (is_second_half)
        {
            curvature = -curvature;
        }

        return curvature;
    }

    // メンバ変数
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turn_radius_publisher_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr flight_distance_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_eight_turning_mode_;
    float current_flight_distance_;

    // 8の字軌道パラメータ
    float max_curvature_;
    float min_radius_;
    float cycle_length_;

    // 軌道セグメントの割合パラメータ
    float straight_ratio_;
    float transition_ratio_;
    float max_curve_ratio_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeEightTurningNode>());
    rclcpp::shutdown();
    return 0;
}