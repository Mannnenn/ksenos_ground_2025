#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ksenos_ground_msgs/msg/flow_rate_data.hpp>
#include <memory>

class LateralAccelerationCalculator : public rclcpp::Node
{
public:
    LateralAccelerationCalculator() : Node("lateral_acceleration_calculator")
    {
        // パラメータの宣言と初期化
        this->declare_parameter("low_pass_filter_alpha", 0.1);
        low_pass_filter_alpha_ = this->get_parameter("low_pass_filter_alpha").as_double();

        // パブリッシャーの初期化
        lateral_acc_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "lateral_acceleration", 10);

        // サブスクライバーの初期化
        flow_rate_sub_ = this->create_subscription<ksenos_ground_msgs::msg::FlowRateData>(
            "flow_rate_data", 10,
            std::bind(&LateralAccelerationCalculator::flowRateCallback, this, std::placeholders::_1));

        turning_radius_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "turning_radius", 10,
            std::bind(&LateralAccelerationCalculator::turningRadiusCallback, this, std::placeholders::_1));

        // 初期値の設定
        filtered_velocity_ = 0.0f;
        current_turning_radius_ = 0.0f;
        is_first_flow_rate_ = true;

        RCLCPP_INFO(this->get_logger(), "Lateral Acceleration Calculator node started");
        RCLCPP_INFO(this->get_logger(), "Low-pass filter alpha: %f", low_pass_filter_alpha_);
    }

private:
    void flowRateCallback(const ksenos_ground_msgs::msg::FlowRateData::SharedPtr msg)
    {
        // ローパスフィルターを適用して速度を算出
        if (is_first_flow_rate_)
        {
            filtered_velocity_ = msg->flow_rate;
            is_first_flow_rate_ = false;
        }
        else
        {
            // ローパスフィルター: y[n] = α * x[n] + (1-α) * y[n-1]
            filtered_velocity_ = low_pass_filter_alpha_ * msg->flow_rate +
                                 (1.0 - low_pass_filter_alpha_) * filtered_velocity_;
        }

        // 横加速度を計算して発行
        calculateAndPublishLateralAcceleration();
    }

    void turningRadiusCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_turning_radius_ = msg->data;

        // 横加速度を計算して発行
        calculateAndPublishLateralAcceleration();
    }

    void calculateAndPublishLateralAcceleration()
    {
        // 旋回半径が0または非常に小さい場合は計算しない
        if (std::abs(current_turning_radius_) < 1e-6)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Turning radius is too small or zero. Cannot calculate lateral acceleration.");
            return;
        }

        // 横加速度の計算: a_lat = v^2 / r
        float lateral_acceleration = (filtered_velocity_ * filtered_velocity_) / current_turning_radius_;

        // メッセージの作成と発行
        auto lat_acc_msg = std_msgs::msg::Float32();
        lat_acc_msg.data = lateral_acceleration;
        lateral_acc_pub_->publish(lat_acc_msg);

        // デバッグ情報の出力
        RCLCPP_DEBUG(this->get_logger(),
                     "Filtered velocity: %.3f m/s, Turning radius: %.3f m, Lateral acceleration: %.3f m/s²",
                     filtered_velocity_, current_turning_radius_, lateral_acceleration);
    }

    // メンバ変数
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lateral_acc_pub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr flow_rate_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr turning_radius_sub_;

    double low_pass_filter_alpha_; // ローパスフィルターのアルファ値
    float filtered_velocity_;      // フィルタリングされた速度
    float current_turning_radius_; // 現在の旋回半径
    bool is_first_flow_rate_;      // 最初のflow_rateデータかどうか
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LateralAccelerationCalculator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}