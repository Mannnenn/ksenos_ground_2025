#include <rclcpp/rclcpp.hpp>
#include <ksenos_ground_msgs/msg/plane_energy.hpp>
#include <ksenos_ground_msgs/msg/control_input.hpp>
#include <chrono>
#include <memory>
#include <algorithm>

class ThrottleControl : public rclcpp::Node
{
public:
    ThrottleControl() : Node("throttle_control")
    {
        // パラメータの宣言と取得
        this->declare_parameter("kp", 0.5);
        this->declare_parameter("ki", 0.1);
        this->declare_parameter("max_throttle", 1.0);
        this->declare_parameter("min_throttle", 0.0);
        this->declare_parameter("steady_throttle", 0.4);
        this->declare_parameter("max_integral", 0.5);

        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        max_throttle_ = this->get_parameter("max_throttle").as_double();
        min_throttle_ = this->get_parameter("min_throttle").as_double();
        steady_throttle_ = this->get_parameter("steady_throttle").as_double();
        max_integral_ = this->get_parameter("max_integral").as_double();

        // サブスクライバーの作成
        reference_energy_sub_ = this->create_subscription<ksenos_ground_msgs::msg::PlaneEnergy>(
            "/airplane/reference_energy", 10,
            std::bind(&ThrottleControl::reference_energy_callback, this, std::placeholders::_1));

        current_energy_sub_ = this->create_subscription<ksenos_ground_msgs::msg::PlaneEnergy>(
            "/airplane/current_energy", 10,
            std::bind(&ThrottleControl::current_energy_callback, this, std::placeholders::_1));

        // パブリッシャーの作成
        control_input_pub_ = this->create_publisher<ksenos_ground_msgs::msg::ControlInput>(
            "/throttle_input", 10);

        // 制御タイマーの作成
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20) // 20Hz
            ,
            std::bind(&ThrottleControl::control_loop, this));

        // 初期化
        reference_energy_ = 0.0;
        current_energy_ = 0.0;
        integral_error_ = 0.0;
        last_time_ = this->now();
        reference_received_ = false;
        current_received_ = false;

        RCLCPP_INFO(this->get_logger(), "Throttle control node initialized");
        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  kp: %.3f", kp_);
        RCLCPP_INFO(this->get_logger(), "  ki: %.3f", ki_);
        RCLCPP_INFO(this->get_logger(), "  max_throttle: %.3f", max_throttle_);
        RCLCPP_INFO(this->get_logger(), "  min_throttle: %.3f", min_throttle_);
        RCLCPP_INFO(this->get_logger(), "  steady_throttle: %.3f", steady_throttle_);
        RCLCPP_INFO(this->get_logger(), "  max_integral: %.3f", max_integral_);
    }

private:
    void reference_energy_callback(const ksenos_ground_msgs::msg::PlaneEnergy::SharedPtr msg)
    {
        reference_energy_ = msg->total_energy;
        reference_received_ = true;
    }

    void current_energy_callback(const ksenos_ground_msgs::msg::PlaneEnergy::SharedPtr msg)
    {
        current_energy_ = msg->total_energy;
        current_received_ = true;
    }

    void control_loop()
    {
        // データが受信されていない場合は制御しない
        if (!reference_received_ || !current_received_)
        {
            return;
        }

        // 時刻差dtの計算
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 初回呼び出し時は制御しない
        if (dt <= 0.0 || dt > 1.0)
        {
            return;
        }

        // エラーの計算（目標エネルギー - 現在エネルギー）
        double error = reference_energy_ - current_energy_;

        // 積分項の計算（アンチワインドアップ付き）
        integral_error_ += error * dt;
        integral_error_ = std::max(-max_integral_, std::min(max_integral_, integral_error_));

        // PI制御の計算
        double throttle_correction = kp_ * error + ki_ * integral_error_;

        // 定常スロットル量と制御量を合計
        double throttle_output = steady_throttle_ + throttle_correction;

        // 出力制限（0.0～1.0の範囲）
        throttle_output = std::max(min_throttle_, std::min(max_throttle_, throttle_output));

        // ControlInputメッセージの作成とパブリッシュ
        auto control_msg = ksenos_ground_msgs::msg::ControlInput();
        control_msg.header.stamp = current_time;
        control_msg.header.frame_id = "base_link";
        control_msg.throttle = static_cast<float>(throttle_output);
        // 他の制御入力は0に設定
        control_msg.aileron = 0.0;
        control_msg.elevator = 0.0;
        control_msg.rudder = 0.0;

        control_input_pub_->publish(control_msg);

        // デバッグ情報の出力
        RCLCPP_DEBUG(this->get_logger(),
                     "Error: %.3f, Integral: %.3f, Throttle output: %.3f (correction: %.3f)",
                     error, integral_error_, throttle_output, throttle_correction);
    }

    // サブスクライバー
    rclcpp::Subscription<ksenos_ground_msgs::msg::PlaneEnergy>::SharedPtr reference_energy_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::PlaneEnergy>::SharedPtr current_energy_sub_;

    // パブリッシャー
    rclcpp::Publisher<ksenos_ground_msgs::msg::ControlInput>::SharedPtr control_input_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 制御パラメータ
    double kp_;              // P制御ゲイン
    double ki_;              // I制御ゲイン
    double max_throttle_;    // 最大throttle値
    double min_throttle_;    // 最小throttle値
    double steady_throttle_; // 定常スロットル量
    double max_integral_;    // 積分項の最大値（アンチワインドアップ）

    // 状態変数
    double reference_energy_;
    double current_energy_;
    double integral_error_;
    rclcpp::Time last_time_;

    // データ受信フラグ
    bool reference_received_;
    bool current_received_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrottleControl>());
    rclcpp::shutdown();
    return 0;
}