#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ksenos_ground_msgs/msg/rpy.hpp>
#include <ksenos_ground_msgs/msg/plane_energy.hpp>
#include <ksenos_ground_msgs/msg/control_input.hpp>
#include <chrono>
#include <cmath>

class ElevatorControl : public rclcpp::Node
{
public:
    ElevatorControl() : Node("elevator_control")
    {
        // パラメータの宣言
        this->declare_parameter("k_energy_gain", 1.0);         // エネルギー制御ゲイン
        this->declare_parameter("kd_pitch_angle", 0.1);        // ピッチ角D制御ゲイン
        this->declare_parameter("kd_pitch", 0.1);              // ピッチ角速度D制御ゲイン
        this->declare_parameter("max_elevator", 0.6);          // 最大エレベータ値
        this->declare_parameter("min_elevator", -0.6);         // 最小エレベータ値
        this->declare_parameter("balanced_flight_pitch", 0.0); // バランス飛行時のピッチ角
        this->declare_parameter("lowpass_cutoff_freq", 10.0);  // ローパスフィルターのカットオフ周波数 [Hz]

        // パラメータの取得
        k_energy_gain_ = this->get_parameter("k_energy_gain").as_double();
        kd_pitch_angle_ = this->get_parameter("kd_pitch_angle").as_double();
        kd_pitch_rate_ = this->get_parameter("kd_pitch").as_double();
        max_elevator_ = this->get_parameter("max_elevator").as_double();
        min_elevator_ = this->get_parameter("min_elevator").as_double();
        balanced_flight_pitch_ = this->get_parameter("balanced_flight_pitch").as_double();
        lowpass_cutoff_freq_ = this->get_parameter("lowpass_cutoff_freq").as_double();

        // エネルギー関連のサブスクライバー
        current_energy_sub_ = this->create_subscription<ksenos_ground_msgs::msg::PlaneEnergy>(
            "/airplane/current_energy", 10,
            std::bind(&ElevatorControl::current_energy_callback, this, std::placeholders::_1));

        reference_energy_sub_ = this->create_subscription<ksenos_ground_msgs::msg::PlaneEnergy>(
            "/airplane/reference_energy", 10,
            std::bind(&ElevatorControl::reference_energy_callback, this, std::placeholders::_1));

        // RPYサブスクライバー（ピッチ角度用）
        rpy_sub_ = this->create_subscription<ksenos_ground_msgs::msg::Rpy>(
            "/rpy", 10,
            std::bind(&ElevatorControl::rpy_callback, this, std::placeholders::_1));

        // IMUサブスクライバー（ピッチ角速度用）
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensor/imu", 10,
            std::bind(&ElevatorControl::imu_callback, this, std::placeholders::_1));

        // パブリッシャーの作成
        control_input_pub_ = this->create_publisher<ksenos_ground_msgs::msg::ControlInput>(
            "/elevator_input", 10);

        // 制御ループのタイマー（20Hz）
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ElevatorControl::control_loop, this));

        // 変数の初期化
        current_kinetic_energy_ = 0.0;
        current_potential_energy_ = 0.0;
        reference_kinetic_energy_ = 0.0;
        reference_potential_energy_ = 0.0;
        current_pitch_ = 0.0;
        pitch_rate_ = 0.0;

        // ローパスフィルター変数の初期化
        filtered_current_kinetic_energy_ = 0.0;
        filtered_current_potential_energy_ = 0.0;
        filtered_reference_kinetic_energy_ = 0.0;
        filtered_reference_potential_energy_ = 0.0;
        filtered_current_pitch_ = 0.0;
        filtered_pitch_rate_ = 0.0;

        // ローパスフィルター時刻の初期化
        previous_current_energy_filter_time_ = this->now();
        previous_reference_energy_filter_time_ = this->now();
        previous_rpy_filter_time_ = this->now();
        previous_imu_filter_time_ = this->now();

        // データ受信フラグの初期化
        current_energy_received_ = false;
        reference_energy_received_ = false;
        rpy_received_ = false;
        imu_received_ = false;

        RCLCPP_INFO(this->get_logger(), "Elevator Control Node Started");
        RCLCPP_INFO(this->get_logger(), "Energy Gain: %.3f, Pitch D Gain: %.3f, Lowpass Cutoff: %.1f Hz",
                    k_energy_gain_, kd_pitch_rate_, lowpass_cutoff_freq_);
    }

private:
    // ローパスフィルターの実装（1次ローパスフィルター）
    double applyLowpassFilter(double input, double &filtered_value, double dt)
    {
        if (dt <= 0.0)
            return filtered_value; // dtが無効な場合は前の値を返す

        double alpha = dt / (dt + 1.0 / (2.0 * M_PI * lowpass_cutoff_freq_));
        filtered_value = alpha * input + (1.0 - alpha) * filtered_value;
        return filtered_value;
    }
    void current_energy_callback(const ksenos_ground_msgs::msg::PlaneEnergy::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - previous_current_energy_filter_time_).seconds();

        current_kinetic_energy_ = msg->kinetic_energy;
        current_potential_energy_ = msg->potential_energy;

        filtered_current_kinetic_energy_ = applyLowpassFilter(
            current_kinetic_energy_, filtered_current_kinetic_energy_, dt);
        filtered_current_potential_energy_ = applyLowpassFilter(
            current_potential_energy_, filtered_current_potential_energy_, dt);

        previous_current_energy_filter_time_ = current_time;
        current_energy_received_ = true;
    }

    void reference_energy_callback(const ksenos_ground_msgs::msg::PlaneEnergy::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - previous_reference_energy_filter_time_).seconds();

        reference_kinetic_energy_ = msg->kinetic_energy;
        reference_potential_energy_ = msg->potential_energy;

        filtered_reference_kinetic_energy_ = applyLowpassFilter(
            reference_kinetic_energy_, filtered_reference_kinetic_energy_, dt);
        filtered_reference_potential_energy_ = applyLowpassFilter(
            reference_potential_energy_, filtered_reference_potential_energy_, dt);

        previous_reference_energy_filter_time_ = current_time;
        reference_energy_received_ = true;
    }

    void rpy_callback(const ksenos_ground_msgs::msg::Rpy::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - previous_rpy_filter_time_).seconds();

        current_pitch_ = msg->pitch;
        filtered_current_pitch_ = applyLowpassFilter(
            current_pitch_, filtered_current_pitch_, dt);

        previous_rpy_filter_time_ = current_time;
        rpy_received_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - previous_imu_filter_time_).seconds();

        // IMUのピッチ角速度を取得（通常はangular_velocity.y）
        pitch_rate_ = msg->angular_velocity.y;
        filtered_pitch_rate_ = applyLowpassFilter(
            pitch_rate_, filtered_pitch_rate_, dt);

        previous_imu_filter_time_ = current_time;
        imu_received_ = true;
    }

    void control_loop()
    {
        // 必要なデータが全て受信されているかチェック
        if (!current_energy_received_ || !reference_energy_received_ ||
            !rpy_received_ || !imu_received_)
        {
            return;
        }

        // エネルギー誤差の計算（フィルター済みの値を使用）
        // K_err = K_ref - K_cur
        double kinetic_error = filtered_reference_kinetic_energy_ - filtered_current_kinetic_energy_;

        // P_err = P_ref - P_cur
        double potential_error = filtered_reference_potential_energy_ - filtered_current_potential_energy_;

        // エレベータ操作量の基本計算: K_err - P_err
        // 運動エネルギーが不足している場合（K_err > 0）は機首を下げる（負のエレベータ）
        // 位置エネルギーが不足している場合（P_err > 0）は機首を上げる（正のエレベータ）
        double elevator_base = k_energy_gain_ * (kinetic_error - potential_error);

        // D制御：フィルター済みピッチ角・角速度を使用して振動を抑制（負の符号で安定化）
        double pitch_error = balanced_flight_pitch_ - filtered_current_pitch_;
        double d_term = kd_pitch_angle_ * pitch_error - kd_pitch_rate_ * filtered_pitch_rate_;

        // 最終的なエレベータ操作量
        double elevator_output = elevator_base + d_term;

        // 出力の制限
        elevator_output = std::max(min_elevator_, std::min(max_elevator_, elevator_output));

        // ControlInputメッセージの作成とパブリッシュ
        auto control_msg = ksenos_ground_msgs::msg::ControlInput();
        control_msg.header.stamp = this->now();
        control_msg.header.frame_id = "base_link";
        control_msg.elevator = static_cast<float>(elevator_output);
        // 他の制御入力は0に設定
        control_msg.throttle = 0.0;
        control_msg.aileron = 0.0;
        control_msg.rudder = 0.0;

        control_input_pub_->publish(control_msg);

        // デバッグ情報の出力
        RCLCPP_DEBUG(this->get_logger(),
                     "K_err: %.3f, P_err: %.3f, Base: %.3f, Pitch Rate: %.3f (filtered: %.3f), D_term: %.3f, Elevator: %.3f",
                     kinetic_error, potential_error, elevator_base, pitch_rate_, filtered_pitch_rate_, d_term, elevator_output);
    }

    // エネルギー関連のサブスクライバー
    rclcpp::Subscription<ksenos_ground_msgs::msg::PlaneEnergy>::SharedPtr current_energy_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::PlaneEnergy>::SharedPtr reference_energy_sub_;

    // RPY・IMUサブスクライバー
    rclcpp::Subscription<ksenos_ground_msgs::msg::Rpy>::SharedPtr rpy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // パブリッシャー
    rclcpp::Publisher<ksenos_ground_msgs::msg::ControlInput>::SharedPtr control_input_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 制御パラメータ
    double k_energy_gain_;         // エネルギー制御ゲイン
    double kd_pitch_angle_;        // ピッチ角度D制御ゲイン
    double kd_pitch_rate_;         // ピッチ角速度D制御ゲイン
    double max_elevator_;          // 最大エレベータ値
    double min_elevator_;          // 最小エレベータ値
    double balanced_flight_pitch_; // バランス飛行時のピッチ角
    double lowpass_cutoff_freq_;   // ローパスフィルターのカットオフ周波数

    // エネルギー状態変数
    float current_kinetic_energy_;
    float current_potential_energy_;
    float reference_kinetic_energy_;
    float reference_potential_energy_;

    // 姿勢状態変数
    float current_pitch_;
    float pitch_rate_;

    // ローパスフィルター済み変数
    double filtered_current_kinetic_energy_;
    double filtered_current_potential_energy_;
    double filtered_reference_kinetic_energy_;
    double filtered_reference_potential_energy_;
    double filtered_current_pitch_;
    double filtered_pitch_rate_;

    // ローパスフィルター用時刻変数（各コールバック独立）
    rclcpp::Time previous_current_energy_filter_time_;
    rclcpp::Time previous_reference_energy_filter_time_;
    rclcpp::Time previous_rpy_filter_time_;
    rclcpp::Time previous_imu_filter_time_;

    // データ受信フラグ
    bool current_energy_received_;
    bool reference_energy_received_;
    bool rpy_received_;
    bool imu_received_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ElevatorControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
