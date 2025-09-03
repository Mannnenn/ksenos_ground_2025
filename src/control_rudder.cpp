#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ksenos_ground_msgs/msg/control_input.hpp>
#include <chrono>
#include <memory>
#include <cmath>

class RudderControl : public rclcpp::Node
{
public:
    RudderControl() : Node("rudder_control")
    {
        // パラメータの宣言
        this->declare_parameter<double>("kp", 0.5);                   // PI制御の比例ゲイン
        this->declare_parameter<double>("ki", 0.1);                   // PI制御の積分ゲイン
        this->declare_parameter<double>("ff_lat_acc_gain", 1.0);      // FF制御ゲイン
        this->declare_parameter<double>("ff_aileron_gain", 1.0);      // FF制御ゲイン
        this->declare_parameter<double>("max_rudder", 1.0);           // 最大ラダー値
        this->declare_parameter<double>("min_rudder", -1.0);          // 最小ラダー値
        this->declare_parameter<double>("max_integral", 10.0);        // 積分項の最大値
        this->declare_parameter<double>("lowpass_cutoff_freq", 10.0); // ローパスフィルターのカットオフ周波数 [Hz]

        // パラメータの取得
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        ff_lat_acc_gain_ = this->get_parameter("ff_lat_acc_gain").as_double();
        ff_aileron_gain_ = this->get_parameter("ff_aileron_gain").as_double();
        max_rudder_ = this->get_parameter("max_rudder").as_double();
        min_rudder_ = this->get_parameter("min_rudder").as_double();
        max_integral_ = this->get_parameter("max_integral").as_double();
        lowpass_cutoff_freq_ = this->get_parameter("lowpass_cutoff_freq").as_double();

        // 制御変数の初期化
        integral_error_ = 0.0;
        previous_time_ = this->now();

        // ローパスフィルター時刻の初期化
        previous_lat_acc_filter_time_ = this->now();
        previous_aileron_filter_time_ = this->now();
        previous_imu_filter_time_ = this->now();

        // ローパスフィルター変数の初期化
        filtered_reference_lateral_acceleration_ = 0.0;
        filtered_current_aileron_angle_ = 0.0;
        filtered_actual_lateral_acceleration_ = 0.0;

        // サブスクライバーの作成
        lateral_acceleration_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/controller/lat/calc/lateral_acceleration", 10,
            std::bind(&RudderControl::lateralAccelerationCallback, this, std::placeholders::_1));

        aileron_control_sub_ = this->create_subscription<ksenos_ground_msgs::msg::ControlInput>(
            "/aileron_input", 10,
            std::bind(&RudderControl::aileronControlCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensor/imu", 10,
            std::bind(&RudderControl::imuCallback, this, std::placeholders::_1));

        // パブリッシャーの作成
        control_output_pub_ = this->create_publisher<ksenos_ground_msgs::msg::ControlInput>(
            "/rudder_input", 10);

        // 制御ループタイマー
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RudderControl::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Rudder Control node initialized");
        RCLCPP_INFO(this->get_logger(), "Parameters - kp: %.3f, ki: %.3f, ff_lat_acc_gain: %.3f, ff_aileron_gain: %.3f, lowpass_cutoff: %.1f Hz",
                    kp_, ki_, ff_lat_acc_gain_, ff_aileron_gain_, lowpass_cutoff_freq_);
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
    void lateralAccelerationCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - previous_lat_acc_filter_time_).seconds();

        reference_lateral_acceleration_ = msg->data;
        filtered_reference_lateral_acceleration_ = applyLowpassFilter(
            reference_lateral_acceleration_, filtered_reference_lateral_acceleration_, dt);

        previous_lat_acc_filter_time_ = current_time;
        target_received_ = true;
    }

    void aileronControlCallback(const ksenos_ground_msgs::msg::ControlInput::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - previous_aileron_filter_time_).seconds();

        current_aileron_angle_ = msg->aileron;
        filtered_current_aileron_angle_ = applyLowpassFilter(
            current_aileron_angle_, filtered_current_aileron_angle_, dt);

        previous_aileron_filter_time_ = current_time;
        aileron_received_ = true;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - previous_imu_filter_time_).seconds();

        // IMUの線形加速度から横方向加速度を取得（y軸方向）
        actual_lateral_acceleration_ = msg->linear_acceleration.y;
        filtered_actual_lateral_acceleration_ = applyLowpassFilter(
            actual_lateral_acceleration_, filtered_actual_lateral_acceleration_, dt);

        previous_imu_filter_time_ = current_time;
        imu_received_ = true;
    }

    void controlLoop()
    {
        // 全てのデータが受信されているかチェック
        if (!target_received_ || !aileron_received_ || !imu_received_)
        {
            return;
        }

        // 時間の計算
        auto current_time = this->now();
        double dt = (current_time - previous_time_).seconds();
        if (dt <= 0.0)
            return;

        // FF制御の計算（フィルター済みの値を使用）
        double ff_output = ff_lat_acc_gain_ * filtered_reference_lateral_acceleration_ +
                           ff_aileron_gain_ * filtered_current_aileron_angle_;

        // エラーの計算（目標値 - 実際値、フィルター済みの値を使用）
        double error = target_lateral_acceleration_ - filtered_actual_lateral_acceleration_;

        // PI制御の計算
        // 積分項の更新（アンチワインドアップ付き）
        integral_error_ += error * dt;

        // 積分項の制限
        if (integral_error_ > max_integral_)
        {
            integral_error_ = max_integral_;
        }
        else if (integral_error_ < -max_integral_)
        {
            integral_error_ = -max_integral_;
        }

        // PI制御出力
        double pi_output = kp_ * error + ki_ * integral_error_;

        // FF + PI制御の総出力
        double total_output = ff_output + pi_output;

        // 出力制限
        if (total_output > max_rudder_)
        {
            total_output = max_rudder_;
        }
        else if (total_output < min_rudder_)
        {
            total_output = min_rudder_;
        }

        // 制御出力の発行
        auto control_msg = ksenos_ground_msgs::msg::ControlInput();
        control_msg.header.stamp = current_time;
        control_msg.header.frame_id = "rudder_control";
        control_msg.throttle = 0.0;        // ラダー制御では使用しない
        control_msg.elevator = 0.0;        // ラダー制御では使用しない
        control_msg.rudder = total_output; // ラダー出力
        control_msg.aileron = 0.0;         // 現在のエルロン角度を保持

        control_output_pub_->publish(control_msg);

        // 時間の更新
        previous_time_ = current_time;

        // デバッグログ（低頻度）
        static int log_counter = 0;
        if (++log_counter % 50 == 0)
        { // 1秒に1回
            RCLCPP_DEBUG(this->get_logger(),
                         "Target: %.3f, Actual: %.3f (filtered: %.3f), Error: %.3f, FF: %.3f, PI: %.3f, Total: %.3f",
                         target_lateral_acceleration_, actual_lateral_acceleration_,
                         filtered_actual_lateral_acceleration_, error, ff_output, pi_output, total_output);
        }
    }

    // サブスクライバー
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lateral_acceleration_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::ControlInput>::SharedPtr aileron_control_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // パブリッシャー
    rclcpp::Publisher<ksenos_ground_msgs::msg::ControlInput>::SharedPtr control_output_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 制御パラメータ
    double kp_;
    double ki_;
    double ff_lat_acc_gain_;
    double ff_aileron_gain_;
    double max_rudder_;
    double min_rudder_;
    double max_integral_;
    double lowpass_cutoff_freq_;

    // 制御変数
    const double target_lateral_acceleration_ = 0.0;
    double reference_lateral_acceleration_ = 0.0;
    double actual_lateral_acceleration_ = 0.0;
    double current_aileron_angle_ = 0.0;
    double integral_error_ = 0.0;
    rclcpp::Time previous_time_;

    // ローパスフィルター用時刻変数（各コールバック独立）
    rclcpp::Time previous_lat_acc_filter_time_;
    rclcpp::Time previous_aileron_filter_time_;
    rclcpp::Time previous_imu_filter_time_;

    // ローパスフィルター済み変数
    double filtered_reference_lateral_acceleration_;
    double filtered_actual_lateral_acceleration_;
    double filtered_current_aileron_angle_;

    // データ受信フラグ
    bool target_received_ = false;
    bool aileron_received_ = false;
    bool imu_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RudderControl>());
    rclcpp::shutdown();
    return 0;
}
