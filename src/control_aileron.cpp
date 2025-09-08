#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ksenos_ground_msgs/msg/rpy.hpp>
#include <ksenos_ground_msgs/msg/control_input.hpp>
#include <chrono>

class AileronControl : public rclcpp::Node
{
public:
    AileronControl() : Node("aileron_control")
    {
        // パラメータの宣言
        this->declare_parameter("kp", 1.0);
        this->declare_parameter("kd", 0.1);
        this->declare_parameter("max_aileron", 0.6);
        this->declare_parameter("min_aileron", -0.6);
        // 左右で異なるPゲインを適用するためのスケール（1.0で従来と同等）
        this->declare_parameter("kp_right_scale", 1.0);
        this->declare_parameter("kp_left_scale", 1.0);

        // パラメータの取得
        kp_ = this->get_parameter("kp").as_double();
        kd_ = this->get_parameter("kd").as_double();
        max_aileron_ = this->get_parameter("max_aileron").as_double();
        min_aileron_ = this->get_parameter("min_aileron").as_double();
        kp_right_scale_ = this->get_parameter("kp_right_scale").as_double();
        kp_left_scale_ = this->get_parameter("kp_left_scale").as_double();

        // サブスクライバーの作成
        target_roll_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/controller/lat/calc/target_roll_angle", 10,
            std::bind(&AileronControl::target_roll_callback, this, std::placeholders::_1));

        rpy_sub_ = this->create_subscription<ksenos_ground_msgs::msg::Rpy>(
            "/rpy", 10,
            std::bind(&AileronControl::rpy_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensor/imu", 10,
            std::bind(&AileronControl::imu_callback, this, std::placeholders::_1));

        // パブリッシャーの作成
        control_input_pub_ = this->create_publisher<ksenos_ground_msgs::msg::ControlInput>(
            "/aileron_input", 10);

        // 制御ループのタイマー（50Hz）
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&AileronControl::control_loop, this));

        // 変数の初期化
        target_roll_ = 0.0;
        current_roll_ = 0.0;
        roll_rate_ = 0.0;
        previous_error_ = 0.0;
        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Aileron Control Node Started");
        RCLCPP_INFO(this->get_logger(), "Kp: %.3f (right_scale: %.3f, left_scale: %.3f), Kd: %.3f",
                    kp_, kp_right_scale_, kp_left_scale_, kd_);
    }

private:
    void target_roll_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_roll_ = msg->data;
    }

    void rpy_callback(const ksenos_ground_msgs::msg::Rpy::SharedPtr msg)
    {
        current_roll_ = msg->roll;
        rpy_received_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // IMUのroll角速度を取得（通常はangular_velocity.x）
        roll_rate_ = msg->angular_velocity.x;
        imu_received_ = true;
    }

    void control_loop()
    {
        // データが受信されているかチェック
        if (!rpy_received_ || !imu_received_)
        {
            return;
        }

        // 現在時刻を取得
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        if (dt <= 0.0)
        {
            return;
        }

        // P制御：目標値と現在値の差分
        double error = target_roll_ - current_roll_;

        // D制御：角速度を使用（負の符号で安定化）
        double d_term = -kd_ * roll_rate_;

        // 方向別Pゲイン（右ロール/左ロールで効きを調整）
        // errorの符号で方向を判定（error<0 を "右ロール方向" と仮定）。
        // 誤差符号の定義は機体の座標系に依存するため、必要ならスケール値で微調整してください。
        const double kp_eff = kp_ * ((error <= 0.0) ? kp_right_scale_ : kp_left_scale_);

        // PD制御の計算
        double aileron_output = kp_eff * error + d_term;

        // 出力の制限
        aileron_output = std::max(min_aileron_, std::min(max_aileron_, aileron_output));

        // ControlInputメッセージの作成とパブリッシュ
        auto control_msg = ksenos_ground_msgs::msg::ControlInput();
        control_msg.header.stamp = current_time;
        control_msg.header.frame_id = "base_link";
        control_msg.aileron = static_cast<float>(aileron_output);
        // 他の制御入力は0に設定（必要に応じて変更）
        control_msg.throttle = 0.0;
        control_msg.elevator = 0.0;
        control_msg.rudder = 0.0;

        control_input_pub_->publish(control_msg);

        // デバッグ情報の出力
        // RCLCPP_DEBUG(this->get_logger(),
        //              "Target: %.3f, Current: %.3f, Error: %.3f, Roll Rate: %.3f, Aileron: %.3f",
        //              target_roll_, current_roll_, error, roll_rate_, aileron_output);

        // 時刻の更新
        last_time_ = current_time;
        previous_error_ = error;
    }

    // サブスクライバー
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_roll_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::Rpy>::SharedPtr rpy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // パブリッシャー
    rclcpp::Publisher<ksenos_ground_msgs::msg::ControlInput>::SharedPtr control_input_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 制御パラメータ
    double kp_;             // P制御ゲイン
    double kd_;             // D制御ゲイン
    double max_aileron_;    // 最大aileron値
    double min_aileron_;    // 最小aileron値
    double kp_right_scale_; // 右ロール方向用のPゲインスケール
    double kp_left_scale_;  // 左ロール方向用のPゲインスケール

    // 状態変数
    double target_roll_;
    double current_roll_;
    double roll_rate_;
    double previous_error_;
    rclcpp::Time last_time_;

    // データ受信フラグ
    bool rpy_received_ = false;
    bool imu_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AileronControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}