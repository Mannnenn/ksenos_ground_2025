#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ksenos_ground_msgs/msg/sbus_data.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <vector>

class SbusCalibration : public rclcpp::Node
{
public:
    SbusCalibration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("sbus_calibration_node", options)
    {
        // パラメータ宣言
        this->declare_parameter("calibration_duration", 1.0);
        this->declare_parameter("input_topic", "sbus_raw");
        this->declare_parameter("offset_topic", "sbus_offset_values");

        // パラメータ取得
        calibration_duration_ = this->get_parameter("calibration_duration").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string offset_topic = this->get_parameter("offset_topic").as_string();

        // 初期化
        is_calibrating_ = true;
        calibration_samples_ = 0;
        aileron_r_sum_ = 0.0;
        elevator_sum_ = 0.0;
        rudder_sum_ = 0.0;
        aileron_l_sum_ = 0.0;

        aileron_r_offset_ = 0.0;
        elevator_offset_ = 0.0;
        rudder_offset_ = 0.0;
        aileron_l_offset_ = 0.0;

        // 開始時刻を記録
        start_time_ = this->now();

        // Subscriber and Publisher
        sbus_sub_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            input_topic, 10,
            std::bind(&SbusCalibration::sbus_callback, this, std::placeholders::_1));

        // オフセット値をパブリッシュするためのパブリッシャー
        // Vector3を使用: x=aileron_r, y=elevator, z=rudder
        // 別途aileron_lを送るためのカスタムメッセージか、追加のトピックが必要
        offset_pub_ = this->create_publisher<ksenos_ground_msgs::msg::SbusData>(
            offset_topic, 10);

        // タイマーでオフセット値を継続的にパブリッシュ
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SbusCalibration::publish_offset, this));

        RCLCPP_INFO(this->get_logger(), "SBUS Calibration Node started. Calibrating for %.1f seconds...", calibration_duration_);
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds();

        if (is_calibrating_)
        {
            if (elapsed_time < calibration_duration_)
            {
                // キャリブレーション中：平均値を計算するためのデータを蓄積
                aileron_r_sum_ += msg->aileron_r;
                elevator_sum_ += msg->elevator;
                rudder_sum_ += msg->rudder;
                aileron_l_sum_ += msg->aileron_l;
                calibration_samples_++;

                // 進捗表示（1秒ごと）
                if (calibration_samples_ % 50 == 0) // 約50Hzの場合、1秒ごと
                {
                    RCLCPP_INFO(this->get_logger(), "Calibrating... %.1fs remaining",
                                calibration_duration_ - elapsed_time);
                }
            }
            else
            {
                // キャリブレーション完了：オフセット値を計算
                if (calibration_samples_ > 0)
                {
                    aileron_r_offset_ = aileron_r_sum_ / calibration_samples_;
                    elevator_offset_ = elevator_sum_ / calibration_samples_;
                    rudder_offset_ = rudder_sum_ / calibration_samples_;
                    aileron_l_offset_ = aileron_l_sum_ / calibration_samples_;

                    RCLCPP_INFO(this->get_logger(),
                                "Calibration completed! Offsets calculated from %d samples:",
                                calibration_samples_);
                    RCLCPP_INFO(this->get_logger(),
                                "  aileron_r: %.4f, elevator: %.4f, rudder: %.4f, aileron_l: %.4f",
                                aileron_r_offset_, elevator_offset_, rudder_offset_, aileron_l_offset_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                                "No samples collected during calibration. Using zero offsets.");
                }

                is_calibrating_ = false;
            }
        }
    }

    void publish_offset()
    {
        // キャリブレーション完了後、オフセット値を継続的にパブリッシュ
        if (!is_calibrating_)
        {
            auto offset_msg = std::make_shared<ksenos_ground_msgs::msg::SbusData>();

            // オフセット値をSbusDataメッセージとして送信
            offset_msg->aileron_r = aileron_r_offset_;
            offset_msg->elevator = elevator_offset_;
            offset_msg->rudder = rudder_offset_;
            offset_msg->aileron_l = aileron_l_offset_;

            // タイムスタンプを設定
            offset_msg->header.stamp = this->now();

            offset_pub_->publish(*offset_msg);
        }
    }

    // Subscriber and Publisher
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_sub_;
    rclcpp::Publisher<ksenos_ground_msgs::msg::SbusData>::SharedPtr offset_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // キャリブレーション関連
    bool is_calibrating_;
    double calibration_duration_;
    rclcpp::Time start_time_;
    int calibration_samples_;

    // 平均値計算用の累積値
    double aileron_r_sum_;
    double elevator_sum_;
    double rudder_sum_;
    double aileron_l_sum_;

    // オフセット値
    double aileron_r_offset_;
    double elevator_offset_;
    double rudder_offset_;
    double aileron_l_offset_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(SbusCalibration)