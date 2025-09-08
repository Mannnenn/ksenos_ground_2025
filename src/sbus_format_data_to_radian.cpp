#include <rclcpp/rclcpp.hpp>
#include <ksenos_ground_msgs/msg/sbus_raw_data.hpp>
#include <ksenos_ground_msgs/msg/sbus_data.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>

class SbusDataProcessor : public rclcpp::Node
{
public:
    SbusDataProcessor() : Node("sbus_data_processor")
    {
        // パブリッシャーの作成
        publisher_ = this->create_publisher<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10);

        // サブスクライバーの作成
        subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusRawData>(
            "sbus_raw_data", 10,
            std::bind(&SbusDataProcessor::sbusRawDataCallback, this, std::placeholders::_1));

        // パラメータの宣言と初期化
        this->declare_parameter("servo_min_count", 352);
        this->declare_parameter("servo_max_count", 1696);
        this->declare_parameter("servo_min_rad", -0.5236); // -30度をラジアンで
        this->declare_parameter("servo_max_rad", 0.5236);  // 30度をラジアンで

        this->declare_parameter("throttle_min_count", 352);
        this->declare_parameter("throttle_max_count", 1696);
        this->declare_parameter("throttle_min_openness", 0.0); // 開度の最小値
        this->declare_parameter("throttle_max_openness", 1.0); // 開度の最大値

        this->declare_parameter("autopilot_threshold", 1000);
        this->declare_parameter("autolanding_threshold", 1000);

        this->declare_parameter("dropping_device_threshold_low", 500);
        this->declare_parameter("dropping_device_threshold_high", 1500);

        this->declare_parameter("autopilot_mode_threshold_low", 500);
        this->declare_parameter("autopilot_mode_threshold_high", 1500);

        RCLCPP_INFO(this->get_logger(), "SBUS Data Processor ノードが開始されました");
    }

private:
    void sbusRawDataCallback(const ksenos_ground_msgs::msg::SbusRawData::SharedPtr msg)
    {
        auto processed_msg = ksenos_ground_msgs::msg::SbusData();

        // ヘッダーをコピー
        processed_msg.header = msg->header;

        // パラメータの取得
        int servo_min_count = this->get_parameter("servo_min_count").as_int();
        int servo_max_count = this->get_parameter("servo_max_count").as_int();
        double servo_min_rad = this->get_parameter("servo_min_rad").as_double();
        double servo_max_rad = this->get_parameter("servo_max_rad").as_double();

        int throttle_min_count = this->get_parameter("throttle_min_count").as_int();
        int throttle_max_count = this->get_parameter("throttle_max_count").as_int();
        double throttle_min_openness = this->get_parameter("throttle_min_openness").as_double();
        double throttle_max_openness = this->get_parameter("throttle_max_openness").as_double();

        int autopilot_threshold = this->get_parameter("autopilot_threshold").as_int();
        int autolanding_threshold = this->get_parameter("autolanding_threshold").as_int();

        int dropping_device_threshold_low = this->get_parameter("dropping_device_threshold_low").as_int();
        int dropping_device_threshold_high = this->get_parameter("dropping_device_threshold_high").as_int();

        int autopilot_mode_threshold_low = this->get_parameter("autopilot_mode_threshold_low").as_int();
        int autopilot_mode_threshold_high = this->get_parameter("autopilot_mode_threshold_high").as_int();

        // uint16からfloat32への変換（サーボカウント値からラジアンに変換）
        processed_msg.aileron_r = -countToRadians(msg->aileron_r, servo_min_count, servo_max_count, servo_min_rad, servo_max_rad);
        processed_msg.elevator = -countToRadians(msg->elevator, servo_min_count, servo_max_count, servo_min_rad, servo_max_rad);
        processed_msg.rudder = -countToRadians(msg->rudder, servo_min_count, servo_max_count, servo_min_rad, servo_max_rad);
        processed_msg.aileron_l = -countToRadians(msg->aileron_l, servo_min_count, servo_max_count, servo_min_rad, servo_max_rad);

        processed_msg.throttle = countToOpenness(msg->throttle, throttle_min_count, throttle_max_count, throttle_min_openness, throttle_max_openness);

        // is_autopilotの変換（しきい値以下かより上かでboolを返す）
        bool is_autopilot = msg->is_autopilot > autopilot_threshold;

        // dropping_deviceの変換（しきい値2つを用意してそれぞれの範囲で0,1,2を返す）
        if (msg->dropping_device <= dropping_device_threshold_low)
        {
            processed_msg.dropping_device = 0;
        }
        else if (msg->dropping_device <= dropping_device_threshold_high)
        {
            processed_msg.dropping_device = 1;
        }
        else
        {
            processed_msg.dropping_device = 2;
        }

        // is_autolanding_enabledの判定
        bool is_autolanding_enabled = msg->is_autolanding_enabled > autolanding_threshold;

        // turning_modeの変換
        if (!is_autopilot)
        {
            processed_msg.autopilot_mode = "manual";
        }
        else
        {
            if (is_autolanding_enabled)
            {
                processed_msg.autopilot_mode = "auto_landing";
            }
            else
            {
                if (msg->turning_mode <= autopilot_mode_threshold_low)
                {
                    processed_msg.autopilot_mode = "horizontal_turning";
                }
                else if (msg->turning_mode <= autopilot_mode_threshold_high)
                {
                    processed_msg.autopilot_mode = "rise_turning";
                }
                else
                {
                    processed_msg.autopilot_mode = "eight_turning";
                }
            }
        }

        // is_lost_frameとis_failsafeの変換（0,1をそのまま変換）
        processed_msg.is_lost_frame = msg->is_lost_frame != 0;
        processed_msg.is_failsafe = msg->is_failsafe != 0;

        // メッセージの配信
        publisher_->publish(processed_msg);

        RCLCPP_DEBUG(this->get_logger(), "SbusData メッセージを配信しました");
    }

    float countToRadians(uint16_t count, int min_count, int max_count, double min_rad, double max_rad)
    {
        // カウント値をラジアンに線形変換
        if (count <= min_count)
        {
            return static_cast<float>(min_rad);
        }
        else if (count >= max_count)
        {
            return static_cast<float>(max_rad);
        }
        else
        {
            double ratio = static_cast<double>(count - min_count) / static_cast<double>(max_count - min_count);
            return static_cast<float>(min_rad + ratio * (max_rad - min_rad));
        }
    }

    float countToOpenness(uint16_t count, int min_count, int max_count, double min_openness, double max_openness)
    {
        if (count <= min_count)
        {
            return static_cast<float>(min_openness);
        }
        else if (count >= max_count)
        {
            return static_cast<float>(max_openness);
        }
        else
        {
            double ratio = static_cast<double>(count - min_count) / static_cast<double>(max_count - min_count);
            return static_cast<float>(min_openness + ratio * (max_openness - min_openness));
        }
    }

    rclcpp::Publisher<ksenos_ground_msgs::msg::SbusData>::SharedPtr publisher_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusRawData>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SbusDataProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
