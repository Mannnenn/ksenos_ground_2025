#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ksenos_ground_msgs/msg/sbus_data.hpp>
#include <ksenos_ground_msgs/msg/sbus_raw_data.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>
#include <limits>

class SbusDataToRawProcessor : public rclcpp::Node
{
public:
    SbusDataToRawProcessor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("sbus_data_to_raw_processor", options)
    {
        // パブリッシャーの作成
        publisher_ = this->create_publisher<ksenos_ground_msgs::msg::SbusRawData>(
            "sbus_raw_data", 10);

        // サブスクライバーの作成
        subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&SbusDataToRawProcessor::sbusDataCallback, this, std::placeholders::_1));

        // パラメータの宣言と初期化
        this->declare_parameter("servo_min_count", 352);
        this->declare_parameter("servo_max_count", 1696);
        this->declare_parameter("servo_min_rad", -0.5236); // -30度をラジアンで
        this->declare_parameter("servo_max_rad", 0.5236);  // 30度をラジアンで

        this->declare_parameter("throttle_min_count", 352);
        this->declare_parameter("throttle_max_count", 1696);
        this->declare_parameter("throttle_min_openness", 0.0); // 開度の最小値
        this->declare_parameter("throttle_max_openness", 1.0); // 開度の最大値

        this->declare_parameter("dropping_device_min", 176);
        this->declare_parameter("dropping_device_max", 1385);

        RCLCPP_INFO(this->get_logger(), "SBUS Data To Raw Processor ノードが開始されました");
    }

private:
    void sbusDataCallback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        auto raw_msg = ksenos_ground_msgs::msg::SbusRawData();

        // ヘッダーをコピー
        raw_msg.header = msg->header;

        // パラメータの取得
        int servo_min_count = this->get_parameter("servo_min_count").as_int();
        int servo_max_count = this->get_parameter("servo_max_count").as_int();
        double servo_min_rad = this->get_parameter("servo_min_rad").as_double();
        double servo_max_rad = this->get_parameter("servo_max_rad").as_double();

        int throttle_min_count = this->get_parameter("throttle_min_count").as_int();
        int throttle_max_count = this->get_parameter("throttle_max_count").as_int();
        double throttle_min_openness = this->get_parameter("throttle_min_openness").as_double();
        double throttle_max_openness = this->get_parameter("throttle_max_openness").as_double();

        int dropping_device_min = this->get_parameter("dropping_device_min").as_int();
        int dropping_device_max = this->get_parameter("dropping_device_max").as_int();

        // ラジアンからカウント値への変換（負の符号を適用）
        raw_msg.aileron_r = radiansToCount(-msg->aileron_r, servo_min_count, servo_max_count, servo_min_rad, servo_max_rad);
        raw_msg.elevator = radiansToCount(-msg->elevator, servo_min_count, servo_max_count, servo_min_rad, servo_max_rad);
        raw_msg.rudder = radiansToCount(-msg->rudder, servo_min_count, servo_max_count, servo_min_rad, servo_max_rad);
        raw_msg.aileron_l = radiansToCount(-msg->aileron_l, servo_min_count, servo_max_count, servo_min_rad, servo_max_rad);

        // 開度からカウント値への変換
        raw_msg.throttle = opennessToCount(msg->throttle, throttle_min_count, throttle_max_count, throttle_min_openness, throttle_max_openness);

        // dropping_deviceはそのまま
        if (msg->dropping_device == 0)
        {
            raw_msg.dropping_device = static_cast<uint16_t>(dropping_device_min); // ドロップしない場合は最小値
        }
        else if (msg->dropping_device == 1)
        {
            raw_msg.dropping_device = static_cast<uint16_t>(dropping_device_max);
        }

        // その他のチャンネルはuint16の最大値で固定
        raw_msg.is_autopilot = std::numeric_limits<uint16_t>::max();
        raw_msg.is_autolanding_enabled = std::numeric_limits<uint16_t>::max();
        raw_msg.turning_mode = std::numeric_limits<uint16_t>::max();
        raw_msg.is_lost_frame = std::numeric_limits<uint16_t>::max();
        raw_msg.is_failsafe = std::numeric_limits<uint16_t>::max();

        // メッセージの配信
        publisher_->publish(raw_msg);

        RCLCPP_DEBUG(this->get_logger(), "SbusRawData メッセージを配信しました");
    }

    uint16_t radiansToCount(float radians, int min_count, int max_count, double min_rad, double max_rad)
    {
        // ラジアンをカウント値に線形変換
        if (radians <= min_rad)
        {
            return static_cast<uint16_t>(min_count);
        }
        else if (radians >= max_rad)
        {
            return static_cast<uint16_t>(max_count);
        }
        else
        {
            double ratio = (radians - min_rad) / (max_rad - min_rad);
            return static_cast<uint16_t>(min_count + ratio * (max_count - min_count));
        }
    }

    uint16_t opennessToCount(float openness, int min_count, int max_count, double min_openness, double max_openness)
    {
        // 開度をカウント値に線形変換
        if (openness <= min_openness)
        {
            return static_cast<uint16_t>(min_count);
        }
        else if (openness >= max_openness)
        {
            return static_cast<uint16_t>(max_count);
        }
        else
        {
            double ratio = (openness - min_openness) / (max_openness - min_openness);
            return static_cast<uint16_t>(min_count + ratio * (max_count - min_count));
        }
    }

    rclcpp::Publisher<ksenos_ground_msgs::msg::SbusRawData>::SharedPtr publisher_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr subscription_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(SbusDataToRawProcessor)
