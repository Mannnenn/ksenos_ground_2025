#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ksenos_ground_msgs/msg/sbus_data.hpp"
#include "ksenos_ground_msgs/msg/flow_rate_data.hpp"
#include <deque>
#include <numeric>

class AverageAltitudeVelocityNode : public rclcpp::Node
{
public:
    AverageAltitudeVelocityNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("calc_average_altitude_velocity", options), is_manual_mode_(false)
    {
        // SbusDataのサブスクライバー
        sbus_subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "/sbus_data", 10,
            std::bind(&AverageAltitudeVelocityNode::sbus_callback, this, std::placeholders::_1));

        // FlowRateDataのサブスクライバー
        flow_rate_subscription_ = this->create_subscription<ksenos_ground_msgs::msg::FlowRateData>(
            "/sensor/flow_rate", 10,
            std::bind(&AverageAltitudeVelocityNode::flow_rate_callback, this, std::placeholders::_1));

        altitude_imu_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/sensor/altitude_imu", 10,
            std::bind(&AverageAltitudeVelocityNode::altitude_imu_callback, this, std::placeholders::_1));

        altitude_lidar_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/sensor/altitude_lidar", 10,
            std::bind(&AverageAltitudeVelocityNode::altitude_lidar_callback, this, std::placeholders::_1));

        // 平均値パブリッシャー
        avg_flow_rate_publisher_ = this->create_publisher<ksenos_ground_msgs::msg::FlowRateData>(
            "/average/flow_rate", 10);

        avg_altitude_imu_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/average/altitude_imu", 10);

        avg_altitude_lidar_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/average/altitude_lidar", 10);

        RCLCPP_INFO(this->get_logger(), "Average Altitude Velocity Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /sbus_data, /sensor/flow_rate");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /average/flow_rate, /average/altitude_imu, /average/altitude_lidar");
        RCLCPP_INFO(this->get_logger(), "Only processes data when autopilot_mode is 'manual'");
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // autopilot_modeが"manual"の場合のみ動作
        is_manual_mode_ = (msg->autopilot_mode == "manual");

        if (is_manual_mode_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Manual mode detected. Ready to process flow rate data.");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Not in manual mode (current: %s). Clearing data queues.",
                         msg->autopilot_mode.c_str());
            // マニュアルモードでない場合は蓄積データをクリア
            flow_rate_data_.clear();
            altitude_imu_data_.clear();
            altitude_lidar_data_.clear();
        }
    }

    void flow_rate_callback(const ksenos_ground_msgs::msg::FlowRateData::SharedPtr msg)
    {
        // マニュアルモードの場合のみデータを処理
        if (!is_manual_mode_)
        {
            return;
        }

        // flow_rateデータを追加
        flow_rate_data_.push_back(msg->flow_rate);
        if (flow_rate_data_.size() > QUEUE_SIZE_HIGH_RATE)
        {
            flow_rate_data_.pop_front();
        }

        // 10個のデータが揃った場合に平均を計算・パブリッシュ
        if (flow_rate_data_.size() == QUEUE_SIZE_HIGH_RATE)
        {
            // flow_rateの平均を計算・パブリッシュ
            float avg_flow_rate = std::accumulate(flow_rate_data_.begin(), flow_rate_data_.end(), 0.0f) / QUEUE_SIZE_HIGH_RATE;
            auto flow_rate_msg = ksenos_ground_msgs::msg::FlowRateData();
            flow_rate_msg.flow_rate = avg_flow_rate;
            avg_flow_rate_publisher_->publish(flow_rate_msg);
        }
    }

    void altitude_imu_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // マニュアルモードの場合のみデータを処理
        if (!is_manual_mode_)
        {
            return;
        }

        // altitude_imuデータを追加
        altitude_imu_data_.push_back(msg->data);
        if (altitude_imu_data_.size() > QUEUE_SIZE_HIGH_RATE)
        {
            altitude_imu_data_.pop_front();
        }

        // 10個のデータが揃った場合に平均を計算・パブリッシュ
        if (altitude_imu_data_.size() == QUEUE_SIZE_HIGH_RATE)
        {
            // altitude_imuの平均を計算・パブリッシュ
            float avg_altitude_imu = std::accumulate(altitude_imu_data_.begin(), altitude_imu_data_.end(), 0.0f) / QUEUE_SIZE_HIGH_RATE;
            auto altitude_imu_msg = std_msgs::msg::Float32();
            altitude_imu_msg.data = avg_altitude_imu;
            avg_altitude_imu_publisher_->publish(altitude_imu_msg);
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "Altitude IMU data received: altitude_imu=%.3f. Queue size: %zu",
                     msg->data, altitude_imu_data_.size());
    }

    void altitude_lidar_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // マニュアルモードの場合のみデータを処理
        if (!is_manual_mode_)
        {
            return;
        }

        // altitude_lidarデータを追加
        altitude_lidar_data_.push_back(msg->data);
        if (altitude_lidar_data_.size() > QUEUE_SIZE_LOW_RATE)
        {
            altitude_lidar_data_.pop_front();
        }

        // 10個のデータが揃った場合に平均を計算・パブリッシュ
        if (altitude_lidar_data_.size() == QUEUE_SIZE_LOW_RATE)
        {
            // altitude_lidarの平均を計算・パブリッシュ
            float avg_altitude_lidar = std::accumulate(altitude_lidar_data_.begin(), altitude_lidar_data_.end(), 0.0f) / QUEUE_SIZE_LOW_RATE;
            auto altitude_lidar_msg = std_msgs::msg::Float32();
            altitude_lidar_msg.data = avg_altitude_lidar;
            avg_altitude_lidar_publisher_->publish(altitude_lidar_msg);
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "Altitude Lidar data received: altitude_lidar=%.3f. Queue size: %zu",
                     msg->data, altitude_lidar_data_.size());
    }
    // サブスクライバー
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscription_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr flow_rate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_lidar_subscription_;

    // パブリッシャー
    rclcpp::Publisher<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr avg_flow_rate_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr avg_altitude_imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr avg_altitude_lidar_publisher_;

    // データ格納用のキュー（10個の平均を取る）
    std::deque<float> flow_rate_data_;
    std::deque<float> altitude_imu_data_;
    std::deque<float> altitude_lidar_data_;

    // 状態管理
    bool is_manual_mode_;
    const size_t QUEUE_SIZE_LOW_RATE = 10;
    const size_t QUEUE_SIZE_HIGH_RATE = 10;
};

// コンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(AverageAltitudeVelocityNode)
