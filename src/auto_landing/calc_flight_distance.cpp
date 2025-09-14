#include <rclcpp/rclcpp.hpp>
#include <ksenos_ground_msgs/msg/flow_rate_data.hpp>
#include <ksenos_ground_msgs/msg/sbus_data.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>

class CalcFlightDistance : public rclcpp::Node
{
public:
    CalcFlightDistance() : Node("calc_flight_distance")
    {
        // Subscriber
        flow_rate_sub_ = this->create_subscription<ksenos_ground_msgs::msg::FlowRateData>(
            "flow_rate_data", 10,
            std::bind(&CalcFlightDistance::flow_rate_callback, this, std::placeholders::_1));

        sbus_sub_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&CalcFlightDistance::sbus_callback, this, std::placeholders::_1));

        // Publisher
        flight_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("flight_distance", 10);

        // 初期化
        total_distance_ = 0.0;
        last_timestamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        current_mode_.clear();

        RCLCPP_INFO(this->get_logger(), "Flight distance calculator node started");
    }

private:
    void flow_rate_callback(const ksenos_ground_msgs::msg::FlowRateData::SharedPtr msg)
    {
        rclcpp::Time current_time = rclcpp::Time(msg->header.stamp);

        // 初回データまたはタイムスタンプが逆行した場合は時間差分を計算しない
        if (last_timestamp_.nanoseconds() == 0 || current_time < last_timestamp_)
        {
            last_timestamp_ = current_time;
            return;
        }

        // 時間差分を計算（秒）
        double dt = (current_time - last_timestamp_).seconds();

        // 異常な時間差分をチェック（10秒以上の場合はスキップ）
        if (dt > 10.0)
        {
            RCLCPP_WARN(this->get_logger(), "Large time gap detected (%.2f s), skipping integration", dt);
            last_timestamp_ = current_time;
            return;
        }

        // 距離を積算（流速 × 時間差分）
        total_distance_ += msg->flow_rate * dt;

        // 結果をパブリッシュ
        auto distance_msg = std_msgs::msg::Float32();
        distance_msg.data = static_cast<float>(total_distance_);
        flight_distance_pub_->publish(distance_msg);

        last_timestamp_ = current_time;

        RCLCPP_DEBUG(this->get_logger(),
                     "Flow rate: %.3f m/s, dt: %.3f s, Total distance: %.3f m (mode: %s)",
                     msg->flow_rate, dt, total_distance_, current_mode_.c_str());
    }

    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        const std::string new_mode = msg->autopilot_mode;

        // モードが変更された場合は毎回リセット
        if (new_mode != current_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "Autopilot mode changed: '%s' -> '%s'. Resetting distance counter.",
                        current_mode_.empty() ? "(none)" : current_mode_.c_str(), new_mode.c_str());
            total_distance_ = 0.0;
            last_timestamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            current_mode_ = new_mode;
        }
    }

    // Subscribers
    rclcpp::Subscription<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr flow_rate_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_sub_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flight_distance_pub_;

    // State variables
    double total_distance_;
    rclcpp::Time last_timestamp_;
    std::string current_mode_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalcFlightDistance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}