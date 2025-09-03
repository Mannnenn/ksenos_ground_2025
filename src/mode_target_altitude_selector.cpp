#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ksenos_ground_msgs/msg/sbus_data.hpp"

class ModeTargetAltitudeSelector : public rclcpp::Node
{
public:
    ModeTargetAltitudeSelector() : Node("mode_target_altitude_selector")
    {
        // パブリッシャーの初期化
        altitude_target_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "altitude_target", 10);

        // サブスクライバーの初期化
        sbus_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&ModeTargetAltitudeSelector::sbus_callback, this, std::placeholders::_1));

        // 高度データのサブスクライバー
        average_altitude_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "average_altitude", 10,
            std::bind(&ModeTargetAltitudeSelector::average_altitude_callback, this, std::placeholders::_1));

        altitude_dynamic_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "altitude_dynamic", 10,
            std::bind(&ModeTargetAltitudeSelector::altitude_dynamic_callback, this, std::placeholders::_1));

        // デフォルト値の初期化
        current_mode_ = "";
        latest_average_altitude_ = 0.0f;
        latest_altitude_dynamic_ = 0.0f;
        average_altitude_available_ = false;
        altitude_dynamic_available_ = false;

        RCLCPP_INFO(this->get_logger(), "Mode Target Altitude Selector node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: sbus_data, average_altitude, altitude_dynamic");
        RCLCPP_INFO(this->get_logger(), "Publishing to: altitude_target");
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        current_mode_ = msg->autopilot_mode;

        // モードに応じて適切な高度データを選択してパブリッシュ
        publish_altitude_target();

        RCLCPP_DEBUG(this->get_logger(), "Current autopilot mode: %s", current_mode_.c_str());
    }

    void average_altitude_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_average_altitude_ = msg->data;
        average_altitude_available_ = true;

        // auto_turningまたはeight_turningモードの場合はパブリッシュ
        if (current_mode_ == "horizontal rotation" || current_mode_ == "eight_turning")
        {
            publish_altitude_target();
        }

        RCLCPP_DEBUG(this->get_logger(), "Average altitude updated: %.2f", latest_average_altitude_);
    }

    void altitude_dynamic_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_altitude_dynamic_ = msg->data;
        altitude_dynamic_available_ = true;

        // rise_turningまたはauto_pilotモードの場合はパブリッシュ
        if (current_mode_ == "rise_turn" || current_mode_ == "auto_pilot")
        {
            publish_altitude_target();
        }

        RCLCPP_DEBUG(this->get_logger(), "Altitude dynamic updated: %.2f", latest_altitude_dynamic_);
    }

    void publish_altitude_target()
    {
        std_msgs::msg::Float32 target_msg;
        bool should_publish = false;
        std::string source = "";

        if (current_mode_ == "horizontal rotation" || current_mode_ == "eight_turning")
        {
            // auto_turningまたはeight_turningモードの場合: average_altitudeを使用
            if (average_altitude_available_)
            {
                target_msg.data = latest_average_altitude_;
                should_publish = true;
                source = "average_altitude";
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Mode %s requires average_altitude, but it's not available", current_mode_.c_str());
            }
        }
        else if (current_mode_ == "rise_turn" || current_mode_ == "auto_pilot")
        {
            // rise_turningまたはauto_pilotモードの場合: altitude_dynamicを使用
            if (altitude_dynamic_available_)
            {
                target_msg.data = latest_altitude_dynamic_;
                should_publish = true;
                source = "altitude_dynamic";
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Mode %s requires altitude_dynamic, but it's not available", current_mode_.c_str());
            }
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Mode %s doesn't require altitude target publishing", current_mode_.c_str());
        }

        if (should_publish)
        {
            altitude_target_publisher_->publish(target_msg);
            RCLCPP_DEBUG(this->get_logger(),
                         "Published altitude_target: %.2f (from %s, mode: %s)",
                         target_msg.data, source.c_str(), current_mode_.c_str());
        }
    }

    // メンバ変数
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_target_publisher_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr average_altitude_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_dynamic_subscriber_;

    std::string current_mode_;
    float latest_average_altitude_;
    float latest_altitude_dynamic_;
    bool average_altitude_available_;
    bool altitude_dynamic_available_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeTargetAltitudeSelector>());
    rclcpp::shutdown();
    return 0;
}
