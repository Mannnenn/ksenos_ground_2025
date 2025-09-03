#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ksenos_ground_msgs/msg/sbus_data.hpp"

class ModeTargetVelocitySelector : public rclcpp::Node
{
public:
    ModeTargetVelocitySelector() : Node("mode_target_velocity_selector")
    {
        // パブリッシャーの初期化
        velocity_target_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "velocity_target", 10);

        // サブスクライバーの初期化
        sbus_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&ModeTargetVelocitySelector::sbus_callback, this, std::placeholders::_1));

        // 速度データのサブスクライバー
        average_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "average_velocity", 10,
            std::bind(&ModeTargetVelocitySelector::average_velocity_callback, this, std::placeholders::_1));

        dynamic_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "dynamic_velocity", 10,
            std::bind(&ModeTargetVelocitySelector::dynamic_velocity_callback, this, std::placeholders::_1));

        // デフォルト値の初期化
        current_mode_ = "";
        latest_average_velocity_ = 0.0f;
        latest_dynamic_velocity_ = 0.0f;
        average_velocity_available_ = false;
        dynamic_velocity_available_ = false;

        RCLCPP_INFO(this->get_logger(), "Mode Target Velocity Selector node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: sbus_data, average_velocity, dynamic_velocity");
        RCLCPP_INFO(this->get_logger(), "Publishing to: velocity_target");
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        current_mode_ = msg->autopilot_mode;

        // モードに応じて適切な速度データを選択してパブリッシュ
        publish_velocity_target();

        RCLCPP_DEBUG(this->get_logger(), "Current autopilot mode: %s", current_mode_.c_str());
    }

    void average_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_average_velocity_ = msg->data;
        average_velocity_available_ = true;

        // auto_turning、eight_turning、rise_turningモードの場合はパブリッシュ
        if (current_mode_ == "horizontal rotation" ||
            current_mode_ == "eight_turning" ||
            current_mode_ == "rise_turn")
        {
            publish_velocity_target();
        }

        RCLCPP_DEBUG(this->get_logger(), "Average velocity updated: %.2f", latest_average_velocity_);
    }

    void dynamic_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_dynamic_velocity_ = msg->data;
        dynamic_velocity_available_ = true;

        // auto_pilotモードの場合はパブリッシュ
        if (current_mode_ == "auto_pilot")
        {
            publish_velocity_target();
        }

        RCLCPP_DEBUG(this->get_logger(), "Dynamic velocity updated: %.2f", latest_dynamic_velocity_);
    }

    void publish_velocity_target()
    {
        std_msgs::msg::Float32 target_msg;
        bool should_publish = false;
        std::string source = "";

        if (current_mode_ == "horizontal rotation" ||
            current_mode_ == "eight_turning" ||
            current_mode_ == "rise_turn")
        {
            // auto_turning、eight_turning、rise_turningモードの場合: average_velocityを使用
            if (average_velocity_available_)
            {
                target_msg.data = latest_average_velocity_;
                should_publish = true;
                source = "average_velocity";
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Mode %s requires average_velocity, but it's not available", current_mode_.c_str());
            }
        }
        else if (current_mode_ == "auto_pilot")
        {
            // auto_pilotモードの場合: dynamic_velocityを使用
            if (dynamic_velocity_available_)
            {
                target_msg.data = latest_dynamic_velocity_;
                should_publish = true;
                source = "dynamic_velocity";
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Mode %s requires dynamic_velocity, but it's not available", current_mode_.c_str());
            }
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Mode %s doesn't require velocity target publishing", current_mode_.c_str());
        }

        if (should_publish)
        {
            velocity_target_publisher_->publish(target_msg);
            RCLCPP_DEBUG(this->get_logger(),
                         "Published velocity_target: %.2f (from %s, mode: %s)",
                         target_msg.data, source.c_str(), current_mode_.c_str());
        }
    }

    // メンバ変数
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_target_publisher_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr average_velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dynamic_velocity_subscriber_;

    std::string current_mode_;
    float latest_average_velocity_;
    float latest_dynamic_velocity_;
    bool average_velocity_available_;
    bool dynamic_velocity_available_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeTargetVelocitySelector>());
    rclcpp::shutdown();
    return 0;
}
