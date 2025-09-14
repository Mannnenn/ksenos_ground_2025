// 目標横加速度のソースをモードで切り替えるノード
// auto_landing:   /l1_acc (Float32)
// その他 (horizontal_turning, eight_turning, rise_turning): /target_acc (Float32)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ksenos_ground_msgs/msg/sbus_data.hpp"

class ModeTargetLatAccSelector : public rclcpp::Node
{
public:
    ModeTargetLatAccSelector() : Node("mode_target_lat_acc_selector")
    {
        // Publisher (統一出力): 目標横加速度
        // 入力の再配信ループを避けるため、入力トピック名と異なる名前にしています
        lat_acc_target_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "lateral_acceleration_target", 10);

        // Subscribers
        sbus_sub_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&ModeTargetLatAccSelector::sbus_callback, this, std::placeholders::_1));

        l1_acc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/l1_acc", 10,
            std::bind(&ModeTargetLatAccSelector::l1_acc_callback, this, std::placeholders::_1));

        target_acc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_acc", 10,
            std::bind(&ModeTargetLatAccSelector::target_acc_callback, this, std::placeholders::_1));

        // 初期値
        current_mode_.clear();
        latest_l1_acc_ = 0.0f;
        latest_target_acc_ = 0.0f;
        l1_acc_available_ = false;
        target_acc_available_ = false;

        RCLCPP_INFO(this->get_logger(), "Mode Target Lateral Acc Selector node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: sbus_data, l1_acc, target_acc");
        RCLCPP_INFO(this->get_logger(), "Publishing to: lateral_acceleration_target");
    }

private:
    // S.BUS (モード) コールバック
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        current_mode_ = msg->autopilot_mode;
        publish_lat_acc_target();
        RCLCPP_DEBUG(this->get_logger(), "Current autopilot mode: %s", current_mode_.c_str());
    }

    // L1 横加速度入力
    void l1_acc_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_l1_acc_ = msg->data;
        l1_acc_available_ = true;

        if (current_mode_ == "auto_landing")
        {
            publish_lat_acc_target();
        }

        RCLCPP_DEBUG(this->get_logger(), "L1 acc updated: %.3f", latest_l1_acc_);
    }

    // 目標横加速度入力（非 auto_landing 時に使用）
    void target_acc_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_target_acc_ = msg->data;
        target_acc_available_ = true;

        if (current_mode_ == "horizontal_turning" ||
            current_mode_ == "eight_turning" ||
            current_mode_ == "rise_turning")
        {
            publish_lat_acc_target();
        }

        RCLCPP_DEBUG(this->get_logger(), "Target acc updated: %.3f", latest_target_acc_);
    }

    void publish_lat_acc_target()
    {
        std_msgs::msg::Float32 out;
        bool should_publish = false;
        std::string source;

        if (current_mode_ == "auto_landing")
        {
            if (l1_acc_available_)
            {
                out.data = latest_l1_acc_;
                should_publish = true;
                source = "l1_acc";
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Mode %s requires l1_acc, but it's not available", current_mode_.c_str());
            }
        }
        else if (current_mode_ == "horizontal_turning" ||
                 current_mode_ == "eight_turning" ||
                 current_mode_ == "rise_turning")
        {
            if (target_acc_available_)
            {
                out.data = latest_target_acc_;
                should_publish = true;
                source = "target_acc";
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Mode %s requires target_acc, but it's not available", current_mode_.c_str());
            }
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Mode %s doesn't require lateral acc target publishing", current_mode_.c_str());
        }

        if (should_publish)
        {
            lat_acc_target_pub_->publish(out);
            RCLCPP_DEBUG(this->get_logger(),
                         "Published lateral_acceleration_target: %.3f (from %s, mode: %s)",
                         out.data, source.c_str(), current_mode_.c_str());
        }
    }

    // メンバ
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lat_acc_target_pub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr l1_acc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_acc_sub_;

    std::string current_mode_;
    float latest_l1_acc_;
    float latest_target_acc_;
    bool l1_acc_available_;
    bool target_acc_available_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeTargetLatAccSelector>());
    rclcpp::shutdown();
    return 0;
}
