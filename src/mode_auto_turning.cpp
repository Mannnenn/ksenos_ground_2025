#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ksenos_ground_msgs/msg/sbus_data.hpp"

class ModeAutoTurning : public rclcpp::Node
{
public:
    ModeAutoTurning() : Node("mode_auto_turning")
    {
        // パラメータの宣言と初期値設定
        this->declare_parameter<float>("target_radius", 1000.0f);

        // パラメータコールバックの設定
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ModeAutoTurning::parameter_callback, this, std::placeholders::_1));

        // 初期パラメータ値の取得
        target_radius_ = this->get_parameter("target_radius").as_double();

        // パブリッシャーの初期化
        target_radius_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "target_turning_radius", 10);

        // サブスクライバーの初期化
        sbus_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&ModeAutoTurning::sbus_callback, this, std::placeholders::_1));

        // デフォルト値の設定
        is_auto_turning_mode_ = false;

        RCLCPP_INFO(this->get_logger(), "Mode Auto Turning node initialized with target_radius: %.2f meters", target_radius_);
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // autopilot_modeが"auto_turning"の場合のみ動作
        is_auto_turning_mode_ = (msg->autopilot_mode == "horizontal_turning");

        if (is_auto_turning_mode_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Auto turning mode detected. Publishing target radius.");

            // 現在のパラメータ値を取得
            target_radius_ = this->get_parameter("target_radius").as_double();

            // 目標旋回半径をパブリッシュ
            auto radius_msg = std_msgs::msg::Float32();
            radius_msg.data = target_radius_;
            target_radius_publisher_->publish(radius_msg);

            RCLCPP_DEBUG(this->get_logger(), "Published target turning radius: %.2f meters", target_radius_);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Not in auto turning mode (current: %s).",
                         msg->autopilot_mode.c_str());
        }
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "target_radius")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    float new_radius = param.as_double();
                    if (new_radius > 0.0f)
                    {
                        target_radius_ = new_radius;
                        RCLCPP_INFO(this->get_logger(), "Target radius updated to: %.2f meters", target_radius_);
                    }
                    else
                    {
                        result.successful = false;
                        result.reason = "Target radius must be greater than 0";
                        RCLCPP_WARN(this->get_logger(), "Invalid target radius: %.2f. Must be > 0", new_radius);
                    }
                }
                else
                {
                    result.successful = false;
                    result.reason = "Target radius parameter must be a float/double";
                }
            }
        }

        return result;
    }

    // メンバ変数
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_radius_publisher_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscriber_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    bool is_auto_turning_mode_;
    float target_radius_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeAutoTurning>());
    rclcpp::shutdown();
    return 0;
}