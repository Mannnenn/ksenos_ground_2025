#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ksenos_ground_msgs/msg/rpy.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class CalcRpyFromQuat : public rclcpp::Node
{
public:
    CalcRpyFromQuat() : Node("calc_rpy_from_quat")
    {
        // パラメータの宣言
        this->declare_parameter("input_topic", "/imu/data");
        this->declare_parameter("output_topic", "/aircraft/rpy");
        this->declare_parameter("frame_id", "base_link");

        // パラメータの取得
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // サブスクライバーとパブリッシャーの初期化
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic, 10,
            std::bind(&CalcRpyFromQuat::imu_callback, this, std::placeholders::_1));

        rpy_publisher_ = this->create_publisher<ksenos_ground_msgs::msg::Rpy>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "calc_rpy_from_quat node started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // クォータニオンからRPYを計算
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // RPYメッセージの作成
        auto rpy_msg = ksenos_ground_msgs::msg::Rpy();
        rpy_msg.header.stamp = msg->header.stamp;
        rpy_msg.header.frame_id = frame_id_;
        rpy_msg.roll = -static_cast<float>(roll);
        rpy_msg.pitch = -static_cast<float>(pitch);
        rpy_msg.yaw = static_cast<float>(yaw);

        // メッセージの発行
        rpy_publisher_->publish(rpy_msg);

        // デバッグ情報の出力（オプション）
        RCLCPP_DEBUG(this->get_logger(),
                     "RPY: roll=%.3f, pitch=%.3f, yaw=%.3f [rad]",
                     roll, pitch, yaw);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<ksenos_ground_msgs::msg::Rpy>::SharedPtr rpy_publisher_;
    std::string frame_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalcRpyFromQuat>());
    rclcpp::shutdown();
    return 0;
}
