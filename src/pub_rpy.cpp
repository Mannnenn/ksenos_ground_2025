#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ksenos_ground_msgs/msg/rpy.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class RPYPublisher : public rclcpp::Node
{
public:
    RPYPublisher() : Node("pub_rpy_node")
    {
        publisher_ = this->create_publisher<ksenos_ground_msgs::msg::Rpy>("rpy", 10);
        angular_velocity_publisher_ = this->create_publisher<ksenos_ground_msgs::msg::Rpy>("rpy_angular_velocity", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(100ms, std::bind(&RPYPublisher::timer_callback, this));

        // 初期化
        first_callback_ = true;
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform("odom", "imu_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to imu_link: %s", ex.what());
            return;
        }

        // クォータニオンからroll, pitch, yawを計算
        double roll, pitch, yaw;
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ksenos_ground_msgs::msg::Rpy msg;
        msg.roll = roll;
        msg.pitch = pitch;
        msg.yaw = yaw;

        publisher_->publish(msg);

        // 角速度の計算
        auto current_time = std::chrono::steady_clock::now();

        if (!first_callback_)
        {
            // 時間差を計算
            auto time_diff = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - previous_time_).count();
            double dt = time_diff / 1e9; // 秒に変換

            if (dt > 0.0)
            {
                // クォータニオンの差分を計算
                tf2::Quaternion q_diff = q * previous_quaternion_.inverse();

                // 角速度を計算（小角度近似）
                double angular_roll = 2.0 * q_diff.x() / dt;
                double angular_pitch = 2.0 * q_diff.y() / dt;
                double angular_yaw = 2.0 * q_diff.z() / dt;

                // 角速度メッセージを作成してパブリッシュ
                ksenos_ground_msgs::msg::Rpy angular_velocity_msg;
                angular_velocity_msg.roll = angular_roll;
                angular_velocity_msg.pitch = angular_pitch;
                angular_velocity_msg.yaw = angular_yaw;

                angular_velocity_publisher_->publish(angular_velocity_msg);
            }
        }
        else
        {
            first_callback_ = false;
        }

        // 次回のために現在の値を保存
        previous_quaternion_ = q;
        previous_time_ = current_time;
    }

    rclcpp::Publisher<ksenos_ground_msgs::msg::Rpy>::SharedPtr publisher_;
    rclcpp::Publisher<ksenos_ground_msgs::msg::Rpy>::SharedPtr angular_velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 角速度計算用の変数
    bool first_callback_;
    tf2::Quaternion previous_quaternion_;
    std::chrono::steady_clock::time_point previous_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RPYPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}