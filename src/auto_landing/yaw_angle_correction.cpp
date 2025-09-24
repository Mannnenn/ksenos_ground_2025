#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class YawAngleCorrection : public rclcpp::Node
{
public:
    YawAngleCorrection() : Node("yaw_angle_correction")
    {
        // TF2のバッファとリスナーを初期化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // TFブロードキャスターを初期化
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // yaw_correctトピックをサブスクライブ
        yaw_correct_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/yaw_correct", 10,
            std::bind(&YawAngleCorrection::yaw_correct_callback, this, std::placeholders::_1));

        // 初期状態（yaw=0）でTFを配信するタイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&YawAngleCorrection::publish_transform, this));

        yaw_correction_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "YawAngleCorrection node started");
    }

private:
    void yaw_correct_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        try
        {
            // ksenos_smooth0からimu_linkまでのtransformを取得
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                "ksenos_smooth0", "imu_link", tf2::TimePointZero);

            // クォータニオンからヨー角を取得
            tf2::Quaternion q;
            tf2::fromMsg(transform_stamped.transform.rotation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // 補正値を計算（既存のyaw角の逆 + yaw_correct）
            yaw_correction_ = -yaw + msg->data;

            RCLCPP_INFO(this->get_logger(),
                        "Current yaw: %.3f, Correction: %.3f, New yaw: %.3f",
                        yaw, msg->data, yaw_correction_);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            // エラーの場合は受信した値をそのまま使用
            yaw_correction_ = msg->data;
        }
    }

    void publish_transform()
    {
        geometry_msgs::msg::TransformStamped t;

        // Header設定
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "imu_link";
        t.child_frame_id = "aircraft_stability_axes";

        // 位置は固定（0, 0, 0）
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        // ヨー角補正のクォータニオンを設定
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_correction_);
        t.transform.rotation = tf2::toMsg(q);

        // TFを配信
        tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_correct_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double yaw_correction_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawAngleCorrection>());
    rclcpp::shutdown();
    return 0;
}