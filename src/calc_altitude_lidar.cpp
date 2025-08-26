#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class AltitudeLidarNode : public rclcpp::Node
{
public:
    AltitudeLidarNode() : Node("calc_altitude_lidar")
    {
        // TF2バッファとリスナーを初期化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // パブリッシャーを作成（/altitude_lidarトピック）
        altitude_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/altitude_lidar", 10);

        // 10Hzのタイマーを作成
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100ms = 10Hz
            std::bind(&AltitudeLidarNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Altitude LiDAR node started. Publishing at 10Hz on /altitude_lidar");
    }

private:
    void timer_callback()
    {
        try
        {
            // map -> ksenos_smooth_0 の変換を取得
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                "map", "ksenos_smooth_0",
                tf2::TimePointZero // 最新の変換を取得
            );

            // z座標（高度）を取得
            float altitude = static_cast<float>(transform_stamped.transform.translation.z);

            // Float32メッセージを作成して配信
            auto altitude_msg = std_msgs::msg::Float32();
            altitude_msg.data = altitude;
            altitude_publisher_->publish(altitude_msg);

            RCLCPP_DEBUG(this->get_logger(), "Published altitude: %.3f m", altitude);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000, // 1秒間隔でワーニングを出力
                "Could not transform map to ksenos_smooth_0: %s", ex.what());
        }
    }

    // メンバ変数
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AltitudeLidarNode>());
    rclcpp::shutdown();
    return 0;
}