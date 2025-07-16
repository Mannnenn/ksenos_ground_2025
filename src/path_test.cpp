#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/joy.hpp>

class LinearTFPublisher : public rclcpp::Node
{
public:
    LinearTFPublisher()
        : Node("linear_tf_publisher"), x_(0.0), y_(0.0), z_(0.0), r_(0.0), p_(0.0), yo_(0.0)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&LinearTFPublisher::joy_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LinearTFPublisher::broadcast_transform, this));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() >= 6)
        {
            x_ += -0.03 * msg->axes[1];
            y_ += -0.03 * msg->axes[0];
            yo_ += 0.1 * msg->axes[4];
        }
    }

    void broadcast_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";

        transform_stamped.transform.translation.x = x_;
        transform_stamped.transform.translation.y = y_;
        transform_stamped.transform.translation.z = z_;

        tf2::Quaternion quaternion;
        quaternion.setRPY(r_, p_, yo_);
        transform_stamped.transform.rotation.x = quaternion.x();
        transform_stamped.transform.rotation.y = quaternion.y();
        transform_stamped.transform.rotation.z = quaternion.z();
        transform_stamped.transform.rotation.w = quaternion.w();

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    double x_, y_, z_;
    double r_, p_, yo_;
};
;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearTFPublisher>());
    rclcpp::shutdown();
    return 0;
}