// Publish uint8 drop signal (0 or 1) according to base_link x position
// Output 1 when x exceeds threshold, 0 otherwise

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <string>

class ControlDropNode : public rclcpp::Node
{
public:
    ControlDropNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("control_drop", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Parameters
        this->declare_parameter<std::string>("world_frame", "start_point");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("drop_signal_topic", "drop_signal");
        this->declare_parameter<std::string>("marker_topic", "visualization_drop_marker");
        this->declare_parameter<double>("x_threshold", 25.0); // threshold x position [m]

        world_frame_ = this->get_parameter("world_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        drop_signal_topic_ = this->get_parameter("drop_signal_topic").as_string();
        marker_topic_ = this->get_parameter("marker_topic").as_string();
        x_threshold_ = this->get_parameter("x_threshold").as_double();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        drop_pub_ = this->create_publisher<std_msgs::msg::UInt8>(drop_signal_topic_, qos);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, qos);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ControlDropNode::on_timer, this));
        RCLCPP_INFO(this->get_logger(), "control_drop started. world_frame=%s base_frame=%s x_threshold=%.3f",
                    world_frame_.c_str(), base_frame_.c_str(), x_threshold_);
    }

private:
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_.lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
            return;
        }

        const double x = tf.transform.translation.x;
        const uint8_t drop_signal = compute_drop_signal(x);

        std_msgs::msg::UInt8 msg;
        msg.data = drop_signal;
        drop_pub_->publish(msg);

        // Visualize threshold plane
        publish_threshold_marker();
    }

    uint8_t compute_drop_signal(double x) const
    {
        return (x > x_threshold_) ? 1 : 0;
    }

    void publish_threshold_marker()
    {
        // Publish a large thin cube as YZ-plane at x = x_threshold_ in world_frame_
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = world_frame_;
        m.ns = "control_drop_threshold";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x_threshold_;
        m.pose.position.y = 0.0;
        m.pose.position.z = 2.5;
        m.pose.orientation.w = 1.0; // axis-aligned
        // Size: thin in X, wide in Y/Z to visualize the plane
        m.scale.x = 0.05; // thickness
        m.scale.y = 20.0; // span in Y
        m.scale.z = 5.0;  // span in Z
        m.color.a = 0.3f;
        m.color.r = 1.0f; // red for drop threshold
        m.color.g = 0.5f;
        m.color.b = 0.0f;
        m.lifetime = rclcpp::Duration(0, 200000000); // 0.2 sec
        marker_pub_->publish(m);
    }

    // pubs/timer
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr drop_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // params
    std::string world_frame_;
    std::string base_frame_;
    std::string drop_signal_topic_;
    std::string marker_topic_;
    double x_threshold_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ControlDropNode)