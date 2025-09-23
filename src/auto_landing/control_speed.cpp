// Publish target speed (Float32) according to base_link x position
// and visualize threshold planes (YZ planes) at x=5 and x=8.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "ksenos_ground_msgs/msg/flow_rate_data.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <string>
#include <algorithm>

class ControlSpeedNode : public rclcpp::Node
{
public:
    ControlSpeedNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("control_speed", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Parametersconst rclcpp::NodeOptions &options = rclcpp::NodeOptions()
        this->declare_parameter<std::string>("world_frame", "start_point");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("target_speed_topic", "target_speed");
        this->declare_parameter<std::string>("marker_topic", "visualization_speed_marker");
        this->declare_parameter<double>("x_sustain_end", 17.0); // until this x, keep max speed
        this->declare_parameter<double>("x_stop", 30.0);        // from sustain_end to this, linearly to 0; then 0
        this->declare_parameter<double>("v_max", 5.0);          // [m/s]

        world_frame_ = this->get_parameter("world_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        target_speed_topic_ = this->get_parameter("target_speed_topic").as_string();
        marker_topic_ = this->get_parameter("marker_topic").as_string();
        x_sustain_end_ = this->get_parameter("x_sustain_end").as_double();
        x_stop_ = this->get_parameter("x_stop").as_double();
        v_max_ = this->get_parameter("v_max").as_double();

        if (x_stop_ <= x_sustain_end_)
        {
            RCLCPP_WARN(this->get_logger(), "x_stop (%.3f) <= x_sustain_end (%.3f); adjusting x_stop", x_stop_, x_sustain_end_);
            x_stop_ = x_sustain_end_ + 0.1;
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        speed_pub_ = this->create_publisher<ksenos_ground_msgs::msg::FlowRateData>(target_speed_topic_, qos);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, qos);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ControlSpeedNode::on_timer, this));
        RCLCPP_INFO(this->get_logger(), "control_speed started. world_frame=%s base_frame=%s", world_frame_.c_str(), base_frame_.c_str());
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
        const float v = static_cast<float>(compute_target_speed(x));

        ksenos_ground_msgs::msg::FlowRateData msg;
        msg.flow_rate = v;
        speed_pub_->publish(msg);

        publish_plane_marker(x_sustain_end_, 0, 0.1f, 0.8f, 0.2f); // green-ish for x=5
        publish_plane_marker(x_stop_, 1, 0.9f, 0.2f, 0.2f);        // red-ish for x=8
    }

    double compute_target_speed(double x) const
    {
        if (x <= 0.0)
        {
            return v_max_;
        }
        if (x < x_sustain_end_)
        {
            return v_max_;
        }
        if (x < x_stop_)
        {
            const double t = (x - x_sustain_end_) / (x_stop_ - x_sustain_end_);
            return v_max_ * std::clamp(1.0 - t, 0.0, 1.0);
        }
        return -1.0; // スロットル停止命令
    }

    void publish_plane_marker(double x_plane, int id, float r, float g, float b)
    {
        // Publish a large thin cube as YZ-plane at x = x_plane in world_frame_
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = world_frame_;
        m.ns = "control_speed_planes";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x_plane;
        m.pose.position.y = 0.0;
        m.pose.position.z = 2.5;
        m.pose.orientation.w = 1.0; // axis-aligned
        // Size: thin in X, wide in Y/Z to visualize the plane
        m.scale.x = 0.05; // thickness
        m.scale.y = 20.0; // span in Y
        m.scale.z = 5.0;  // span in Z
        m.color.a = 0.25f;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.lifetime = rclcpp::Duration(0, 200000000); // 0.2 sec
        marker_pub_->publish(m);
    }

    // pubs/timer
    rclcpp::Publisher<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr speed_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // params
    std::string world_frame_;
    std::string base_frame_;
    std::string target_speed_topic_;
    std::string marker_topic_;
    double x_sustain_end_;
    double x_stop_;
    double v_max_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ControlSpeedNode)