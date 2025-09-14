#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ksenos_ground_msgs/msg/flow_rate_data.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>
#include <string>

namespace
{
    double normalize_angle(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a < -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    struct Vec2
    {
        double x{0.0}, y{0.0};
    };

    inline double lerp(double a, double b, double t)
    {
        return a + (b - a) * t;
    }

    double distance2(const Vec2 &a, const Vec2 &b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    double distance(const Vec2 &a, const Vec2 &b)
    {
        return std::sqrt(distance2(a, b));
    }
} // namespace

class L1ControlNode : public rclcpp::Node
{
public:
    L1ControlNode() : Node("control_L1_law"),
                      tf_buffer_(this->get_clock()),
                      tf_listener_(tf_buffer_)
    {
        // Parameters
        this->declare_parameter<std::string>("path_topic", "/path");
        this->declare_parameter<std::string>("velocity_topic", "estimated_velocity");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("eta_topic", "eta");
        this->declare_parameter<std::string>("marker_topic", "visualization_marker");
        this->declare_parameter<double>("lookahead_gain", 2.0);     // [s] equivalent (L1 = gain * speed)
        this->declare_parameter<double>("lookahead_min", 3.0);      // [m]
        this->declare_parameter<double>("lookahead_max", 50.0);     // [m]
        this->declare_parameter<bool>("publish_lateral_acc", true); // publish lateral acceleration by default
        this->declare_parameter<std::string>("lateral_acc_topic", "lateral_acceleration");
        this->declare_parameter<double>("min_speed_for_heading", 0.5);                    // [m/s] use velocity heading above this
        this->declare_parameter<std::string>("target_altitude_topic", "target_altitude"); // publish target altitude (L1 point z)

        std::string path_topic = this->get_parameter("path_topic").as_string();
        std::string velocity_topic = this->get_parameter("velocity_topic").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        eta_topic_ = this->get_parameter("eta_topic").as_string();
        marker_topic_ = this->get_parameter("marker_topic").as_string();
        lookahead_gain_ = this->get_parameter("lookahead_gain").as_double();
        lookahead_min_ = this->get_parameter("lookahead_min").as_double();
        lookahead_max_ = this->get_parameter("lookahead_max").as_double();
        publish_lateral_acc_ = this->get_parameter("publish_lateral_acc").as_bool();
        lateral_acc_topic_ = this->get_parameter("lateral_acc_topic").as_string();
        min_speed_for_heading_ = this->get_parameter("min_speed_for_heading").as_double();
        target_altitude_topic_ = this->get_parameter("target_altitude_topic").as_string();

        // QoS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        // Subscribers
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            path_topic, qos,
            std::bind(&L1ControlNode::on_path, this, std::placeholders::_1));

        vel_sub_ = this->create_subscription<ksenos_ground_msgs::msg::FlowRateData>(
            velocity_topic, qos,
            std::bind(&L1ControlNode::on_velocity, this, std::placeholders::_1));

        // Publishers
        eta_pub_ = this->create_publisher<std_msgs::msg::Float32>(eta_topic_, qos);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, qos);
        if (publish_lateral_acc_)
        {
            lat_acc_pub_ = this->create_publisher<std_msgs::msg::Float32>(lateral_acc_topic_, qos);
        }
        target_alt_pub_ = this->create_publisher<std_msgs::msg::Float32>(target_altitude_topic_, qos);

        // Timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&L1ControlNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "L1ControlNode started. path_topic=%s velocity_topic=%s base_frame=%s",
                    path_topic.c_str(), velocity_topic.c_str(), base_frame_.c_str());
    }

private:
    void on_path(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        latest_path_ = *msg;
        path_available_ = !latest_path_.poses.empty();
        path_frame_ = latest_path_.header.frame_id;
    }

    void on_velocity(const ksenos_ground_msgs::msg::FlowRateData::SharedPtr msg)
    {
        speed_ = msg->flow_rate;
        vel_available_ = true;
    }

    void on_timer()
    {
        if (!path_available_)
            return;

        // Lookup TF: path_frame_ <- base_frame_
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_.lookupTransform(path_frame_, base_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
            return;
        }

        // Current position and heading in path frame
        Vec2 p{tf.transform.translation.x, tf.transform.translation.y};

        // Heading from TF yaw (path frame)
        const auto &q = tf.transform.rotation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double heading = std::atan2(siny_cosp, cosy_cosp);

        // Build path positions (XY)
        const auto &poses = latest_path_.poses;
        const size_t n = poses.size();
        if (n < 2)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Path too short: %zu", n);
            return;
        }

        std::vector<Vec2> wpts;
        std::vector<double> wz;
        wpts.reserve(n);
        wz.reserve(n);
        for (const auto &pose : poses)
        {
            wpts.push_back(Vec2{pose.position.x, pose.position.y});
            wz.push_back(pose.position.z);
        }

        // Find closest point on polyline (segment projection)
        size_t seg_idx = 0; // segment start index i, closest point lies on [i, i+1]
        double seg_t = 0.0; // interpolation factor in [0,1]
        double best_d2 = std::numeric_limits<double>::infinity();
        Vec2 closest = wpts.front();
        double closest_z = wz.front();
        for (size_t i = 0; i + 1 < n; ++i)
        {
            Vec2 a = wpts[i];
            Vec2 b = wpts[i + 1];
            double vx = b.x - a.x;
            double vy = b.y - a.y;
            double len2 = vx * vx + vy * vy;
            double t = 0.0;
            if (len2 > 1e-12)
            {
                t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2;
                t = std::clamp(t, 0.0, 1.0);
            }
            Vec2 proj{a.x + t * vx, a.y + t * vy};
            double d2 = distance2(p, proj);
            if (d2 < best_d2)
            {
                best_d2 = d2;
                seg_idx = i;
                seg_t = t;
                closest = proj;
                closest_z = lerp(wz[i], wz[i + 1], t);
            }
        }

        // Compute L1 distance
        double L1 = std::clamp(lookahead_gain_ * std::max(speed_, 0.0), lookahead_min_, lookahead_max_);

        // March along the path from closest point by L1 distance
        Vec2 l1_point = closest;
        double l1_z = closest_z;
        double remaining = L1;
        size_t i = seg_idx;
        // If starting inside a segment, consume the remainder of this segment first
        if (i + 1 < n)
        {
            double seg_len = distance(wpts[i], wpts[i + 1]);
            double consumed = seg_len * (1.0 - seg_t);
            if (consumed >= remaining && seg_len > 1e-6)
            {
                double t = seg_t + remaining / seg_len;
                l1_point.x = wpts[i].x + t * (wpts[i + 1].x - wpts[i].x);
                l1_point.y = wpts[i].y + t * (wpts[i + 1].y - wpts[i].y);
                l1_z = lerp(wz[i], wz[i + 1], t);
                remaining = 0.0;
            }
            else
            {
                remaining -= consumed;
                ++i;
                if (i < n)
                {
                    l1_point = wpts[i];
                    l1_z = wz[i];
                }
            }
        }

        while (remaining > 0.0 && i + 1 < n)
        {
            const double seg_len = distance(wpts[i], wpts[i + 1]);
            if (seg_len <= 1e-6)
            {
                ++i;
                continue;
            }
            if (remaining <= seg_len)
            {
                const double t = remaining / seg_len;
                l1_point.x = wpts[i].x + t * (wpts[i + 1].x - wpts[i].x);
                l1_point.y = wpts[i].y + t * (wpts[i + 1].y - wpts[i].y);
                l1_z = lerp(wz[i], wz[i + 1], t);
                remaining = 0.0;
                break;
            }
            remaining -= seg_len;
            ++i;
            l1_point = wpts[i];
            l1_z = wz[i];
        }
        // If we ran out of path, clamp to last point (end handling)
        if (remaining > 0.0)
        {
            l1_point = wpts.back();
            l1_z = wz.back();
        }

        // Compute eta
        const double to_l1 = std::atan2(l1_point.y - p.y, l1_point.x - p.x);
        const double eta = normalize_angle(to_l1 - heading);

        // Publish eta
        std_msgs::msg::Float32 eta_msg;
        eta_msg.data = static_cast<float>(eta);
        eta_pub_->publish(eta_msg);

        // Publish target altitude (z of the L1 point)
        std_msgs::msg::Float32 alt_msg;
        alt_msg.data = static_cast<float>(l1_z);
        target_alt_pub_->publish(alt_msg);

        // Optionally compute and publish lateral acceleration: a_lat = v^2 * sin(eta) / L1
        if (publish_lateral_acc_ && L1 > 1e-3)
        {
            const double a_lat = (speed_ * speed_) * std::sin(eta) / L1;
            std_msgs::msg::Float32 a_msg;
            a_msg.data = static_cast<float>(a_lat);
            lat_acc_pub_->publish(a_msg);
        }

        // Visualize nearest point and L1 point
        publish_marker(closest, closest_z, 0, 1.0, 0.2, 0.2); // red for nearest
        publish_marker(l1_point, l1_z, 1, 0.2, 0.8, 1.0);     // cyan for L1 point
    }

    void publish_marker(const Vec2 &pt, double z, int id, double r, double g, double b)
    {
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = path_frame_;
        m.ns = "l1_control";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = pt.x;
        m.pose.position.y = pt.y;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.5;
        m.scale.y = 0.5;
        m.scale.z = 0.5;
        m.color.a = 0.9;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.lifetime = rclcpp::Duration(0, 200000000); // 0.2 sec
        marker_pub_->publish(m);
    }

    // Subs/Pubs
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr eta_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lat_acc_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_alt_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // State
    geometry_msgs::msg::PoseArray latest_path_;
    bool path_available_{false};
    bool vel_available_{false};
    std::string path_frame_{"map"};
    std::string base_frame_;
    std::string eta_topic_;
    std::string marker_topic_;
    std::string lateral_acc_topic_;
    std::string target_altitude_topic_;

    // Velocity
    double speed_{4.0};

    // Params
    double lookahead_gain_{2.0};
    double lookahead_min_{3.0};
    double lookahead_max_{10.0};
    bool publish_lateral_acc_{false};
    double min_speed_for_heading_{0.5};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<L1ControlNode>());
    rclcpp::shutdown();
    return 0;
}
