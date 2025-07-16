#include "nokolat2024/main_control.hpp"

class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), current_target_index_(0)
    {
        this->declare_parameter<std::string>("input_path_topic_name", "/path");
        this->declare_parameter<std::string>("input_mode_topic_name", "/mode");
        this->declare_parameter<std::string>("output_cmd_vel_topic_name", "/cmd_vel");
        this->declare_parameter<std::string>("output_target_pose_topic_name", "/target_pose");

        // パラメータの取得
        std::string input_path_topic_name;
        this->get_parameter("input_path_topic_name", input_path_topic_name);
        std::string output_cmd_vel_topic_name;
        std::string input_mode_topic_name;
        this->get_parameter("input_mode_topic_name", input_mode_topic_name);
        this->get_parameter("output_cmd_vel_topic_name", output_cmd_vel_topic_name);
        std::string output_target_pose_topic_name;
        this->get_parameter("output_target_pose_topic_name", output_target_pose_topic_name);

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            input_path_topic_name, 10, std::bind(&PathFollower::path_callback, this, std::placeholders::_1));

        mode_subscriber_ = this->create_subscription<std_msgs::msg::String>(input_mode_topic_name, 10, std::bind(&PathFollower::mode_callback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_name, 10);

        target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_target_pose_topic_name, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&PathFollower::update_target, this));
    }

private:
    void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        path_ = msg->poses;
        // current_target_index_ = 0; // Reset target index when a new path is received
    }

    void mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        control_mode_ = msg->data;

        if (control_mode_ != nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_LANDING))
        {
            current_target_index_ = 0;
        }
    }

    void update_target()
    {
        if (path_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path is empty");
            return;
        }

        if (control_mode_ != nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_LANDING))
        {
            return;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.pose.position.x = transformStamped.transform.translation.x;
        current_pose.pose.position.y = transformStamped.transform.translation.y;
        current_pose.pose.position.z = transformStamped.transform.translation.z;
        current_pose.pose.orientation = transformStamped.transform.rotation;

        while (current_target_index_ < path_.size())
        {
            auto target_pose = path_[current_target_index_];

            // Publish the current target pose
            geometry_msgs::msg::PoseStamped target_pose_stamped;
            target_pose_stamped.header.stamp = this->now();
            target_pose_stamped.header.frame_id = "map";
            target_pose_stamped.pose = target_pose;
            target_pose_publisher_->publish(target_pose_stamped);

            double dx = target_pose.position.x - current_pose.pose.position.x;
            double dy = target_pose.position.y - current_pose.pose.position.y;

            double distance = std::sqrt(dx * dx + dy * dy);
            // double angle_to_target = std::atan2(dy, dx);

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            tf2::Matrix3x3 m(q);

            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // double angle_diff = std::fabs(angle_to_target - yaw);

            // 目標点が機体前方1m、または目標点が機体の後方にある場合に次の目標点に切り替える
            if (distance < 1 || dx + 0.5 > 0)
            {
                current_target_index_++;
            }
            else
            {
                // RCLCPP_INFO(this->get_logger(), "Target Position: [%.2f, %.2f, %.2f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);
                // RCLCPP_INFO(this->get_logger(), "Target Orientation: [%.2f, %.2f, %.2f, %.2f]", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
                // RCLCPP_INFO(this->get_logger(), "Angle diff: %.2f", angle_diff);
                follow_target(target_pose, current_pose.pose);
                break;
            }
        }

        if (current_target_index_ >= path_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All targets reached");
            stop_robot();
        }
    }

    void follow_target(const geometry_msgs::msg::Pose &target_pose, const geometry_msgs::msg::Pose &current_pose)
    {
        geometry_msgs::msg::Twist cmd_vel;

        double dx = target_pose.position.x - current_pose.position.x;
        double dy = target_pose.position.y - current_pose.position.y;
        double dz = target_pose.position.z - current_pose.position.z;

        double angle_to_target = std::atan2(dy, dx);
        double pitch_to_target = std::atan2(dz, std::sqrt(dx * dx + dy * dy));

        tf2::Quaternion q(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double yaw_diff = angle_to_target - yaw;
        double pitch_diff = pitch_to_target - pitch;

        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = target_pose.position.z;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = pitch_diff;
        cmd_vel.angular.z = yaw_diff;

        // RCLCPP_INFO(this->get_logger(), "Altitude: %.2f, Yaw: %.2f, Pitch: %.2f", target_pose.position.z, yaw_diff, pitch_diff);

        cmd_vel_publisher_->publish(cmd_vel);
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.angular.y = 0;
        cmd_vel_publisher_->publish(cmd_vel);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<geometry_msgs::msg::Pose> path_;
    size_t current_target_index_;

    std::string control_mode_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}