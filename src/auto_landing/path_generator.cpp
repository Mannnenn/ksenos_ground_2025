#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"

// Define the type for the spline
typedef Eigen::Spline<double, 3> Spline3d;

class PathGenerator : public rclcpp::Node
{
public:
    PathGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("path_generator", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("output_path_topic_name", "/path");

        // パラメータの取得
        std::string output_path_topic_name;
        this->get_parameter("output_path_topic_name", output_path_topic_name);

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_path_topic_name, 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathGenerator::on_timer, this));
    }

private:
    void on_timer()
    {
        try
        {
            std::vector<geometry_msgs::msg::TransformStamped> transforms;
            for (int i = 0;; ++i)
            {
                try
                {
                    std::string frame_id = "waypoint_" + std::to_string(i);
                    auto transform = tf_buffer_.lookupTransform("map", frame_id, tf2::TimePointZero);
                    transforms.push_back(transform);
                }
                catch (tf2::TransformException &ex)
                {
                    // Break the loop if no more waypoints are found
                    break;
                }
            }

            if (transforms.size() < 2)
            {
                RCLCPP_WARN(this->get_logger(), "Not enough waypoints found to generate a path.");
                return;
            }

            // Define control points and orientations
            // ROS2::xyzw, Eigen::Quaterniond::wxyz
            std::vector<Eigen::Vector3d> control_points;
            for (const auto &transform : transforms)
            {
                control_points.emplace_back(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z);
            }

            std::vector<Eigen::Quaterniond> orientations;
            for (const auto &transform : transforms)
            {
                orientations.emplace_back(
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z);
            }

            // Create the spline
            Eigen::MatrixXd points(3, control_points.size());
            for (size_t i = 0; i < control_points.size(); ++i)
            {
                points.col(i) = control_points[i];
            }

            Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2); // 2nd order spline

            // Generate the path
            geometry_msgs::msg::PoseArray path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = "map";

            for (double t = 0; t <= 1; t += 0.02)
            {
                Eigen::Vector3d point = spline(t);
                Eigen::Quaterniond orientation = interpolate_orientation(orientations, t);

                orientation.inverse().normalize();
                geometry_msgs::msg::Pose pose;
                pose.position.x = point.x();
                pose.position.y = point.y();
                pose.position.z = point.z();
                pose.orientation.x = orientation.x();
                pose.orientation.y = orientation.y();
                pose.orientation.z = orientation.z();
                pose.orientation.w = orientation.w();
                path_msg.poses.push_back(pose);
            }

            publisher_->publish(path_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    Eigen::Quaterniond interpolate_orientation(const std::vector<Eigen::Quaterniond> &orientations, double t)
    {
        // Simple linear interpolation for orientations
        size_t n = orientations.size();
        double segment_length = 1.0 / (n - 1);
        size_t segment = std::min(static_cast<size_t>(t / segment_length), n - 2);
        double local_t = (t - segment * segment_length) / segment_length;
        return orientations[segment].slerp(local_t, orientations[segment + 1]);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(PathGenerator)