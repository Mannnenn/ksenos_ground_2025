// Copyright 2025 ksenos_ground
//
// Licensed under the MIT License (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_ros/transforms.hpp"

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("pointcloud_transform_node", options),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // パラメータの宣言と初期値設定
        this->declare_parameter("input_topic", "lidar_points");
        this->declare_parameter("output_topic", "transformed_points");
        this->declare_parameter("target_frame", "motor_base");
        this->declare_parameter("source_frame", "hesai_lidar");
        this->declare_parameter("timeout_seconds", 0.01);
        this->declare_parameter("queue_size", 10);

        // パラメータの取得
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        source_frame_ = this->get_parameter("source_frame").as_string();
        timeout_seconds_ = this->get_parameter("timeout_seconds").as_double();
        int queue_size = this->get_parameter("queue_size").as_int();

        // パブリッシャとサブスクライバの作成
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, queue_size);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, queue_size,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointCloud Transformer initialized:");
        RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Transform: %s -> %s", source_frame_.c_str(), target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Timeout: %.3f seconds", timeout_seconds_);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 transformed_points;
        try
        {
            // tf2を利用して変換（常に最新のTFを使う）
            tf_buffer_.lookupTransform(target_frame_, source_frame_, rclcpp::Time(0),
                                       rclcpp::Duration::from_seconds(timeout_seconds_));

            // pcl_ros::transformPointCloudを利用してPointCloudを変換
            pcl_ros::transformPointCloud(target_frame_, *msg, transformed_points, tf_buffer_);

            RCLCPP_DEBUG(this->get_logger(), "PointCloud transformed from %s to %s.",
                         source_frame_.c_str(), target_frame_.c_str());
            pub_->publish(transformed_points);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform failed from %s to %s: %s",
                         source_frame_.c_str(), target_frame_.c_str(), ex.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // パラメータ用メンバ変数
    std::string target_frame_;
    std::string source_frame_;
    double timeout_seconds_;
};

// コンポーネントの登録
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudTransformer)
