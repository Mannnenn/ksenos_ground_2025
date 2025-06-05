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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_ros/transforms.hpp"

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer()
        : Node("pointcloud_transform_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // パブリッシャ: 変換後のPointCloudを"transformed_points"トピックへ配信
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_points", 10);
        // サブスクライバ: hesai_lidarフレームで取得したlidat_pointsを受信（"lidat_points"トピック）
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_points", 10,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 transformed_points;
        try
        {
            // tf2を利用して変換
            // ここではmsg->header.stampを用いてtf2の値を取得。タイムアウトを0.01秒に設定
            tf_buffer_.lookupTransform("motor_base", "hesai_lidar", msg->header.stamp, rclcpp::Duration::from_seconds(0.01));

            // pcl_ros::transformPointCloudを利用してPointCloudを変換
            pcl_ros::transformPointCloud("motor_base", *msg, transformed_points, tf_buffer_);

            RCLCPP_INFO(this->get_logger(), "PointCloud transformed from hesai_lidar to motor_base.");
            pub_->publish(transformed_points);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}