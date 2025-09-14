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
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class LidarScanNode : public rclcpp::Node
{
public:
    LidarScanNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("lidar_scan_node", options)
    {
        // パラメータの宣言とデフォルト値の設定
        this->declare_parameter<double>("scan_duration", 10.0);
        this->declare_parameter<double>("lower_limit_rad", -M_PI / 2.0);
        this->declare_parameter<double>("upper_limit_rad", M_PI / 2.0);
        this->declare_parameter<double>("init_angle_rad", 0.0);
        this->declare_parameter<double>("voxel_leaf_size", 0.1);
        this->declare_parameter<int>("timer_period_ms", 100);
        // トピック名のパラメータを追加
        this->declare_parameter<std::string>("angle_topic", "target_pitch_angle");
        this->declare_parameter<std::string>("input_pointcloud_topic", "transformed_points");
        this->declare_parameter<std::string>("output_pointcloud_topic", "map_points");

        // パラメータの取得
        scan_duration_ = this->get_parameter("scan_duration").as_double();
        lower_limit_rad_ = this->get_parameter("lower_limit_rad").as_double();
        upper_limit_rad_ = this->get_parameter("upper_limit_rad").as_double();
        init_angle_rad_ = this->get_parameter("init_angle_rad").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        int timer_period_ms = this->get_parameter("timer_period_ms").as_int();
        std::string angle_topic = this->get_parameter("angle_topic").as_string();
        std::string input_pointcloud_topic = this->get_parameter("input_pointcloud_topic").as_string();
        std::string output_pointcloud_topic = this->get_parameter("output_pointcloud_topic").as_string();

        // publisher: LiDARの角度指定用
        angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(angle_topic, 10);
        // subscriber: transformed_points
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_pointcloud_topic, 10,
            std::bind(&LidarScanNode::pointCloudCallback, this, std::placeholders::_1));

        // publisher: フィルタ後のポイントクラウド出力用 (信頼性の高いQoSプロファイルを使用)
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);
        filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_pointcloud_topic, qos);

        start_time_ = this->now();
        // scan期間中に定期的に角度制御を行うタイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&LidarScanNode::timerCallback, this));

        // accumulated cloudの初期化
        acc_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        header_set_ = false;

        RCLCPP_INFO(this->get_logger(), "LidarScanNode started with parameters:");
        RCLCPP_INFO(this->get_logger(), "  scan_duration: %.2f", scan_duration_);
        RCLCPP_INFO(this->get_logger(), "  lower_limit_rad: %.2f", lower_limit_rad_);
        RCLCPP_INFO(this->get_logger(), "  upper_limit_rad: %.2f", upper_limit_rad_);
        RCLCPP_INFO(this->get_logger(), "  init_angle_rad: %.2f", init_angle_rad_);
        RCLCPP_INFO(this->get_logger(), "  voxel_leaf_size: %.2f", voxel_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "  angle_topic: %s", angle_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  input_pointcloud_topic: %s", input_pointcloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  output_pointcloud_topic: %s", output_pointcloud_topic.c_str());
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // sensor_msgs::msg::PointCloud2をpcl::PointCloudに変換
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 最初のメッセージからheaderを保存
        if (!header_set_)
        {
            cloud_header_ = msg->header;
            header_set_ = true;
        }

        // 累積: 新たに受信したcloudの全ポイントを追加
        *acc_cloud_ += *cloud;
        RCLCPP_DEBUG(this->get_logger(), "Accumulated %zu points, total: %zu",
                     cloud->points.size(), acc_cloud_->points.size());
    }

    // 蓄積したcloudにフィルタを適用
    sensor_msgs::msg::PointCloud2::SharedPtr filterPointCloud()
    {
        if (acc_cloud_->empty())
        {
            return nullptr;
        }

        // VoxelGridフィルタの適用
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(acc_cloud_);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        vg.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*cloud_filtered, *filtered_msg);
        // 蓄積時に保存したheaderを設定
        if (header_set_)
        {
            filtered_msg->header = cloud_header_;
        }
        return filtered_msg;
    }

    void timerCallback()
    {
        auto elapsed = (this->now() - start_time_).seconds();
        if (elapsed < scan_duration_)
        {
            // 時間に応じてLiDARのターゲットピッチ角を線形補間
            float t = static_cast<float>(elapsed / scan_duration_);
            std_msgs::msg::Float32 angle_msg;
            angle_msg.data = lower_limit_rad_ + t * (upper_limit_rad_ - lower_limit_rad_);
            angle_pub_->publish(angle_msg);
            RCLCPP_DEBUG(this->get_logger(), "Publishing target pitch angle: %.2f", angle_msg.data);
        }
        else
        {
            // スキャン終了後モーターを初期位置に戻す
            // 一定時間待機
            rclcpp::sleep_for(std::chrono::seconds(1));

            std_msgs::msg::Float32 reset_angle_msg;
            reset_angle_msg.data = init_angle_rad_;
            angle_pub_->publish(reset_angle_msg);
            RCLCPP_INFO(this->get_logger(), "Scan duration reached. Resetting angle to: %.2f", reset_angle_msg.data);

            // スキャン終了: 蓄積したポイントクラウドにフィルタを適用しパブリッシュ
            auto filtered = filterPointCloud();
            if (filtered)
            {
                filtered_pub_->publish(*filtered);
                RCLCPP_INFO(this->get_logger(), "Published filtered pointcloud");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No pointcloud data received");
            }
            // タイマー停止
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Scan finished. Node remains active.");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    double scan_duration_;
    double lower_limit_rad_;
    double upper_limit_rad_;
    double init_angle_rad_;
    double voxel_leaf_size_;

    // vectorによる蓄積を廃止し、単一の累積クラウドを使用
    pcl::PointCloud<pcl::PointXYZ>::Ptr acc_cloud_;
    bool header_set_;
    std_msgs::msg::Header cloud_header_;
};

// コンポーネントの登録
RCLCPP_COMPONENTS_REGISTER_NODE(LidarScanNode)
