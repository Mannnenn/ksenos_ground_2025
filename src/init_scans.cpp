#include "rclcpp/rclcpp.hpp"
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
    LidarScanNode()
        : Node("lidar_scan_node"),
          scan_duration_(10.0),
          lower_limit_rad_(-M_PI / 2.0),
          upper_limit_rad_(M_PI / 2.0),
          init_angle_rad_(M_PI / 12.0f)
    {
        // publisher: LiDARの角度指定用
        angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_pitch_angle", 10);
        // subscriber: transformed_points
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "transformed_points", 10,
            std::bind(&LidarScanNode::pointCloudCallback, this, std::placeholders::_1));
        // publisher: フィルタ後のポイントクラウド出力用
        filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);

        start_time_ = this->now();
        // scan期間中に定期的に角度制御を行うタイマー
        timer_ = this->create_wall_timer(100ms, std::bind(&LidarScanNode::timerCallback, this));

        // accumulated cloudの初期化
        acc_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        header_set_ = false;

        RCLCPP_INFO(this->get_logger(), "LidarScanNode started");
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
        RCLCPP_INFO(this->get_logger(), "Accumulated %zu points, total: %zu",
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
        vg.setLeafSize(0.15f, 0.15f, 0.15f);
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
            RCLCPP_INFO(this->get_logger(), "Publishing target pitch angle: %.2f", angle_msg.data);
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
            // タイマー停止・ノード終了
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Scan finished. Shutting down node.");
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    const double scan_duration_;
    const float lower_limit_rad_;
    const float upper_limit_rad_;
    const float init_angle_rad_;

    // vectorによる蓄積を廃止し、単一の累積クラウドを使用
    pcl::PointCloud<pcl::PointXYZ>::Ptr acc_cloud_;
    bool header_set_;
    std_msgs::msg::Header cloud_header_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarScanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}