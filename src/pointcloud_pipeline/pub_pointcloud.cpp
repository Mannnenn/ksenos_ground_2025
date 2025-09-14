#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <chrono>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
// #define PRINT_FLAG

int frameItem = 0;

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("pointcloud_publisher", options)
    {
        // PointCloud2パブリッシャーを作成
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 10);

        // LiDARの初期化
        pandar_general_ = std::make_unique<PandarGeneralSDK>(
            std::string("192.168.1.201"), 2368, 0, 10110,
            std::bind(&PointCloudPublisher::lidarCallbackWrapper, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&PointCloudPublisher::lidarAlgorithmCallbackWrapper, this,
                      std::placeholders::_1),
            std::bind(&PointCloudPublisher::gpsCallbackWrapper, this,
                      std::placeholders::_1),
            0, 0, 0,
            std::string("Pandar40P"), std::string("frame_id"),
            "", "", "", false);

        pandar_general_->Start();

        RCLCPP_INFO(this->get_logger(), "PointCloud Publisher node has been started.");
    }

    ~PointCloudPublisher()
    {
        if (pandar_general_)
        {
            // LiDARの停止処理があれば実行
        }
    }

    void publishPointCloud(const std::vector<pcl::PointXYZI> &points, double timestamp)
    {
        // PCLポイントクラウドを作成
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // 点群データをPCLに設定
        cloud->points.assign(points.begin(), points.end());
        cloud->width = points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        // ROS2メッセージに変換
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);

        // ヘッダー情報を設定
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "hesai_lidar";

        // パブリッシュ
        publisher_->publish(cloud_msg);
    }

    void gpsCallbackWrapper(int timestamp)
    {
        // 空実装
    }

    void lidarCallbackWrapper(boost::shared_ptr<PPointCloud> cld, double timestamp)
    {

        // SDK の点群データ構造体から PCL PointXYZI 形式に変換
        std::vector<pcl::PointXYZI> points;
        for (const auto &pt : cld->points)
        {
            pcl::PointXYZI p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity; // 反射強度を設定
            points.push_back(p);
        }

        // ROS2でポイントクラウドをパブリッシュ
        publishPointCloud(points, timestamp);
    }

    void lidarAlgorithmCallbackWrapper(HS_Object3D_Object_List *object_t)
    {
        HS_Object3D_Object *object;
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::unique_ptr<PandarGeneralSDK> pandar_general_;
};

// コンポーネントの登録
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudPublisher)
