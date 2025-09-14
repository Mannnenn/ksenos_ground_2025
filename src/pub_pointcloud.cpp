#include <rclcpp/rclcpp.hpp>
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
    PointCloudPublisher() : Node("pointcloud_publisher")
    {
        // PointCloud2パブリッシャーを作成
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloud Publisher node has been started.");
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

#ifdef PRINT_FLAG
        RCLCPP_INFO(this->get_logger(), "Published pointcloud with %zu points at timestamp: %lf",
                    points.size(), timestamp);
#endif
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

// グローバルなパブリッシャーノードのポインタ
std::shared_ptr<PointCloudPublisher> g_publisher_node = nullptr;

void gpsCallback(int timestamp)
{
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
#ifdef PRINT_FLAG
    printf("timestamp: %lf, point_size: %ld\n", timestamp, cld->points.size());
#endif

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
    if (g_publisher_node != nullptr)
    {
        g_publisher_node->publishPointCloud(points, timestamp);
    }
}

void lidarAlgorithmCallback(HS_Object3D_Object_List *object_t)
{
    HS_Object3D_Object *object;
#ifdef PRINT_FLAG
    printf("----------------------\n");
    printf("total objects: %d\n", object_t->valid_size);
    for (size_t i = 0; i < object_t->valid_size; i++)
    {
        object = &object_t->data[i];
        printf("id: %u, type: %u\n", object->data.id, object->type);
    }
    printf("----------------------\n");
#endif
}

int main(int argc, char **argv)
{
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // パブリッシャーノードを作成
    g_publisher_node = std::make_shared<PointCloudPublisher>();

    // LiDARの初期化
    PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 0, 10110,
                                   lidarCallback, lidarAlgorithmCallback, gpsCallback, 0, 0, 0,
                                   std::string("Pandar40P"), std::string("frame_id"),
                                   "", "", "", false);
    pandarGeneral.Start();

    // ROS2のスピンループを別スレッドで実行
    std::thread spin_thread([&]()
                            { rclcpp::spin(g_publisher_node); });

    // メインループ（LiDARデータの取得を継続）
    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 終了処理
    rclcpp::shutdown();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    return 0;
}
