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

/**
 * @file detect_movable_objects.cpp
 * @brief 移動物体検出ノード
 *
 * 地図データとリアルタイムスキャンデータを比較して移動物体を検出し、
 * 追跡してTFフレームとして発行する。最大クラスタの仰角も計算する。
 */

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/octree/octree_pointcloud_changedetector.h"
#include "pcl/segmentation/extract_clusters.h"

// 定数定義
namespace
{
    constexpr double TF_TIMEOUT_SECONDS = 1.0;
    constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;
    constexpr double RADIANS_TO_DEGREES = 180.0 / M_PI;
}

/**
 * @brief 球面座標系の座標表現
 */
struct SphericalCoordinates
{
    double range;     // 距離 [m]
    double azimuth;   // 方位角 [rad] (-π to π)
    double elevation; // 仰角 [rad] (-π/2 to π/2)
};

/**
 * @brief 追跡対象物体の情報構造体
 */
struct TrackedObject
{
    int id;
    double x, y, z;         // 現在位置
    rclcpp::Time last_seen; // 最後に見つかった時刻
    bool is_active;         // アクティブかどうか
    size_t cluster_size;    // クラスタのサイズ（点数）

    TrackedObject() : id(0), x(0.0), y(0.0), z(0.0), last_seen(rclcpp::Time(0)), is_active(false), cluster_size(0) {}

    TrackedObject(int obj_id, double pos_x, double pos_y, double pos_z, rclcpp::Time time, size_t size)
        : id(obj_id), x(pos_x), y(pos_y), z(pos_z), last_seen(time), is_active(true), cluster_size(size) {}
};

/**
 * @brief 移動物体検出ノードクラス
 *
 * 地図データとリアルタイムスキャンをOctreeで比較し、
 * クラスタリングによって移動物体を検出・追跡する。
 */
class MovableObjectDetector : public rclcpp::Node
{
public:
    MovableObjectDetector() : Node("movable_object_detector")
    {
        // トピック関連パラメータの宣言
        this->declare_parameter("map_topic", "/map_points");
        this->declare_parameter("scan_topic", "/transformed_points");
        this->declare_parameter("output_topic", "/detected_objects");
        this->declare_parameter("elevation_topic", "/target_pitch_angle");

        // フレーム関連パラメータの宣言
        this->declare_parameter("lidar_frame", "lidar_center");
        this->declare_parameter("base_frame", "motor_base");
        this->declare_parameter("object_frame_prefix", "movable_object_");

        // 点群処理パラメータの宣言
        this->declare_parameter("octree_resolution", 1.5);
        this->declare_parameter("voxel_leaf_size", 0.025);

        // クラスタリングパラメータの宣言
        this->declare_parameter("cluster_tolerance", 0.25);
        this->declare_parameter("min_cluster_size", 10);
        this->declare_parameter("max_cluster_size", 250);

        // 物体追跡パラメータの宣言
        this->declare_parameter("object_timeout", 5.0);           // 物体が見えなくなってからのタイムアウト[秒]
        this->declare_parameter("max_association_distance", 5.0); // 物体関連付けの最大距離[m]
        this->declare_parameter("min_tf_cluster_size", 10);       // TF発行する最小クラスタサイズ

        // パラメータの取得
        map_topic_ = this->get_parameter("map_topic").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        elevation_topic_ = this->get_parameter("elevation_topic").as_string();
        lidar_frame_ = this->get_parameter("lidar_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        object_frame_prefix_ = this->get_parameter("object_frame_prefix").as_string();
        octree_resolution_ = this->get_parameter("octree_resolution").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        object_timeout_ = this->get_parameter("object_timeout").as_double();
        max_association_distance_ = this->get_parameter("max_association_distance").as_double();
        min_tf_cluster_size_ = this->get_parameter("min_tf_cluster_size").as_int();

        // 初期化
        next_object_id_ = 0;

        // TFバッファとリスナーの初期化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // パブリッシャーとTFブロードキャスターの作成
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        elevation_publisher_ = this->create_publisher<std_msgs::msg::Float32>(elevation_topic_, 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // パラメータ値をログ出力
        RCLCPP_INFO(this->get_logger(), "MovableObjectDetector initialized with:");
        RCLCPP_INFO(this->get_logger(), "  Map topic: %s", map_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Scan topic: %s", scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Elevation topic: %s", elevation_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Lidar frame: %s", lidar_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Base frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Object frame prefix: %s", object_frame_prefix_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Octree resolution: %.2f m", octree_resolution_);
        RCLCPP_INFO(this->get_logger(), "  Voxel leaf size: %.3f m", voxel_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "  Cluster tolerance: %.2f m", cluster_tolerance_);
        RCLCPP_INFO(this->get_logger(), "  Min cluster size: %d points", min_cluster_size_);
        RCLCPP_INFO(this->get_logger(), "  Max cluster size: %d points", max_cluster_size_);
        RCLCPP_INFO(this->get_logger(), "  Object timeout: %.2f seconds", object_timeout_);
        RCLCPP_INFO(this->get_logger(), "  Max association distance: %.2f m", max_association_distance_);
        RCLCPP_INFO(this->get_logger(), "  Min TF cluster size: %d points", min_tf_cluster_size_);
        RCLCPP_INFO(this->get_logger(), "  Next object ID: %d", next_object_id_);

        // 地図データ用の一回限りのサブスクライバー
        map_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic_, 1,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {
                // 地図データの受信と保存
                pcl::fromROSMsg(*msg, *map_cloud_);
                RCLCPP_INFO(this->get_logger(), "Map point cloud received with %zu points", map_cloud_->size());

                // 地図を受信したら、継続的なスキャンデータのサブスクライブを開始
                map_subscription_.reset();

                scan_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    scan_topic_, 10,
                    std::bind(&MovableObjectDetector::scanCallback, this, std::placeholders::_1));

                RCLCPP_INFO(this->get_logger(), "Started subscribing to scan data on: %s", scan_topic_.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "Waiting for map data on: %s", map_topic_.c_str());
    }

private:
    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 地図データの存在確認
        if (map_cloud_->empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Map cloud is empty, skipping scan processing");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *scan_cloud);

        // scan_cloudから高さ10m以下の点のみを抽出
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &pt : scan_cloud->points)
        {
            if (pt.z <= 10.0)
            {
                filtered_scan_cloud->points.push_back(pt);
            }
        }
        filtered_scan_cloud->width = filtered_scan_cloud->points.size();
        filtered_scan_cloud->height = 1;
        filtered_scan_cloud->is_dense = true;
        scan_cloud = filtered_scan_cloud;

        if (scan_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty scan cloud");
            return;
        }

        // 特に近い距離で検出したときに点群の数が多くなりすぎるのを避けるためにダウンサンプリング
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(scan_cloud);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(*downsampled_cloud);
        scan_cloud = downsampled_cloud;

        RCLCPP_DEBUG(this->get_logger(), "Scan received with %zu points (after downsampling: %zu)",
                     static_cast<size_t>(msg->width * msg->height), scan_cloud->size());

        // Octreeを使って差分を抽出
        pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extractMapDifferenceOctree(scan_cloud, diff_cloud);

        // クラスタリングして移動物体を検出し、TFをパブリッシュ
        pcl::PointCloud<pcl::PointXYZ>::Ptr movable_objects(new pcl::PointCloud<pcl::PointXYZ>);
        clusterPointsAndPublishTF(diff_cloud, movable_objects, msg->header);

        // 結果をパブリッシュ
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*movable_objects, output_msg);
        output_msg.header = msg->header;

        publisher_->publish(output_msg);
    }

    /**
     * @brief Octreeを使用して地図データとスキャンデータの差分を抽出
     * @param scan_cloud 入力スキャン点群
     * @param diff_cloud 出力差分点群（地図にない新しい点）
     */
    void extractMapDifferenceOctree(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &diff_cloud)
    {
        // 一時的なOctreeを作成してmap_cloud_を設定
        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> temp_octree(octree_resolution_);
        temp_octree.setInputCloud(map_cloud_);
        temp_octree.addPointsFromInputCloud();
        temp_octree.switchBuffers();

        // 新しいスキャンデータを設定
        temp_octree.setInputCloud(scan_cloud);
        temp_octree.addPointsFromInputCloud();

        // 変化検出を実行（map_cloud_にない新しい点を取得）
        std::vector<int> newPointIdxVector;
        temp_octree.getPointIndicesFromNewVoxels(newPointIdxVector);

        // 新しい点（地図にない点）を抽出
        for (const auto &idx : newPointIdxVector)
        {
            diff_cloud->points.push_back(scan_cloud->points[idx]);
        }

        diff_cloud->width = diff_cloud->points.size();
        diff_cloud->height = 1;
        diff_cloud->is_dense = true;

        RCLCPP_DEBUG(this->get_logger(), "Extracted %zu points as map difference using Octree", diff_cloud->size());
    }

    /**
     * @brief デカルト座標を球面座標に変換
     * @param x X座標
     * @param y Y座標
     * @param z Z座標
     * @return 球面座標構造体
     */
    SphericalCoordinates cartesianToSpherical(double x, double y, double z)
    {
        SphericalCoordinates spherical;

        // 距離（range）
        spherical.range = std::sqrt(x * x + y * y + z * z);

        // 方位角（azimuth）: XY平面での角度
        spherical.azimuth = std::atan2(y, x);

        // 仰角（elevation）: Z軸との角度
        double horizontal_distance = std::sqrt(x * x + y * y);
        spherical.elevation = std::atan2(z, horizontal_distance);

        return spherical;
    }

    /**
     * @brief 点群をクラスタリングしてTFを発行
     * @param cloud 入力点群
     * @param clustered_cloud 出力クラスタ点群
     * @param header メッセージヘッダー
     */
    void clusterPointsAndPublishTF(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &clustered_cloud,
                                   const std_msgs::msg::Header &header)
    {
        if (cloud->points.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "No points to cluster");
            // 非アクティブ物体のクリーンアップのみ実行
            cleanupInactiveObjects(header.stamp);
            return;
        }

        // kd-treeの作成
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // クラスタリングの実行
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        RCLCPP_DEBUG(this->get_logger(), "Found %zu clusters", cluster_indices.size()); // 現在のフレームで見つかったクラスタの重心リスト
        std::vector<std::tuple<double, double, double, size_t>> current_centroids;

        // 各クラスタの重心を計算し、サイズチェックを行う
        size_t filtered_clusters = 0;
        for (const auto &indices : cluster_indices)
        {
            // バウンディングボックスの計算
            double min_x = std::numeric_limits<double>::max();
            double min_y = std::numeric_limits<double>::max();
            double min_z = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double max_y = std::numeric_limits<double>::lowest();
            double max_z = std::numeric_limits<double>::lowest();

            double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            for (const auto &index : indices.indices)
            {
                const auto &point = cloud->points[index];
                sum_x += point.x;
                sum_y += point.y;
                sum_z += point.z;

                // バウンディングボックスの更新
                min_x = std::min(min_x, static_cast<double>(point.x));
                min_y = std::min(min_y, static_cast<double>(point.y));
                min_z = std::min(min_z, static_cast<double>(point.z));
                max_x = std::max(max_x, static_cast<double>(point.x));
                max_y = std::max(max_y, static_cast<double>(point.y));
                max_z = std::max(max_z, static_cast<double>(point.z));
            }

            // バウンディングボックスのサイズを計算
            double size_x = max_x - min_x;
            double size_y = max_y - min_y;
            double size_z = max_z - min_z;

            // 1.5m四方の立体に収まらない場合は除去
            constexpr double MAX_CLUSTER_SIZE = 1.5;
            if (size_x > MAX_CLUSTER_SIZE || size_y > MAX_CLUSTER_SIZE || size_z > MAX_CLUSTER_SIZE)
            {
                RCLCPP_DEBUG(this->get_logger(), "Cluster filtered out due to size: %.2fx%.2fx%.2f m", size_x, size_y, size_z);
                filtered_clusters++;
                continue;
            }

            // サイズチェックを通過したクラスタのみ処理
            for (const auto &index : indices.indices)
            {
                clustered_cloud->points.push_back(cloud->points[index]);
            }

            double centroid_x = sum_x / indices.indices.size();
            double centroid_y = sum_y / indices.indices.size();
            double centroid_z = sum_z / indices.indices.size();
            size_t cluster_size = indices.indices.size();

            current_centroids.emplace_back(centroid_x, centroid_y, centroid_z, cluster_size);
        }

        if (filtered_clusters > 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Filtered out %zu clusters due to size constraint", filtered_clusters);
        }

        // 物体の追跡と関連付け
        trackAndAssociateObjects(current_centroids, header);

        clustered_cloud->width = clustered_cloud->points.size();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;

        RCLCPP_DEBUG(this->get_logger(), "Found %zu clusters, tracking %zu objects, TF publishing %zu objects (min size: %d)",
                     cluster_indices.size(), tracked_objects_.size(),
                     std::count_if(tracked_objects_.begin(), tracked_objects_.end(),
                                   [this](const auto &pair)
                                   {
                                       return pair.second.is_active && pair.second.cluster_size >= static_cast<size_t>(min_tf_cluster_size_);
                                   }),
                     min_tf_cluster_size_);

        // 球座標計算と仰角の処理
        calculateAndPublishElevation(header);
    }

    /**
     * @brief 現在のクラスタと既存の追跡物体を関連付け
     * @param current_centroids 現在フレームのクラスタ重心リスト
     * @param header メッセージヘッダー
     */
    void trackAndAssociateObjects(const std::vector<std::tuple<double, double, double, size_t>> &current_centroids,
                                  const std_msgs::msg::Header &header)
    {
        rclcpp::Time current_time = rclcpp::Time(header.stamp);

        // 既存の物体をすべて非アクティブにマーク
        for (auto &obj : tracked_objects_)
        {
            obj.second.is_active = false;
        }

        // 各クラスタを既存の物体と関連付け
        for (const auto &centroid : current_centroids)
        {
            double x = std::get<0>(centroid);
            double y = std::get<1>(centroid);
            double z = std::get<2>(centroid);
            size_t cluster_size = std::get<3>(centroid);

            // 最も近い既存物体を探す
            int best_match_id = -1;
            double min_distance = max_association_distance_;

            for (auto &obj_pair : tracked_objects_)
            {
                auto &obj = obj_pair.second;
                double distance = std::sqrt(
                    std::pow(x - obj.x, 2) +
                    std::pow(y - obj.y, 2) +
                    std::pow(z - obj.z, 2));

                if (distance < min_distance)
                {
                    min_distance = distance;
                    best_match_id = obj.id;
                }
            }

            if (best_match_id != -1)
            {
                // 既存物体を更新
                auto &obj = tracked_objects_[best_match_id];
                obj.x = x;
                obj.y = y;
                obj.z = z;
                obj.last_seen = current_time;
                obj.is_active = true;
                obj.cluster_size = cluster_size;

                RCLCPP_DEBUG(this->get_logger(), "Updated object %d at (%.2f, %.2f, %.2f) with %zu points",
                             best_match_id, x, y, z, cluster_size);
            }
            else
            {
                // 新しい物体として登録 - emplace を使用
                tracked_objects_.emplace(next_object_id_, TrackedObject(next_object_id_, x, y, z, current_time, cluster_size));
                RCLCPP_INFO(this->get_logger(), "New object %d detected at (%.2f, %.2f, %.2f) with %zu points",
                            next_object_id_, x, y, z, cluster_size);
                next_object_id_++;
            }
        }

        // アクティブな物体のTFをパブリッシュ（最小クラスタサイズ以上の場合のみ）
        for (const auto &obj_pair : tracked_objects_)
        {
            const auto &obj = obj_pair.second;
            if (obj.is_active && obj.cluster_size >= static_cast<size_t>(min_tf_cluster_size_))
            {
                publishObjectTF(obj, header);
            }
        }

        // 非アクティブ物体のクリーンアップ
        cleanupInactiveObjects(current_time);
    }

    /**
     * @brief 追跡物体のTFを発行
     * @param obj 追跡物体
     * @param header メッセージヘッダー
     */
    void publishObjectTF(const TrackedObject &obj, const std_msgs::msg::Header &header)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header = header;
        transform_stamped.header.frame_id = base_frame_;
        transform_stamped.child_frame_id = object_frame_prefix_ + std::to_string(obj.id);

        transform_stamped.transform.translation.x = obj.x;
        transform_stamped.transform.translation.y = obj.y;
        transform_stamped.transform.translation.z = obj.z;

        // 回転は単位クォータニオン（回転なし）
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    /**
     * @brief タイムアウトした非アクティブ物体を削除
     * @param current_time 現在時刻
     */
    void cleanupInactiveObjects(rclcpp::Time current_time)
    {
        auto it = tracked_objects_.begin();
        while (it != tracked_objects_.end())
        {
            const auto &obj = it->second;
            double time_since_last_seen = (current_time - obj.last_seen).seconds();

            if (!obj.is_active && time_since_last_seen > object_timeout_)
            {
                RCLCPP_INFO(this->get_logger(), "Removing object %d (timeout: %.1fs)",
                            obj.id, time_since_last_seen);
                it = tracked_objects_.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    /**
     * @brief 最大クラスタの仰角を計算してパブリッシュ
     * @param header メッセージヘッダー
     */
    void calculateAndPublishElevation(const std_msgs::msg::Header &header)
    {
        // 最も点群数が多いアクティブなオブジェクトを見つける
        const TrackedObject *largest_object = nullptr;
        size_t max_cluster_size = 0;

        for (const auto &obj_pair : tracked_objects_)
        {
            const auto &obj = obj_pair.second;
            if (obj.is_active && obj.cluster_size > max_cluster_size)
            {
                max_cluster_size = obj.cluster_size;
                largest_object = &obj;
            }
        }

        if (largest_object == nullptr)
        {
            RCLCPP_DEBUG(this->get_logger(), "No active objects to track");
            return;
        }

        try
        {
            geometry_msgs::msg::TransformStamped lidar_transform =
                tf_buffer_->lookupTransform(lidar_frame_, header.frame_id, header.stamp,
                                            rclcpp::Duration::from_seconds(TF_TIMEOUT_SECONDS));

            geometry_msgs::msg::PointStamped point_in_map;
            point_in_map.header = header;
            point_in_map.point.x = largest_object->x;
            point_in_map.point.y = largest_object->y;
            point_in_map.point.z = largest_object->z;

            geometry_msgs::msg::PointStamped point_in_lidar;
            tf2::doTransform(point_in_map, point_in_lidar, lidar_transform);

            SphericalCoordinates spherical = cartesianToSpherical(
                point_in_lidar.point.x,
                point_in_lidar.point.y,
                point_in_lidar.point.z);

            // 最大クラスタの迎角をパブリッシュ
            std_msgs::msg::Float32 elevation_msg;
            elevation_msg.data = static_cast<float>(spherical.elevation) - 2 * M_PI * 3 / 360;
            elevation_publisher_->publish(elevation_msg);

            RCLCPP_INFO(this->get_logger(),
                        "Tracking largest object %d (%zu points) - Range=%.2fm, Azimuth=%.2f°, Elevation=%.2f°",
                        largest_object->id, largest_object->cluster_size,
                        spherical.range,
                        spherical.azimuth * RADIANS_TO_DEGREES,
                        spherical.elevation * RADIANS_TO_DEGREES);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform to %s: %s", lidar_frame_.c_str(), ex.what());
        }
    }

private:
    // トピック名とフレーム名
    std::string map_topic_;
    std::string scan_topic_;
    std::string output_topic_;
    std::string elevation_topic_;
    std::string lidar_frame_;
    std::string base_frame_;
    std::string object_frame_prefix_;

    // 点群処理パラメータ
    double octree_resolution_;
    double voxel_leaf_size_;

    // クラスタリングパラメータ
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

    // 物体追跡パラメータ
    double object_timeout_;
    double max_association_distance_;
    int min_tf_cluster_size_;
    int next_object_id_;
    std::map<int, TrackedObject> tracked_objects_;

    // 保存用の地図点群
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_{new pcl::PointCloud<pcl::PointXYZ>};

    // TF関連
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // サブスクライバー、パブリッシャー、TFブロードキャスター
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevation_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovableObjectDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}