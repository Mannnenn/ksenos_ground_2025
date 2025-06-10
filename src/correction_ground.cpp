#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class GroundCorrectionNode : public rclcpp::Node
{
public:
    GroundCorrectionNode() : Node("ground_correction_node")
    {
        // パラメータの宣言と取得
        this->declare_parameter("input_topic", "/input_pointcloud");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("lidar_frame", "lidar");
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("plane_distance_threshold", 0.02);
        this->declare_parameter("max_iterations", 1000);
        this->declare_parameter("min_inliers", 1000);
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("statistical_filter_mean_k", 50);
        this->declare_parameter("statistical_filter_stddev", 1.0);

        input_topic_ = this->get_parameter("input_topic").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        lidar_frame_ = this->get_parameter("lidar_frame").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        plane_distance_threshold_ = this->get_parameter("plane_distance_threshold").as_double();
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        min_inliers_ = static_cast<std::size_t>(this->get_parameter("min_inliers").as_int());
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        statistical_filter_mean_k_ = this->get_parameter("statistical_filter_mean_k").as_int();
        statistical_filter_stddev_ = this->get_parameter("statistical_filter_stddev").as_double();

        // サブスクライバーとパブリッシャーの初期化
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&GroundCorrectionNode::pointcloudCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // TF配信用タイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&GroundCorrectionNode::publishTransform, this));

        RCLCPP_INFO(this->get_logger(), "Ground correction node parameters:");
        RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Base frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  LiDAR frame: %s", lidar_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Voxel size: %.3f", voxel_size_);
        RCLCPP_INFO(this->get_logger(), "  Plane distance threshold: %.3f", plane_distance_threshold_);
        RCLCPP_INFO(this->get_logger(), "  Max iterations: %d", max_iterations_);
        RCLCPP_INFO(this->get_logger(), "  Min inliers: %zu", min_inliers_);
        RCLCPP_INFO(this->get_logger(), "  Publish rate: %.2f Hz", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "  Statistical filter mean K: %d", statistical_filter_mean_k_);
        RCLCPP_INFO(this->get_logger(), "  Statistical filter stddev: %.2f", statistical_filter_stddev_);
        RCLCPP_INFO(this->get_logger(), "  Transform will be published at %.2f Hz", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "  Transform frame IDs: %s -> %s", base_frame_.c_str(), lidar_frame_.c_str());
        // 初期化完了メッセージ

        RCLCPP_INFO(this->get_logger(), "Ground correction node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Base frame: %s, LiDAR frame: %s", base_frame_.c_str(), lidar_frame_.c_str());
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            // PCLに変換
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);

            if (cloud->empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
                return;
            }

            // 前処理：ボクセルフィルタ
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(cloud);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*cloud_filtered);

            // 外れ値除去
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_denoised(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_filtered);
            sor.setMeanK(statistical_filter_mean_k_);
            sor.setStddevMulThresh(statistical_filter_stddev_);
            sor.filter(*cloud_denoised);

            // 平面検出
            if (estimateGroundPlane(cloud_denoised))
            {
                last_valid_timestamp_ = msg->header.stamp;
                RCLCPP_DEBUG(this->get_logger(), "Ground plane estimation successful");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to estimate ground plane");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
        }
    }

    bool estimateGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        if (cloud->size() < min_inliers_)
        {
            RCLCPP_WARN(this->get_logger(), "Not enough points for plane estimation");
            return false;
        }

        // RANSACによる平面推定
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations_);
        seg.setDistanceThreshold(plane_distance_threshold_);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < min_inliers_)
        {
            RCLCPP_WARN(this->get_logger(), "Not enough inliers for ground plane: %zu", inliers->indices.size());
            return false;
        }

        // 平面の法線ベクトル
        Eigen::Vector3d plane_normal(
            coefficients->values[0],
            coefficients->values[1],
            coefficients->values[2]);

        // 法線を正規化
        plane_normal.normalize();

        // 法線がZ軸負方向を向くように調整（床面の法線は上向き）
        if (plane_normal.z() < 0)
        {
            plane_normal = -plane_normal;
            coefficients->values[3] = -coefficients->values[3];
        }

        // 平面からの距離（床面の高さ）
        double ground_height = std::abs(coefficients->values[3]);

        // Z軸（垂直軸）との角度からロール・ピッチを計算
        Eigen::Vector3d z_axis(0, 0, 1);

        // 法線ベクトルからロール・ピッチを計算
        // ピッチ：XZ平面での傾き
        double pitch = std::asin(-plane_normal.x());

        // ロール：YZ平面での傾き
        double roll = std::atan2(plane_normal.y(), plane_normal.z());

        // クォータニオンを作成（ZYX順序）
        tf2::Quaternion q;
        q.setRPY(roll, pitch, 0.0);

        // 変換行列を更新
        std::lock_guard<std::mutex> lock(transform_mutex_);
        transform_.header.frame_id = base_frame_;
        transform_.child_frame_id = lidar_frame_;
        transform_.transform.translation.x = 0.0;
        transform_.transform.translation.y = 0.0;
        transform_.transform.translation.z = ground_height;
        transform_.transform.rotation.x = q.x();
        transform_.transform.rotation.y = q.y();
        transform_.transform.rotation.z = q.z();
        transform_.transform.rotation.w = q.w();

        has_valid_transform_ = true;

        RCLCPP_DEBUG(this->get_logger(),
                     "Ground plane: height=%.3f, roll=%.3f°, pitch=%.3f°, inliers=%zu",
                     ground_height, roll * 180.0 / M_PI, pitch * 180.0 / M_PI, inliers->indices.size());

        return true;
    }

    void publishTransform()
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);

        if (!has_valid_transform_)
        {
            return;
        }

        // タイムスタンプを更新
        transform_.header.stamp = last_valid_timestamp_.nanoseconds() == 0 ? this->get_clock()->now() : last_valid_timestamp_;

        try
        {
            tf_broadcaster_->sendTransform(transform_);
            RCLCPP_DEBUG_ONCE(this->get_logger(), "Publishing TF transform");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish transform: %s", e.what());
        }
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string input_topic_;
    std::string base_frame_;
    std::string lidar_frame_;
    double voxel_size_;
    double plane_distance_threshold_;
    int max_iterations_;
    std::size_t min_inliers_;
    double publish_rate_;
    int statistical_filter_mean_k_;
    double statistical_filter_stddev_;

    geometry_msgs::msg::TransformStamped transform_;
    std::mutex transform_mutex_;
    bool has_valid_transform_ = false;
    rclcpp::Time last_valid_timestamp_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundCorrectionNode>();

    RCLCPP_INFO(node->get_logger(), "Starting ground correction node...");

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Node crashed: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}