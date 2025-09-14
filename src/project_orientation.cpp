#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class TfProjectionNode : public rclcpp::Node
{
private:
    // タイマー間隔とスリープ時間の定数
    static constexpr int TIMER_INTERVAL_MS = 10;
    static constexpr int WAIT_SLEEP_MS = 1000;

public:
    TfProjectionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("tf_projection_node", options)
    {
        initializeTransforms();
        initializeTimer();
    }

private:
    void initializeTransforms()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // base_linkが利用可能になるまで待機
        waitForTransform("odom", "imu_link");
    }

    void waitForTransform(const std::string &target_frame, const std::string &source_frame)
    {
        while (rclcpp::ok())
        {
            try
            {
                tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "%s is now available.", source_frame.c_str());
                break;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for %s to become available: %s",
                            source_frame.c_str(), ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(WAIT_SLEEP_MS));
            }
        }
    }

    void initializeTimer()
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(TIMER_INTERVAL_MS),
            std::bind(&TfProjectionNode::timer_callback, this));
    }
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        if (!getTransform("odom", "imu_link", transform_stamped))
        {
            return;
        }

        // 回転を取得
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);

        tf2::Quaternion projected_xy_plane = projectToXYPlane(q);

        publishProjectedTransform(transform_stamped, projected_xy_plane);
    }

    bool getTransform(const std::string &target_frame, const std::string &source_frame,
                      geometry_msgs::msg::TransformStamped &transform_stamped)
    {
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
            return false;
        }
    }

    void publishProjectedTransform(const geometry_msgs::msg::TransformStamped &original_transform,
                                   const tf2::Quaternion &projected_orientation)
    {
        geometry_msgs::msg::TransformStamped transform_stamped_base;
        transform_stamped_base.header.stamp = this->now();
        transform_stamped_base.header.frame_id = "odom";
        transform_stamped_base.child_frame_id = "imu_link_projected";
        transform_stamped_base.transform.rotation.x = projected_orientation.x();
        transform_stamped_base.transform.rotation.y = projected_orientation.y();
        transform_stamped_base.transform.rotation.z = projected_orientation.z();
        transform_stamped_base.transform.rotation.w = projected_orientation.w();
        transform_stamped_base.transform.translation.x = original_transform.transform.translation.x;
        transform_stamped_base.transform.translation.y = original_transform.transform.translation.y;
        transform_stamped_base.transform.translation.z = original_transform.transform.translation.z;

        // パブリッシュ
        tf_broadcaster_->sendTransform(transform_stamped_base);
    }

    tf2::Quaternion projectToXYPlane(const tf2::Quaternion &orientation)
    {
        // クォータニオンから回転行列を取得
        tf2::Matrix3x3 rotation_matrix(orientation);

        // x軸とy軸を取得
        tf2::Vector3 x_axis = rotation_matrix.getColumn(0); // x軸
        tf2::Vector3 y_axis = rotation_matrix.getColumn(1); // y軸

        // x軸とy軸をxy平面に投影（z成分を0にする）
        x_axis.setZ(0.0);
        y_axis.setZ(0.0);

        // 投影後のベクトルを正規化
        x_axis.normalize();
        y_axis.normalize();

        // 新しいz軸を計算（クロスプロダクト）
        tf2::Vector3 z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // 新しい回転行列を作成
        tf2::Matrix3x3 new_rotation_matrix(
            x_axis.getX(), y_axis.getX(), z_axis.getX(),
            x_axis.getY(), y_axis.getY(), z_axis.getY(),
            x_axis.getZ(), y_axis.getZ(), z_axis.getZ());

        // 新しい回転行列からクォータニオンを生成
        tf2::Quaternion projected_xy_plane;
        new_rotation_matrix.getRotation(projected_xy_plane);

        RCLCPP_DEBUG(this->get_logger(), "Projected IMU orientation: (%f, %f, %f, %f)",
                     projected_xy_plane.x(), projected_xy_plane.y(),
                     projected_xy_plane.z(), projected_xy_plane.w());

        return projected_xy_plane;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

// コンポーネントの登録
RCLCPP_COMPONENTS_REGISTER_NODE(TfProjectionNode)
