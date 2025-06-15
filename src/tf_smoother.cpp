#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class TfSmoother : public rclcpp::Node
{
public:
    TfSmoother() : Node("tf_smoother")
    {
        // パラメータの宣言
        this->declare_parameter("source_frame", "map");
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("smoothed_target_frame", "base_link_smooth");
        this->declare_parameter("max_translation_threshold", 0.5); // [m]
        this->declare_parameter("max_rotation_threshold", 0.5);    // [rad]
        this->declare_parameter("timeout_duration", 1.0);          // [s]
        this->declare_parameter("publish_rate", 10.0);             // [Hz]
        this->declare_parameter("smoothing_factor", 0.1);          // 平滑化係数 (0.0-1.0)

        // パラメータの取得
        source_frame_ = this->get_parameter("source_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        smoothed_target_frame_ = this->get_parameter("smoothed_target_frame").as_string();
        max_translation_threshold_ = this->get_parameter("max_translation_threshold").as_double();
        max_rotation_threshold_ = this->get_parameter("max_rotation_threshold").as_double();
        timeout_duration_ = std::chrono::duration<double>(this->get_parameter("timeout_duration").as_double());
        smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();

        // TF2の初期化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // タイマーの初期化
        double publish_rate = this->get_parameter("publish_rate").as_double();
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(timer_period, std::bind(&TfSmoother::timer_callback, this));

        // 初期化フラグ
        is_initialized_ = false;
        last_update_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "TF Smoother initialized");
        RCLCPP_INFO(this->get_logger(), "Source frame: %s", source_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Smoothed target frame: %s", smoothed_target_frame_.c_str());
    }

private:
    void timer_callback()
    {
        try
        {
            // 現在の変換を取得
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                source_frame_, target_frame_, tf2::TimePointZero);

            auto current_time = this->get_clock()->now();

            // 初回の場合は現在の位置を初期位置として設定
            if (!is_initialized_)
            {
                previous_transform_ = transform_stamped;
                smoothed_transform_ = transform_stamped;
                smoothed_transform_.child_frame_id = smoothed_target_frame_;
                is_initialized_ = true;
                last_update_time_ = current_time;
                RCLCPP_INFO(this->get_logger(), "Initialized with current transform");
                broadcast_smoothed_transform();
                return;
            }

            // タイムアウトチェック
            auto time_since_last_update = current_time - last_update_time_;
            if (time_since_last_update > timeout_duration_)
            {
                RCLCPP_WARN(this->get_logger(), "Timeout detected, reinitializing with current transform");
                previous_transform_ = transform_stamped;
                smoothed_transform_ = transform_stamped;
                smoothed_transform_.child_frame_id = smoothed_target_frame_;
                last_update_time_ = current_time;
                broadcast_smoothed_transform();
                return;
            }

            // 変化量を計算
            double dx = transform_stamped.transform.translation.x - previous_transform_.transform.translation.x;
            double dy = transform_stamped.transform.translation.y - previous_transform_.transform.translation.y;
            double dz = transform_stamped.transform.translation.z - previous_transform_.transform.translation.z;
            double translation_diff = std::sqrt(dx * dx + dy * dy + dz * dz);

            // 回転の変化量を計算（クォータニオンの角度差）
            double rotation_diff = calculate_rotation_difference(
                previous_transform_.transform.rotation,
                transform_stamped.transform.rotation);

            // 閾値チェック
            if (translation_diff > max_translation_threshold_ || rotation_diff > max_rotation_threshold_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Large movement detected: translation=%.3f[m], rotation=%.3f[rad] - ignoring",
                            translation_diff, rotation_diff);
                return;
            }

            // 平滑化を適用
            apply_smoothing(transform_stamped);

            // 前回の変換として保存
            previous_transform_ = transform_stamped;
            last_update_time_ = current_time;

            // 平滑化された変換をブロードキャスト
            broadcast_smoothed_transform();
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                         target_frame_.c_str(), source_frame_.c_str(), ex.what());
        }
    }

    void apply_smoothing(const geometry_msgs::msg::TransformStamped &new_transform)
    {
        // 線形補間による平滑化
        smoothed_transform_.header.stamp = new_transform.header.stamp;
        smoothed_transform_.header.frame_id = new_transform.header.frame_id;
        smoothed_transform_.child_frame_id = smoothed_target_frame_;

        // 位置の平滑化
        smoothed_transform_.transform.translation.x =
            (1.0 - smoothing_factor_) * smoothed_transform_.transform.translation.x +
            smoothing_factor_ * new_transform.transform.translation.x;

        smoothed_transform_.transform.translation.y =
            (1.0 - smoothing_factor_) * smoothed_transform_.transform.translation.y +
            smoothing_factor_ * new_transform.transform.translation.y;

        smoothed_transform_.transform.translation.z =
            (1.0 - smoothing_factor_) * smoothed_transform_.transform.translation.z +
            smoothing_factor_ * new_transform.transform.translation.z;

        // 回転の平滑化（SLERP - 球面線形補間）
        smoothed_transform_.transform.rotation = slerp_quaternion(
            smoothed_transform_.transform.rotation,
            new_transform.transform.rotation,
            smoothing_factor_);
    }

    double calculate_rotation_difference(
        const geometry_msgs::msg::Quaternion &q1,
        const geometry_msgs::msg::Quaternion &q2)
    {
        // クォータニオンの内積を計算
        double dot_product = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;

        // 角度差を計算
        dot_product = std::abs(dot_product);
        if (dot_product > 1.0)
            dot_product = 1.0;

        return 2.0 * std::acos(dot_product);
    }

    geometry_msgs::msg::Quaternion slerp_quaternion(
        const geometry_msgs::msg::Quaternion &q1,
        const geometry_msgs::msg::Quaternion &q2,
        double t)
    {
        geometry_msgs::msg::Quaternion result;

        // 内積を計算
        double dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;

        // 最短経路を選択
        geometry_msgs::msg::Quaternion q2_corrected = q2;
        if (dot < 0.0)
        {
            q2_corrected.x = -q2.x;
            q2_corrected.y = -q2.y;
            q2_corrected.z = -q2.z;
            q2_corrected.w = -q2.w;
            dot = -dot;
        }

        // 線形補間またはSLERP
        if (dot > 0.9995)
        {
            // 線形補間（クォータニオンが非常に近い場合）
            result.x = q1.x + t * (q2_corrected.x - q1.x);
            result.y = q1.y + t * (q2_corrected.y - q1.y);
            result.z = q1.z + t * (q2_corrected.z - q1.z);
            result.w = q1.w + t * (q2_corrected.w - q1.w);
        }
        else
        {
            // SLERP
            double theta_0 = std::acos(dot);
            double sin_theta_0 = std::sin(theta_0);
            double theta = theta_0 * t;
            double sin_theta = std::sin(theta);

            double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
            double s1 = sin_theta / sin_theta_0;

            result.x = s0 * q1.x + s1 * q2_corrected.x;
            result.y = s0 * q1.y + s1 * q2_corrected.y;
            result.z = s0 * q1.z + s1 * q2_corrected.z;
            result.w = s0 * q1.w + s1 * q2_corrected.w;
        }

        // 正規化
        double norm = std::sqrt(result.x * result.x + result.y * result.y +
                                result.z * result.z + result.w * result.w);
        if (norm > 0.0)
        {
            result.x /= norm;
            result.y /= norm;
            result.z /= norm;
            result.w /= norm;
        }

        return result;
    }

    void broadcast_smoothed_transform()
    {
        tf_broadcaster_->sendTransform(smoothed_transform_);
    }

    // メンバー変数
    std::string source_frame_;
    std::string target_frame_;
    std::string smoothed_target_frame_;
    double max_translation_threshold_;
    double max_rotation_threshold_;
    std::chrono::duration<double> timeout_duration_;
    double smoothing_factor_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped previous_transform_;
    geometry_msgs::msg::TransformStamped smoothed_transform_;

    bool is_initialized_;
    rclcpp::Time last_update_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfSmoother>());
    rclcpp::shutdown();
    return 0;
}