#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/exceptions.h"

// Eigenライブラリのヘッダ
#include <Eigen/Dense>

using namespace std::chrono_literals;

class AirplaneTrackerNode : public rclcpp::Node
{
public:
    AirplaneTrackerNode()
        : Node("airplane_tracker_node")
    {
        // === ROS2 パラメータ設定 ===
        this->declare_parameter<std::string>("target_frame", "world");
        this->declare_parameter<std::string>("source_frame", "airplane_base");
        this->declare_parameter<std::string>("velocity_topic", "estimated_velocity");
        // ノイズのパラメータ。起動時やlaunchファイルから調整可能にする
        this->declare_parameter<double>("process_noise", 0.1);
        this->declare_parameter<double>("measurement_noise", 5.0);

        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("velocity_topic", velocity_topic_);
        double process_noise = this->get_parameter("process_noise").as_double();
        double measurement_noise = this->get_parameter("measurement_noise").as_double();

        // === TFリスナー初期化 ===
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // === パブリッシャー初期化 ===
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            velocity_topic_, 10);

        // === カルマンフィルタ初期化 ===
        // 状態ベクトル x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]' (6x1)
        x_hat_ = Eigen::VectorXd(6);

        // 状態遷移行列 F (6x6) - dtに依存するためタイマー内で更新
        F_ = Eigen::MatrixXd(6, 6);

        // 観測行列 H (3x6) - 位置のみを観測
        H_ = Eigen::MatrixXd(3, 6);
        H_ << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0;

        // プロセスノイズ共分散 Q (6x6)
        Q_ = Eigen::MatrixXd::Identity(6, 6) * process_noise;

        // 観測ノイズ共分散 R (3x3)
        R_ = Eigen::MatrixXd::Identity(3, 3) * measurement_noise;

        // 状態推定共分散 P (6x6) - 大きな値で初期化
        P_ = Eigen::MatrixXd::Identity(6, 6) * 100.0;

        // === タイマー設定 ===
        // より短い周期で実行（例: 100ms）
        timer_ = this->create_wall_timer(
            100ms, std::bind(&AirplaneTrackerNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "Airplane tracker node initialized.");
    }

private:
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_->lookupTransform(
                target_frame_, source_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        // --- 観測データの準備 ---
        Eigen::VectorXd z(3);
        z << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;

        // --- カルマンフィルタの実行 ---
        if (!is_initialized_)
        {
            // 最初の観測値で状態を初期化
            x_hat_ << z(0), z(1), z(2), 0, 0, 0; // 位置は観測値、速度は0で初期化
            last_update_time_ = this->get_clock()->now();
            is_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Kalman filter initialized with first measurement.");
            return;
        }

        // --- 1. 予測 (Predict) ---
        double dt = (this->get_clock()->now() - last_update_time_).seconds();
        if (dt <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "dt is zero or negative, skipping prediction.");
            return;
        }
        last_update_time_ = this->get_clock()->now();

        // 状態遷移行列 F の更新
        F_ << 1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        x_hat_ = F_ * x_hat_;
        P_ = F_ * P_ * F_.transpose() + Q_;

        // --- 2. 更新 (Update) ---
        Eigen::VectorXd y = z - H_ * x_hat_;                   // 観測残差
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;     // 残差の共分散
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); // カルマンゲイン
        x_hat_ = x_hat_ + (K * y);                             // 状態の更新
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_ = (I - K * H_) * P_; // 共分散の更新

        // --- 結果の表示 ---
        RCLCPP_INFO(
            this->get_logger(),
            "Estimated Pos: [x: %.2f, y: %.2f, z: %.2f], Vel: [vx: %.2f, vy: %.2f, vz: %.2f]",
            x_hat_(0), x_hat_(1), x_hat_(2),
            x_hat_(3), x_hat_(4), x_hat_(5));

        // --- 速度のパブリッシュ ---
        geometry_msgs::msg::TwistStamped velocity_msg;
        velocity_msg.header.stamp = this->get_clock()->now();
        velocity_msg.header.frame_id = source_frame_;
        velocity_msg.twist.linear.x = x_hat_(3);
        velocity_msg.twist.linear.y = x_hat_(4);
        velocity_msg.twist.linear.z = x_hat_(5);
        velocity_msg.twist.angular.x = 0.0;
        velocity_msg.twist.angular.y = 0.0;
        velocity_msg.twist.angular.z = 0.0;

        velocity_publisher_->publish(velocity_msg);
    }

    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::string target_frame_;
    std::string source_frame_;
    std::string velocity_topic_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;

    // Kalman Filter
    bool is_initialized_ = false;
    rclcpp::Time last_update_time_;
    Eigen::VectorXd x_hat_; // State estimate
    Eigen::MatrixXd F_;     // State transition matrix
    Eigen::MatrixXd H_;     // Measurement matrix
    Eigen::MatrixXd Q_;     // Process noise covariance
    Eigen::MatrixXd R_;     // Measurement noise covariance
    Eigen::MatrixXd P_;     // Estimate error covariance
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirplaneTrackerNode>());
    rclcpp::shutdown();
    return 0;
}