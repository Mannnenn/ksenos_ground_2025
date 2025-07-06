#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <deque>
#include <numeric>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

struct PositionData
{
    double x, y, z;
    rclcpp::Time timestamp;

    PositionData(double x_val, double y_val, double z_val, rclcpp::Time time)
        : x(x_val), y(y_val), z(z_val), timestamp(time) {}
};

// Simple Kalman Filter for position and velocity estimation
class SimpleKalmanFilter
{
public:
    SimpleKalmanFilter()
    {
        // State vector: [x, y, z, vx, vy, vz]
        x_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6) * 1000.0; // High initial uncertainty

        // Process noise (acceleration uncertainty)
        Q_ = Eigen::MatrixXd::Zero(6, 6);
        Q_.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * 0.1; // Velocity noise

        // Measurement noise (position uncertainty)
        R_ = Eigen::MatrixXd::Identity(3, 3) * 0.01; // Position measurement noise

        // Measurement matrix (we only observe position)
        H_ = Eigen::MatrixXd::Zero(3, 6);
        H_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);

        is_initialized_ = false;
    }

    void predict(double dt)
    {
        if (!is_initialized_)
            return;

        // State transition matrix
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;

        // Predict state and covariance
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q_ * dt * dt;
    }

    void update(double pos_x, double pos_y, double pos_z)
    {
        Eigen::VectorXd z(3);
        z << pos_x, pos_y, pos_z;

        if (!is_initialized_)
        {
            // Initialize state with first measurement
            x_.head<3>() = z;
            x_.tail<3>().setZero();
            is_initialized_ = true;
            return;
        }

        // Innovation
        Eigen::VectorXd y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

        // Update state and covariance
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;
    }

    void get_velocity(double &vx, double &vy, double &vz) const
    {
        if (!is_initialized_)
        {
            vx = vy = vz = 0.0;
            return;
        }
        vx = x_(3);
        vy = x_(4);
        vz = x_(5);
    }

    bool is_initialized() const { return is_initialized_; }

private:
    Eigen::VectorXd x_; // State vector [x, y, z, vx, vy, vz]
    Eigen::MatrixXd P_; // Covariance matrix
    Eigen::MatrixXd Q_; // Process noise
    Eigen::MatrixXd R_; // Measurement noise
    Eigen::MatrixXd H_; // Measurement matrix
    bool is_initialized_;
};

class EstimateVelNode : public rclcpp::Node
{
public:
    EstimateVelNode()
        : Node("estimate_vel_node")
    {
        // === ROS2 パラメータ設定 ===
        this->declare_parameter<std::string>("target_frame", "world");
        this->declare_parameter<std::string>("source_frame", "airplane_base");
        this->declare_parameter<std::string>("velocity_topic", "estimated_velocity");
        this->declare_parameter<int>("filter_window_size", 5);
        this->declare_parameter<double>("velocity_smoothing_factor", 0.8);
        this->declare_parameter<bool>("use_moving_average", true);
        this->declare_parameter<bool>("use_kalman_filter", false);
        this->declare_parameter<double>("kalman_process_noise", 0.1);
        this->declare_parameter<double>("kalman_measurement_noise", 0.01);

        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("velocity_topic", velocity_topic_);
        this->get_parameter("filter_window_size", filter_window_size_);
        this->get_parameter("velocity_smoothing_factor", velocity_smoothing_factor_);
        this->get_parameter("use_moving_average", use_moving_average_);
        this->get_parameter("use_kalman_filter", use_kalman_filter_);
        this->get_parameter("kalman_process_noise", kalman_process_noise_);
        this->get_parameter("kalman_measurement_noise", kalman_measurement_noise_);

        RCLCPP_INFO(
            this->get_logger(),
            "Target frame: %s, Source frame: %s, Velocity topic: %s",
            target_frame_.c_str(), source_frame_.c_str(), velocity_topic_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Filter settings - Window size: %d, Smoothing factor: %.2f, Use moving average: %s, Use Kalman: %s",
            filter_window_size_, velocity_smoothing_factor_,
            use_moving_average_ ? "true" : "false", use_kalman_filter_ ? "true" : "false");

        // Initialize Kalman filter with custom noise parameters
        if (use_kalman_filter_)
        {
            // カルマンフィルタのノイズパラメータをカスタマイズ
            RCLCPP_INFO(this->get_logger(),
                        "Kalman filter enabled with process noise: %.3f, measurement noise: %.3f",
                        kalman_process_noise_, kalman_measurement_noise_);
        }

        // === TFリスナー初期化 ===
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // === パブリッシャー初期化 ===
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            velocity_topic_, 10);

        // === タイマー設定 ===
        // 100ms周期で実行
        timer_ = this->create_wall_timer(
            100ms, std::bind(&EstimateVelNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "Estimate velocity node initialized.");
    }

private:
    // Advanced velocity estimation methods
    void calculate_velocity_with_moving_average(double current_x, double current_y, double current_z,
                                                rclcpp::Time current_time,
                                                double &vel_x, double &vel_y, double &vel_z)
    {
        // Add current position to history
        position_history_.emplace_back(current_x, current_y, current_z, current_time);

        // Keep only the specified window size
        while (position_history_.size() > static_cast<size_t>(filter_window_size_))
        {
            position_history_.pop_front();
        }

        if (position_history_.size() < 2)
        {
            vel_x = vel_y = vel_z = 0.0;
            return;
        }

        // Calculate velocities using linear regression over the window
        double sum_t = 0.0, sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        double sum_t2 = 0.0, sum_tx = 0.0, sum_ty = 0.0, sum_tz = 0.0;
        size_t n = position_history_.size();

        // Use first timestamp as reference
        rclcpp::Time ref_time = position_history_[0].timestamp;

        for (const auto &pos : position_history_)
        {
            double t = (pos.timestamp - ref_time).seconds();
            sum_t += t;
            sum_x += pos.x;
            sum_y += pos.y;
            sum_z += pos.z;
            sum_t2 += t * t;
            sum_tx += t * pos.x;
            sum_ty += t * pos.y;
            sum_tz += t * pos.z;
        }

        // Linear regression: velocity = slope
        double denom = n * sum_t2 - sum_t * sum_t;
        if (std::abs(denom) < 1e-9)
        {
            vel_x = vel_y = vel_z = 0.0;
            return;
        }

        vel_x = (n * sum_tx - sum_t * sum_x) / denom;
        vel_y = (n * sum_ty - sum_t * sum_y) / denom;
        vel_z = (n * sum_tz - sum_t * sum_z) / denom;
    }

    void apply_velocity_smoothing(double &vel_x, double &vel_y, double &vel_z)
    {
        // Exponential moving average for smoothing
        smoothed_vel_x_ = velocity_smoothing_factor_ * smoothed_vel_x_ +
                          (1.0 - velocity_smoothing_factor_) * vel_x;
        smoothed_vel_y_ = velocity_smoothing_factor_ * smoothed_vel_y_ +
                          (1.0 - velocity_smoothing_factor_) * vel_y;
        smoothed_vel_z_ = velocity_smoothing_factor_ * smoothed_vel_z_ +
                          (1.0 - velocity_smoothing_factor_) * vel_z;

        vel_x = smoothed_vel_x_;
        vel_y = smoothed_vel_y_;
        vel_z = smoothed_vel_z_;
    }

    void calculate_velocity_with_kalman(double current_x, double current_y, double current_z,
                                        rclcpp::Time current_time,
                                        double &vel_x, double &vel_y, double &vel_z)
    {
        if (kalman_filter_.is_initialized())
        {
            // Calculate time difference for prediction step
            double dt = (current_time - last_kalman_time_).seconds();
            if (dt > 0.0 && dt < 1.0)
            { // Reasonable time step
                kalman_filter_.predict(dt);
            }
        }

        // Update with measurement
        kalman_filter_.update(current_x, current_y, current_z);

        // Get velocity estimate
        kalman_filter_.get_velocity(vel_x, vel_y, vel_z);

        last_kalman_time_ = current_time;
    }

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

        // 現在の位置と時刻を取得
        double current_x = t.transform.translation.x;
        double current_y = t.transform.translation.y;
        double current_z = t.transform.translation.z;
        rclcpp::Time current_time = this->get_clock()->now();

        double vel_x, vel_y, vel_z;

        if (use_kalman_filter_)
        {
            // Use Kalman filter for optimal state estimation
            calculate_velocity_with_kalman(current_x, current_y, current_z, current_time,
                                           vel_x, vel_y, vel_z);
        }
        else if (use_moving_average_)
        {
            // Use moving average method for smoother velocity estimation
            calculate_velocity_with_moving_average(current_x, current_y, current_z, current_time,
                                                   vel_x, vel_y, vel_z);
        }
        else
        {
            // Use simple differential method
            if (!is_initialized_)
            {
                // 最初の測定値で初期化
                prev_x_ = current_x;
                prev_y_ = current_y;
                prev_z_ = current_z;
                prev_time_ = current_time;
                is_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Position tracking initialized.");
                return;
            }

            // 時間差を計算
            double dt = (current_time - prev_time_).seconds();
            if (dt <= 0.0)
            {
                RCLCPP_WARN(this->get_logger(), "dt is zero or negative, skipping velocity calculation.");
                return;
            }

            // 速度を位置の微分として計算
            vel_x = (current_x - prev_x_) / dt;
            vel_y = (current_y - prev_y_) / dt;
            vel_z = (current_z - prev_z_) / dt;

            // 次回のために現在の値を保存
            prev_x_ = current_x;
            prev_y_ = current_y;
            prev_z_ = current_z;
            prev_time_ = current_time;
        }

        // Apply smoothing if enabled
        if (velocity_smoothing_factor_ > 0.0 && velocity_smoothing_factor_ < 1.0)
        {
            apply_velocity_smoothing(vel_x, vel_y, vel_z);
        }

        // 速度の大きさを計算
        double speed = std::sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z);

        RCLCPP_INFO(
            this->get_logger(),
            "Estimated Speed: %.2f [m/s] (vx=%.2f, vy=%.2f, vz=%.2f)",
            speed, vel_x, vel_y, vel_z);

        // 速度をパブリッシュ
        geometry_msgs::msg::TwistStamped velocity_msg;
        velocity_msg.header.stamp = current_time;
        velocity_msg.header.frame_id = source_frame_;
        velocity_msg.twist.linear.x = vel_x;
        velocity_msg.twist.linear.y = vel_y;
        velocity_msg.twist.linear.z = vel_z;
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

    // Position tracking for velocity estimation
    bool is_initialized_ = false;
    double prev_x_, prev_y_, prev_z_;
    rclcpp::Time prev_time_;

    // Advanced velocity estimation parameters
    int filter_window_size_;
    double velocity_smoothing_factor_;
    bool use_moving_average_;
    bool use_kalman_filter_;
    double kalman_process_noise_;
    double kalman_measurement_noise_;

    // Position history for moving average
    std::deque<PositionData> position_history_;

    // Smoothed velocity values
    double smoothed_vel_x_ = 0.0;
    double smoothed_vel_y_ = 0.0;
    double smoothed_vel_z_ = 0.0;

    // Kalman filter for advanced estimation
    SimpleKalmanFilter kalman_filter_;
    rclcpp::Time last_kalman_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimateVelNode>());
    rclcpp::shutdown();
    return 0;
}