#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

class AttitudeEstimatorKF : public rclcpp::Node
{
public:
    AttitudeEstimatorKF() : Node("attitude_estimator_kf"), is_initialized_(false), last_time_(0.0),
                            bias_calibration_complete_(false), bias_sample_count_(0)
    {
        // パラメータの宣言
        this->declare_parameter("imu_topic", "/imu/data");
        this->declare_parameter("pose_topic", "/estimated_pose");
        this->declare_parameter("process_noise_gyro", 0.01);
        this->declare_parameter("process_noise_acc", 0.1);
        this->declare_parameter("measurement_noise", 0.1);
        this->declare_parameter("process_noise_bias", 1e-6);
        this->declare_parameter("process_noise_bias_yaw", 1e-4); // ヨー軸バイアス用（大きめの値）
        this->declare_parameter("initial_bias_uncertainty", 0.1);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("child_frame", "imu_link");
        this->declare_parameter("bias_calibration_time", 5.0);    // バイアス計測時間（秒）
        this->declare_parameter("bias_calibration_samples", 300); // 最小サンプル数

        // パラメータの取得
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        process_noise_gyro_ = this->get_parameter("process_noise_gyro").as_double();
        process_noise_acc_ = this->get_parameter("process_noise_acc").as_double();
        measurement_noise_ = this->get_parameter("measurement_noise").as_double();
        process_noise_bias_ = this->get_parameter("process_noise_bias").as_double();
        process_noise_bias_yaw_ = this->get_parameter("process_noise_bias_yaw").as_double();
        initial_bias_uncertainty_ = this->get_parameter("initial_bias_uncertainty").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();
        bias_calibration_time_ = this->get_parameter("bias_calibration_time").as_double();
        bias_calibration_samples_ = this->get_parameter("bias_calibration_samples").as_int();

        // バイアス計測用変数の初期化
        gyro_bias_sum_ = Eigen::Vector3d::Zero();
        bias_start_time_ = 0.0;

        // Publisher/Subscriberの初期化
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&AttitudeEstimatorKF::imuCallback, this, std::placeholders::_1));

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10);

        // TFブロードキャスターの初期化
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // カルマンフィルターの初期化
        initializeKalmanFilter();

        RCLCPP_INFO(this->get_logger(), "Attitude Estimator with Kalman Filter initialized");
        RCLCPP_INFO(this->get_logger(), "Starting gyro bias calibration for %.1f seconds...", bias_calibration_time_);
    }

private:
    void initializeKalmanFilter()
    {
        // 状態ベクトル: [qw, qx, qy, qz, bias_gx, bias_gy, bias_gz]
        x_ = Eigen::VectorXd::Zero(7);
        x_(0) = 1.0; // 初期クォータニオン (w=1, x=y=z=0)

        // 状態共分散行列
        P_ = Eigen::MatrixXd::Identity(7, 7);
        P_.block<4, 4>(0, 0) *= 0.1;                       // クォータニオンの初期不確かさ
        P_.block<3, 3>(4, 4) *= initial_bias_uncertainty_; // ジャイロバイアスの初期不確かさ

        // プロセスノイズ共分散行列
        Q_ = Eigen::MatrixXd::Zero(7, 7);
        Q_.block<4, 4>(0, 0) = Eigen::MatrixXd::Identity(4, 4) * process_noise_gyro_;
        // ジャイロバイアス: X,Y軸は小さく、Z軸（ヨー）は大きく設定
        Q_(4, 4) = process_noise_bias_;     // X軸バイアス
        Q_(5, 5) = process_noise_bias_;     // Y軸バイアス
        Q_(6, 6) = process_noise_bias_yaw_; // Z軸（ヨー）バイアス

        // 観測ノイズ共分散行列（加速度センサーから得られるロール・ピッチ）
        R_ = Eigen::MatrixXd::Identity(2, 2) * measurement_noise_;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // バイアス計測フェーズ
        if (!bias_calibration_complete_)
        {
            calibrateGyroBias(msg, current_time);
            return;
        }

        if (!is_initialized_)
        {
            // 初期姿勢を加速度センサーから推定
            initializePoseFromAccelerometer(msg);
            last_time_ = current_time;
            is_initialized_ = true;
            return;
        }

        double dt = current_time - last_time_;
        if (dt <= 0.0 || dt > 1.0) // 異常なdt値をスキップ
        {
            last_time_ = current_time;
            return;
        }

        // カルマンフィルターの予測ステップ
        predict(msg, dt);

        // カルマンフィルターの更新ステップ（加速度センサーによる観測）
        update(msg);

        // 結果の公開
        publishPose(msg->header);
        publishTF(msg->header);

        last_time_ = current_time;
    }

    void calibrateGyroBias(const sensor_msgs::msg::Imu::SharedPtr msg, double current_time)
    {
        if (bias_start_time_ == 0.0)
        {
            bias_start_time_ = current_time;
        }

        double elapsed_time = current_time - bias_start_time_;

        // 静止状態の検証（加速度がほぼ重力加速度と一致するかチェック）
        Eigen::Vector3d acc(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
        double acc_magnitude = acc.norm();

        // 重力加速度から大きく外れている場合は動いている可能性があるので警告
        if (std::abs(acc_magnitude - 9.81) > 0.5)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Device may be moving during bias calibration! Acc magnitude: %.2f", acc_magnitude);
        }

        // ジャイロスコープデータを累積
        gyro_bias_sum_.x() += msg->angular_velocity.x;
        gyro_bias_sum_.y() += msg->angular_velocity.y;
        gyro_bias_sum_.z() += msg->angular_velocity.z;
        bias_sample_count_++;

        // 進捗表示
        if (bias_sample_count_ % 50 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Bias calibration progress: %.1f%% (%d samples)",
                        (elapsed_time / bias_calibration_time_) * 100.0, bias_sample_count_);
        }

        // 計測完了条件をチェック
        if (elapsed_time >= bias_calibration_time_ && bias_sample_count_ >= bias_calibration_samples_)
        {
            // バイアスの計算
            Eigen::Vector3d measured_bias = gyro_bias_sum_ / bias_sample_count_;

            // カルマンフィルターの状態にバイアスを設定
            x_(4) = measured_bias.x(); // X軸バイアス
            x_(5) = measured_bias.y(); // Y軸バイアス
            x_(6) = measured_bias.z(); // Z軸（ヨー）バイアス

            // バイアスの共分散を小さくする（より確信を持つ）
            P_(4, 4) = 1e-12; // X軸バイアスの不確かさを非常に小さく
            P_(5, 5) = 1e-12; // Y軸バイアスの不確かさを非常に小さく
            P_(6, 6) = 1e-9;  // Z軸バイアスの不確かさを非常に小さく

            bias_calibration_complete_ = true;

            RCLCPP_INFO(this->get_logger(),
                        "Gyro bias calibration completed with %d samples over %.1f seconds",
                        bias_sample_count_, elapsed_time);
            RCLCPP_INFO(this->get_logger(),
                        "Measured gyro bias: X=%.6f, Y=%.6f, Z=%.6f rad/s",
                        measured_bias.x(), measured_bias.y(), measured_bias.z());
            RCLCPP_INFO(this->get_logger(),
                        "Yaw bias: %.4f deg/s", measured_bias.z() * 180.0 / M_PI);
        }
    }

    void initializePoseFromAccelerometer(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 加速度ベクトルから初期ロール・ピッチを計算
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        // 正規化
        double norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm > 0.1) // ゼロ除算防止
        {
            ax /= norm;
            ay /= norm;
            az /= norm;
        }

        // ロール・ピッチ計算
        double roll = atan2(ay, az);
        double pitch = atan2(-ax, sqrt(ay * ay + az * az));
        double yaw = 0.0; // 初期ヨーは0と仮定

        // オイラー角からクォータニオンに変換
        Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        x_(0) = q.w();
        x_(1) = q.x();
        x_(2) = q.y();
        x_(3) = q.z();

        RCLCPP_INFO(this->get_logger(), "Initial pose: roll=%.3f, pitch=%.3f, yaw=%.3f",
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }

    void predict(const sensor_msgs::msg::Imu::SharedPtr msg, double dt)
    {
        // ジャイロスコープデータ（バイアス補正済み）
        Eigen::Vector3d omega;
        omega << msg->angular_velocity.x - x_(4),
            msg->angular_velocity.y - x_(5),
            msg->angular_velocity.z - x_(6);

        // クォータニオンの微分方程式
        Eigen::Quaterniond q(x_(0), x_(1), x_(2), x_(3));
        Eigen::Quaterniond q_omega(0, omega.x() * 0.5, omega.y() * 0.5, omega.z() * 0.5);
        Eigen::Quaterniond q_dot = q * q_omega;

        // クォータニオンの更新（オイラー積分）
        q.w() += q_dot.w() * dt;
        q.x() += q_dot.x() * dt;
        q.y() += q_dot.y() * dt;
        q.z() += q_dot.z() * dt;

        // 正規化
        q.normalize();

        // 状態ベクトルの更新
        x_(0) = q.w();
        x_(1) = q.x();
        x_(2) = q.y();
        x_(3) = q.z();
        // バイアスは変化しないと仮定

        // ヤコビアン行列の計算
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7, 7);

        // クォータニオンに対するヤコビアン
        double qw = x_(0), qx = x_(1), qy = x_(2), qz = x_(3);
        double wx = omega.x(), wy = omega.y(), wz = omega.z();

        F(0, 0) = 1;
        F(0, 1) = -wx * dt * 0.5;
        F(0, 2) = -wy * dt * 0.5;
        F(0, 3) = -wz * dt * 0.5;
        F(1, 0) = wx * dt * 0.5;
        F(1, 1) = 1;
        F(1, 2) = wz * dt * 0.5;
        F(1, 3) = -wy * dt * 0.5;
        F(2, 0) = wy * dt * 0.5;
        F(2, 1) = -wz * dt * 0.5;
        F(2, 2) = 1;
        F(2, 3) = wx * dt * 0.5;
        F(3, 0) = wz * dt * 0.5;
        F(3, 1) = wy * dt * 0.5;
        F(3, 2) = -wx * dt * 0.5;
        F(3, 3) = 1;

        // バイアスに対するヤコビアン
        F(0, 4) = qx * dt * 0.5;
        F(0, 5) = qy * dt * 0.5;
        F(0, 6) = qz * dt * 0.5;
        F(1, 4) = -qw * dt * 0.5;
        F(1, 5) = qz * dt * 0.5;
        F(1, 6) = -qy * dt * 0.5;
        F(2, 4) = -qz * dt * 0.5;
        F(2, 5) = -qw * dt * 0.5;
        F(2, 6) = qx * dt * 0.5;
        F(3, 4) = qy * dt * 0.5;
        F(3, 5) = -qx * dt * 0.5;
        F(3, 6) = -qw * dt * 0.5;

        // 共分散行列の更新
        P_ = F * P_ * F.transpose() + Q_ * dt;
    }

    void update(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 加速度センサーのデータ
        Eigen::Vector3d acc;
        acc << msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z;

        // 正規化（重力ベクトルとして扱う）
        double norm = acc.norm();
        if (norm < 0.1)
            return; // 無重力状態では更新しない

        acc /= norm;

        // 予測される重力ベクトル（クォータニオンから計算）
        Eigen::Quaterniond q(x_(0), x_(1), x_(2), x_(3));
        Eigen::Vector3d gravity_body = q.inverse() * Eigen::Vector3d(0, 0, -1);

        // 観測値（加速度センサーから得られるロール・ピッチ）
        Eigen::VectorXd z(2);
        z(0) = atan2(acc.y(), acc.z());                                      // ロール
        z(1) = atan2(-acc.x(), sqrt(acc.y() * acc.y() + acc.z() * acc.z())); // ピッチ

        // 予測される観測値
        Eigen::VectorXd h(2);
        h(0) = atan2(gravity_body.y(), gravity_body.z());                                                                 // 予測ロール
        h(1) = atan2(-gravity_body.x(), sqrt(gravity_body.y() * gravity_body.y() + gravity_body.z() * gravity_body.z())); // 予測ピッチ

        // 観測ヤコビアン（簡易的な線形化）
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 7);

        // 数値微分でヤコビアンを計算
        double eps = 1e-6;
        for (int i = 0; i < 4; ++i)
        {
            Eigen::VectorXd x_plus = x_;
            x_plus(i) += eps;

            // 正規化
            Eigen::Quaterniond q_plus(x_plus(0), x_plus(1), x_plus(2), x_plus(3));
            q_plus.normalize();

            Eigen::Vector3d gravity_plus = q_plus.inverse() * Eigen::Vector3d(0, 0, -1);
            Eigen::VectorXd h_plus(2);
            h_plus(0) = atan2(gravity_plus.y(), gravity_plus.z());
            h_plus(1) = atan2(-gravity_plus.x(), sqrt(gravity_plus.y() * gravity_plus.y() + gravity_plus.z() * gravity_plus.z()));

            H(0, i) = (h_plus(0) - h(0)) / eps;
            H(1, i) = (h_plus(1) - h(1)) / eps;
        }

        // カルマンゲインの計算
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 状態の更新
        Eigen::VectorXd y = z - h; // 残差

        // 角度の差分を適切に処理
        while (y(0) > M_PI)
            y(0) -= 2 * M_PI;
        while (y(0) < -M_PI)
            y(0) += 2 * M_PI;
        while (y(1) > M_PI)
            y(1) -= 2 * M_PI;
        while (y(1) < -M_PI)
            y(1) += 2 * M_PI;

        x_ = x_ + K * y;

        // クォータニオンの正規化
        Eigen::Quaterniond q_updated(x_(0), x_(1), x_(2), x_(3));
        q_updated.normalize();
        x_(0) = q_updated.w();
        x_(1) = q_updated.x();
        x_(2) = q_updated.y();
        x_(3) = q_updated.z();

        // 共分散行列の更新
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
        P_ = (I - K * H) * P_;
    }

    void publishPose(const std_msgs::msg::Header &header)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = header;
        pose_msg.header.frame_id = base_frame_;

        pose_msg.pose.orientation.w = x_(0);
        pose_msg.pose.orientation.x = x_(1);
        pose_msg.pose.orientation.y = x_(2);
        pose_msg.pose.orientation.z = x_(3);

        pose_msg.pose.position.x = 0.0;
        pose_msg.pose.position.y = 0.0;
        pose_msg.pose.position.z = 0.0;

        pose_publisher_->publish(pose_msg);

        // デバッグ用：オイラー角での出力
        Eigen::Quaterniond q(x_(0), x_(1), x_(2), x_(3));
        auto euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // YPR順

        RCLCPP_INFO(this->get_logger(),
                    "Pose: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°, Bias=[%.4f, %.4f, %.4f]",
                    euler[2] * 180.0 / M_PI, // Roll
                    euler[1] * 180.0 / M_PI, // Pitch
                    euler[0] * 180.0 / M_PI, // Yaw
                    x_(4), x_(5), x_(6));    // Gyro bias
    }

    void publishTF(const std_msgs::msg::Header &header)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header = header;
        transform_stamped.header.frame_id = base_frame_;
        transform_stamped.child_frame_id = child_frame_;

        // 位置は原点とする
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;

        // 姿勢（クォータニオン）
        transform_stamped.transform.rotation.w = x_(0);
        transform_stamped.transform.rotation.x = x_(1);
        transform_stamped.transform.rotation.y = x_(2);
        transform_stamped.transform.rotation.z = x_(3);

        // TFを送信
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // カルマンフィルターの状態
    Eigen::VectorXd x_; // 状態ベクトル [qw, qx, qy, qz, bias_gx, bias_gy, bias_gz]
    Eigen::MatrixXd P_; // 状態共分散行列
    Eigen::MatrixXd Q_; // プロセスノイズ共分散行列
    Eigen::MatrixXd R_; // 観測ノイズ共分散行列

    // パラメータ
    double process_noise_gyro_;
    double process_noise_acc_;
    double measurement_noise_;
    double process_noise_bias_;
    double process_noise_bias_yaw_; // ヨー軸バイアス用
    double initial_bias_uncertainty_;
    std::string base_frame_;
    std::string child_frame_;
    double bias_calibration_time_;
    int bias_calibration_samples_;

    // フィルターの状態
    bool is_initialized_;
    double last_time_;

    // バイアス計測用変数
    bool bias_calibration_complete_;
    int bias_sample_count_;
    Eigen::Vector3d gyro_bias_sum_;
    double bias_start_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AttitudeEstimatorKF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}