#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <Eigen/Dense>

using namespace std;

class EstimateKalmanGain : public rclcpp::Node
{
public:
    EstimateKalmanGain() : Node("estimate_kalman_gain"), sample_count_(0), estimation_complete_(false)
    {
        // パラメータの宣言
        this->declare_parameter("estimation_duration", 30.0); // 推定時間（秒）
        this->declare_parameter("output_file", "/tmp/kalman_parameters.yaml");
        this->declare_parameter("imu_topic", "/imu/data");
        this->declare_parameter("estimation_start_delay", 5.0); // 開始前の待機時間

        estimation_duration_ = this->get_parameter("estimation_duration").as_double();
        output_file_ = this->get_parameter("output_file").as_string();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        start_delay_ = this->get_parameter("estimation_start_delay").as_double();

        // データ収集用のベクター
        gyro_x_data_.reserve(10000);
        gyro_y_data_.reserve(10000);
        gyro_z_data_.reserve(10000);
        acc_x_data_.reserve(10000);
        acc_y_data_.reserve(10000);
        acc_z_data_.reserve(10000);

        // IMUデータのサブスクライバー
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10,
            std::bind(&EstimateKalmanGain::imu_callback, this, std::placeholders::_1));

        // 推定開始タイマー
        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(start_delay_ * 1000)),
            std::bind(&EstimateKalmanGain::start_estimation, this));

        RCLCPP_INFO(this->get_logger(), "Kalman parameter estimation node started");
        RCLCPP_INFO(this->get_logger(), "Waiting %f seconds before starting data collection...", start_delay_);
    }

private:
    void start_estimation()
    {
        start_timer_->cancel();
        start_time_ = this->now();
        collecting_data_ = true;

        RCLCPP_INFO(this->get_logger(), "Starting data collection for %f seconds", estimation_duration_);

        // 推定終了タイマー
        end_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(estimation_duration_ * 1000)),
            std::bind(&EstimateKalmanGain::finish_estimation, this));
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!collecting_data_ || estimation_complete_)
        {
            return;
        }

        sample_count_++;

        // ジャイロスコープデータの収集
        gyro_x_data_.push_back(msg->angular_velocity.x);
        gyro_y_data_.push_back(msg->angular_velocity.y);
        gyro_z_data_.push_back(msg->angular_velocity.z);

        // 加速度計データの収集
        acc_x_data_.push_back(msg->linear_acceleration.x);
        acc_y_data_.push_back(msg->linear_acceleration.y);
        acc_z_data_.push_back(msg->linear_acceleration.z);

        // 進捗表示（1秒ごと）
        if (sample_count_ % 100 == 0)
        { // 100Hzと仮定
            auto elapsed = (this->now() - start_time_).seconds();
            RCLCPP_INFO(this->get_logger(), "Collected %d samples (%.1f seconds)",
                        sample_count_, elapsed);
        }
    }

    void finish_estimation()
    {
        end_timer_->cancel();
        collecting_data_ = false;
        estimation_complete_ = true;

        RCLCPP_INFO(this->get_logger(), "Data collection finished. Analyzing %d samples...", sample_count_);

        if (sample_count_ < 100)
        {
            RCLCPP_ERROR(this->get_logger(), "Insufficient data for parameter estimation");
            return;
        }

        // パラメーター推定の実行
        estimate_parameters();

        // 結果の保存
        save_parameters();

        RCLCPP_INFO(this->get_logger(), "Parameter estimation complete. Results saved to %s",
                    output_file_.c_str());
    }

    void estimate_parameters()
    {
        // 1. ジャイロスコープノイズ推定（静止状態での分散）
        process_noise_gyro_ = calculate_variance_3d(gyro_x_data_, gyro_y_data_, gyro_z_data_);

        // 2. 加速度計ノイズ推定
        // 重力を除去した加速度計ノイズを計算
        double acc_x_mean = calculate_mean(acc_x_data_);
        double acc_y_mean = calculate_mean(acc_y_data_);
        double acc_z_mean = calculate_mean(acc_z_data_);

        // 重力ベクトルを推定（静止状態では重力のみ検出される）
        double gravity_magnitude = sqrt(acc_x_mean * acc_x_mean + acc_y_mean * acc_y_mean + acc_z_mean * acc_z_mean);
        RCLCPP_INFO(this->get_logger(), "Detected gravity magnitude: %.3f m/s²", gravity_magnitude);

        // 重力を差し引いた残差の分散を計算
        process_noise_acc_ = calculate_residual_variance_3d(acc_x_data_, acc_y_data_, acc_z_data_,
                                                            acc_x_mean, acc_y_mean, acc_z_mean);

        // 3. 測定ノイズ（観測ノイズとして加速度計ノイズを使用）
        measurement_noise_ = process_noise_acc_;

        // 4. バイアスプロセスノイズ推定（時間変動の分析）
        process_noise_bias_ = estimate_bias_drift(gyro_x_data_, gyro_y_data_, gyro_z_data_);

        // 5. ヨーバイアスプロセスノイズ（Z軸ジャイロスコープの特別な処理）
        process_noise_bias_yaw_ = calculate_variance(gyro_z_data_) * 0.1; // 保守的な推定

        // 6. 初期バイアス不確実性（観測されたバイアスの標準偏差）
        double gyro_x_bias = calculate_mean(gyro_x_data_);
        double gyro_y_bias = calculate_mean(gyro_y_data_);
        double gyro_z_bias = calculate_mean(gyro_z_data_);

        initial_bias_uncertainty_ = sqrt(gyro_x_bias * gyro_x_bias +
                                         gyro_y_bias * gyro_y_bias +
                                         gyro_z_bias * gyro_z_bias);

        // 結果の表示
        RCLCPP_INFO(this->get_logger(), "Estimated parameters:");
        RCLCPP_INFO(this->get_logger(), "  process_noise_gyro: %.6f", process_noise_gyro_);
        RCLCPP_INFO(this->get_logger(), "  process_noise_acc: %.6f", process_noise_acc_);
        RCLCPP_INFO(this->get_logger(), "  measurement_noise: %.6f", measurement_noise_);
        RCLCPP_INFO(this->get_logger(), "  process_noise_bias: %.6f", process_noise_bias_);
        RCLCPP_INFO(this->get_logger(), "  process_noise_bias_yaw: %.6f", process_noise_bias_yaw_);
        RCLCPP_INFO(this->get_logger(), "  initial_bias_uncertainty: %.6f", initial_bias_uncertainty_);
    }

    double calculate_mean(const vector<double> &data)
    {
        if (data.empty())
            return 0.0;
        return accumulate(data.begin(), data.end(), 0.0) / data.size();
    }

    double calculate_variance(const vector<double> &data)
    {
        if (data.size() < 2)
            return 0.0;

        double mean = calculate_mean(data);
        double variance = 0.0;

        for (const auto &value : data)
        {
            variance += (value - mean) * (value - mean);
        }

        return variance / (data.size() - 1);
    }

    double calculate_variance_3d(const vector<double> &x_data,
                                 const vector<double> &y_data,
                                 const vector<double> &z_data)
    {
        double var_x = calculate_variance(x_data);
        double var_y = calculate_variance(y_data);
        double var_z = calculate_variance(z_data);

        return (var_x + var_y + var_z) / 3.0; // 平均分散
    }

    double calculate_residual_variance_3d(const vector<double> &x_data,
                                          const vector<double> &y_data,
                                          const vector<double> &z_data,
                                          double mean_x, double mean_y, double mean_z)
    {
        if (x_data.size() < 2)
            return 0.0;

        double variance = 0.0;
        size_t n = x_data.size();

        for (size_t i = 0; i < n; ++i)
        {
            double residual_x = x_data[i] - mean_x;
            double residual_y = y_data[i] - mean_y;
            double residual_z = z_data[i] - mean_z;
            variance += residual_x * residual_x + residual_y * residual_y + residual_z * residual_z;
        }

        return variance / (3.0 * (n - 1)); // 3軸の自由度を考慮
    }

    double estimate_bias_drift(const vector<double> &x_data,
                               const vector<double> &y_data,
                               const vector<double> &z_data)
    {
        // 時間窓での移動平均を計算してバイアスの時間変動を推定
        size_t window_size = min(static_cast<size_t>(100), x_data.size() / 10);
        if (window_size < 10)
            return 1e-6; // デフォルト値

        vector<double> bias_variations;

        for (size_t i = window_size; i < x_data.size() - window_size; i += window_size)
        {
            double mean1_x = 0, mean1_y = 0, mean1_z = 0;
            double mean2_x = 0, mean2_y = 0, mean2_z = 0;

            // 前の窓の平均
            for (size_t j = i - window_size; j < i; ++j)
            {
                mean1_x += x_data[j];
                mean1_y += y_data[j];
                mean1_z += z_data[j];
            }
            mean1_x /= window_size;
            mean1_y /= window_size;
            mean1_z /= window_size;

            // 後の窓の平均
            for (size_t j = i; j < i + window_size; ++j)
            {
                mean2_x += x_data[j];
                mean2_y += y_data[j];
                mean2_z += z_data[j];
            }
            mean2_x /= window_size;
            mean2_y /= window_size;
            mean2_z /= window_size;

            // バイアス変動の大きさ
            double variation = sqrt((mean2_x - mean1_x) * (mean2_x - mean1_x) +
                                    (mean2_y - mean1_y) * (mean2_y - mean1_y) +
                                    (mean2_z - mean1_z) * (mean2_z - mean1_z));
            bias_variations.push_back(variation);
        }

        if (bias_variations.empty())
            return 1e-6;

        return calculate_variance(bias_variations);
    }

    void save_parameters()
    {
        ofstream file(output_file_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file_.c_str());
            return;
        }

        file << "# Kalman Filter Parameters Estimated from IMU Static Data\n";
        file << "# Estimation duration: " << estimation_duration_ << " seconds\n";
        file << "# Number of samples: " << sample_count_ << "\n";
        file << "# Generated on: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "\n\n";

        file << "kalman_filter:\n";
        file << "  ros__parameters:\n";
        file << "    process_noise_gyro: " << scientific << process_noise_gyro_ << "\n";
        file << "    process_noise_acc: " << scientific << process_noise_acc_ << "\n";
        file << "    measurement_noise: " << scientific << measurement_noise_ << "\n";
        file << "    process_noise_bias: " << scientific << process_noise_bias_ << "\n";
        file << "    process_noise_bias_yaw: " << scientific << process_noise_bias_yaw_ << "\n";
        file << "    initial_bias_uncertainty: " << scientific << initial_bias_uncertainty_ << "\n";

        file.close();
    }

    // メンバー変数
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::TimerBase::SharedPtr end_timer_;

    // パラメーター
    double estimation_duration_;
    double start_delay_;
    string output_file_;
    string imu_topic_;

    // データ収集
    vector<double> gyro_x_data_, gyro_y_data_, gyro_z_data_;
    vector<double> acc_x_data_, acc_y_data_, acc_z_data_;
    int sample_count_;
    bool collecting_data_ = false;
    bool estimation_complete_;
    rclcpp::Time start_time_;

    // 推定パラメーター
    double process_noise_gyro_;
    double process_noise_acc_;
    double measurement_noise_;
    double process_noise_bias_;
    double process_noise_bias_yaw_;
    double initial_bias_uncertainty_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimateKalmanGain>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}