#include <rclcpp/rclcpp.hpp>
#include <ksenos_ground_msgs/msg/pressure_data.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vector>
#include <numeric>
#include <cmath>

class AltitudeEstimator : public rclcpp::Node
{
public:
    AltitudeEstimator() : Node("altitude_estimator")
    {
        // パラメータの宣言
        this->declare_parameter("pressure_topic", "/sensor/pressure");
        this->declare_parameter("altitude_topic", "/estimated_altitude");
        this->declare_parameter("calibration_duration", 5.0);   // 校正時間（秒）
        this->declare_parameter("filter_window_size", 10);      // 移動平均フィルタのウィンドウサイズ
        this->declare_parameter("sea_level_pressure", 1013.25); // 海面気圧 (hPa)

        // パラメータの取得
        pressure_topic_ = this->get_parameter("pressure_topic").as_string();
        altitude_topic_ = this->get_parameter("altitude_topic").as_string();
        calibration_duration_ = this->get_parameter("calibration_duration").as_double();
        filter_window_size_ = this->get_parameter("filter_window_size").as_int();
        sea_level_pressure_ = this->get_parameter("sea_level_pressure").as_double();

        // サブスクライバーとパブリッシャーの作成
        pressure_sub_ = this->create_subscription<ksenos_ground_msgs::msg::PressureData>(
            pressure_topic_, 100,
            std::bind(&AltitudeEstimator::pressureCallback, this, std::placeholders::_1));

        altitude_pub_ = this->create_publisher<std_msgs::msg::Float32>(altitude_topic_, 10);
        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/estimated_position", 10);

        // 初期化
        is_calibrated_ = false;
        calibration_samples_ = 0;
        reference_pressure_ = 0.0;
        pressure_sum_ = 0.0;
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "高度推定ノードを開始しました");
        RCLCPP_INFO(this->get_logger(), "校正期間: %.1f秒", calibration_duration_);
        RCLCPP_INFO(this->get_logger(), "フィルタウィンドウサイズ: %d", filter_window_size_);
    }

private:
    void pressureCallback(const ksenos_ground_msgs::msg::PressureData::SharedPtr msg)
    {
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds();

        if (!is_calibrated_)
        {
            // 校正期間中：基準気圧を計算
            if (elapsed_time < calibration_duration_)
            {
                if (msg->pressure != -1) // 気圧データが有効な場合のみ
                {
                    double calibration_pressure = msg->pressure;

                    // 気圧がPa単位の場合、hPaに変換
                    if (calibration_pressure > 10000.0) // 100hPa以上の場合、Pa単位と判断
                    {
                        calibration_pressure = calibration_pressure / 100.0;
                    }

                    // 異常値チェック
                    if (calibration_pressure >= 300.0 && calibration_pressure <= 2000.0)
                    {
                        RCLCPP_DEBUG(this->get_logger(), "校正中: %.2f hPa", calibration_pressure);
                        pressure_sum_ += calibration_pressure;
                        calibration_samples_++;
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                             "校正中に異常な気圧値を検出: %.2f hPa", calibration_pressure);
                    }

                    // 進捗表示（1秒ごと）
                    static int last_second = -1;
                    int current_second = static_cast<int>(elapsed_time);
                    if (current_second != last_second)
                    {
                        RCLCPP_INFO(this->get_logger(), "校正中... %.1f/%.1f秒",
                                    elapsed_time, calibration_duration_);
                        last_second = current_second;
                    }
                }
            }
            else
            {
                // 校正完了
                if (calibration_samples_ > 0)
                {
                    reference_pressure_ = pressure_sum_ / calibration_samples_;
                    is_calibrated_ = true;
                    RCLCPP_INFO(this->get_logger(), "校正完了！基準気圧: %.2f hPa (サンプル数: %d)",
                                reference_pressure_, calibration_samples_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "校正期間中にデータを受信できませんでした");
                    return;
                }
            }
        }
        else
        {
            // 校正後：高度推定
            // データの有効性チェックと単位変換
            double current_pressure = msg->pressure;
            double current_temperature = msg->temperature;

            // 気圧データの単位変換（PaからhPaへ）と有効性チェック
            if (current_pressure != -1)
            {
                // 気圧がPa単位の場合、hPaに変換
                if (current_pressure > 10000.0) // 100hPa以上の場合、Pa単位と判断
                {
                    current_pressure = current_pressure / 100.0;
                }

                if (current_pressure >= 300.0 && current_pressure <= 2000.0)
                {
                    pressure_history_.push_back(current_pressure);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "異常な気圧値を検出しました: %.2f hPa", current_pressure);
                }
            }

            // 温度データの有効性チェック
            if (current_temperature != -1)
            {
                // 温度の妥当性チェック（-50℃～+70℃）
                if (current_temperature >= -50.0 && current_temperature <= 70.0)
                {
                    temperature_history_.push_back(current_temperature);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "異常な温度値を検出しました: %.1f℃", current_temperature);
                }
            }

            // フィルタウィンドウサイズを超えた場合、古いデータを削除
            if (pressure_history_.size() > static_cast<size_t>(filter_window_size_))
            {
                pressure_history_.erase(pressure_history_.begin());
            }

            if (temperature_history_.size() > static_cast<size_t>(filter_window_size_))
            {
                temperature_history_.erase(temperature_history_.begin());
            }

            // フィルタリングされた値を計算（データが十分にある場合のみ）
            if (pressure_history_.empty() || temperature_history_.empty())
            {
                return; // データが不十分な場合は処理を終了
            }

            double filtered_pressure = std::accumulate(pressure_history_.begin(),
                                                       pressure_history_.end(), 0.0) /
                                       pressure_history_.size();
            double filtered_temperature = std::accumulate(temperature_history_.begin(),
                                                          temperature_history_.end(), 0.0) /
                                          temperature_history_.size();

            // 高度推定（気圧式高度計の公式）
            // h = (T/L) * ((P0/P)^(R*L/g) - 1)
            // ここでは簡略化した国際標準大気モデルを使用
            double altitude = calculateAltitude(filtered_pressure, filtered_temperature);

            // 結果をパブリッシュ
            auto altitude_msg = std_msgs::msg::Float32();
            altitude_msg.data = static_cast<float>(altitude);
            altitude_pub_->publish(altitude_msg);

            // 位置情報としてもパブリッシュ（Z座標のみ）
            auto position_msg = geometry_msgs::msg::PointStamped();
            position_msg.header = msg->header;
            position_msg.header.frame_id = "pressure_link";
            position_msg.point.x = 0.0;
            position_msg.point.y = 0.0;
            position_msg.point.z = altitude;
            position_pub_->publish(position_msg);

            // デバッグ情報（5Hzで出力）
            static auto last_debug_time = current_time;
            if ((current_time - last_debug_time).seconds() > 0.2)
            {
                RCLCPP_DEBUG(this->get_logger(),
                             "気圧: %.2f hPa, 気温: %.1f°C, 推定高度: %.2f m",
                             filtered_pressure, filtered_temperature, altitude);
                last_debug_time = current_time;
            }
        }
    }

    double calculateAltitude(double pressure_hpa, double temperature_celsius)
    {
        // 国際標準大気モデルを使用した高度計算
        const double R = 287.0;   // 気体定数 [J/(kg·K)]
        const double g = 9.80665; // 重力加速度 [m/s²]
        const double L = 0.0065;  // 温度勾配 [K/m]

        // 温度をケルビンに変換
        double temperature_k = temperature_celsius + 273.15;

        // 基準気圧からの相対高度を計算
        // 簡略化した式: h = (T/L) * ((P0/P)^(R*L/g) - 1)
        double pressure_ratio = reference_pressure_ / pressure_hpa;
        double exponent = (R * L) / g; // ≈ 0.1902

        double altitude = (temperature_k / L) * (std::pow(pressure_ratio, exponent) - 1.0);

        return altitude;
    }

    // メンバ変数
    rclcpp::Subscription<ksenos_ground_msgs::msg::PressureData>::SharedPtr pressure_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;

    // パラメータ
    std::string pressure_topic_;
    std::string altitude_topic_;
    double calibration_duration_;
    int filter_window_size_;
    double sea_level_pressure_;

    // 校正関連
    bool is_calibrated_;
    int calibration_samples_;
    double reference_pressure_;
    double pressure_sum_;
    rclcpp::Time start_time_;

    // フィルタリング用
    std::vector<double> pressure_history_;
    std::vector<double> temperature_history_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AltitudeEstimator>();

    RCLCPP_INFO(node->get_logger(), "高度推定ノードを実行中...");

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "ノード実行中にエラーが発生しました: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}