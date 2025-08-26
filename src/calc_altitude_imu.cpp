#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class CalcAltitudeNode : public rclcpp::Node
{
public:
    CalcAltitudeNode() : Node("calc_altitude_node")
    {
        // ToFセンサーのレンジデータを購読
        tof_subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensor/tof", 10,
            std::bind(&CalcAltitudeNode::tofCallback, this, std::placeholders::_1));

        // IMUデータを購読（姿勢情報取得用）
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&CalcAltitudeNode::imuCallback, this, std::placeholders::_1));

        // 補正後の高度をパブリッシュ
        corrected_altitude_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/altitude_imu", 10);

        // 初期化
        current_roll_ = 0.0;
        current_pitch_ = 0.0;
        current_range_ = 0.0;
        has_imu_data_ = false;
        has_tof_data_ = false;

        RCLCPP_INFO(this->get_logger(), "Calc Altitude Node has been started");
    }

private:
    void tofCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        current_range_ = msg->range;
        has_tof_data_ = true;

        // IMUデータが揃っている場合、補正計算を実行
        if (has_imu_data_)
        {
            calculateCorrectedAltitude();
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // クォータニオンからロール・ピッチ角を計算
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(q);
        double yaw;
        m.getRPY(current_roll_, current_pitch_, yaw);

        has_imu_data_ = true;

        // ToFデータが揃っている場合、補正計算を実行
        if (has_tof_data_)
        {
            calculateCorrectedAltitude();
        }
    }

    void calculateCorrectedAltitude()
    {
        // ロールとピッチの傾きを考慮した垂直距離の補正
        // 機体が傾いている場合、ToFセンサーが測定する斜め距離を垂直距離に補正

        // コサイン補正: vertical_distance = measured_distance * cos(tilt_angle)
        // 総合的な傾き角度を計算（ロールとピッチの合成）
        double total_tilt = sqrt(current_roll_ * current_roll_ + current_pitch_ * current_pitch_);

        // 傾き補正係数を計算
        double correction_factor = cos(total_tilt);

        // 補正後の垂直距離を計算
        double corrected_altitude = current_range_ * correction_factor;

        // パブリッシュ
        auto msg = std_msgs::msg::Float32();
        msg.data = static_cast<float>(corrected_altitude);
        corrected_altitude_publisher_->publish(msg);

        // デバッグ情報をログ出力
        RCLCPP_DEBUG(this->get_logger(),
                     "Roll: %.3f rad, Pitch: %.3f rad, Total Tilt: %.3f rad, "
                     "Raw Range: %.3f m, Corrected Altitude: %.3f m",
                     current_roll_, current_pitch_, total_tilt, current_range_, corrected_altitude);
    }

    // サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

    // パブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr corrected_altitude_publisher_;

    // 状態変数
    double current_roll_;
    double current_pitch_;
    double current_range_;
    bool has_imu_data_;
    bool has_tof_data_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalcAltitudeNode>());
    rclcpp::shutdown();
    return 0;
}