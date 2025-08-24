#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <cmath>
#include <numeric>

#include "sensor_msgs/msg/imu.hpp"

#include "ksenos_ground_msgs/msg/rpy.hpp"
#include "std_msgs/msg/float32.hpp"

class RotationCounterNode : public rclcpp::Node
{
public:
    RotationCounterNode()
        : Node("yaw_angle_serialization_node"),
          start_yaw_(0.0),
          previous_yaw_(0.0),
          yaw_offset_(0.0),
          rotation_speed_ave_(0.0),
          rotation_direction_(0),
          over_2pi_(0),
          is_first_counter_(true),
          last_time_(this->now()),
          last_time_for_several_rotate_(this->now())
    {
        setupParameters();
        setupSubscribersAndPublishers();

        RCLCPP_INFO(this->get_logger(), "yaw_angle_serialization_node has been started.");
    }

private:
    // 定数定義
    static constexpr double BEYOND_BOUNDARY_CRITERIA = 0.1;
    static constexpr double LAP_JUDGMENT_CRITERIA = 0.1;
    static constexpr double COUNT_DUPLICATION_TIME_CRITERIA = 1.0;
    static constexpr size_t YAW_SPEED_HISTORY_SIZE = 10;

    void setupParameters()
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_angular_topic_name", "/rpy");
        this->declare_parameter<std::string>("input_angular_velocity_topic_name", "/angular_velocity");
        this->declare_parameter<std::string>("input_counter_reset_topic_name", "/counter_reset");
        this->declare_parameter<std::string>("output_serialized_yaw_angle_topic_name", "/yaw_angle_serialization");
    }

    void setupSubscribersAndPublishers()
    {
        // パラメータの取得
        std::string input_angular_topic_name;
        this->get_parameter("input_angular_topic_name", input_angular_topic_name);
        std::string input_angular_velocity_topic_name;
        this->get_parameter("input_angular_velocity_topic_name", input_angular_velocity_topic_name);
        std::string input_counter_reset_topic_name;
        this->get_parameter("input_counter_reset_topic_name", input_counter_reset_topic_name);
        std::string output_serialized_yaw_angle_topic_name;
        this->get_parameter("output_serialized_yaw_angle_topic_name", output_serialized_yaw_angle_topic_name);

        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        rpy_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::Rpy>(
            input_angular_topic_name, qos,
            std::bind(&RotationCounterNode::rpy_callback, this, std::placeholders::_1));

        angular_velocity_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_angular_velocity_topic_name, qos,
            std::bind(&RotationCounterNode::angular_velocity_callback, this, std::placeholders::_1));

        rclcpp::QoS qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        counter_reset_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            input_counter_reset_topic_name, qos_reliable,
            std::bind(&RotationCounterNode::counter_reset_callback, this, std::placeholders::_1));

        yaw_angle_serialization_pub_ = this->create_publisher<std_msgs::msg::Float32>(output_serialized_yaw_angle_topic_name, qos_reliable);
    }
    void rpy_callback(const ksenos_ground_msgs::msg::Rpy::SharedPtr msg)
    {
        // プラスが左回転、マイナスが右回転
        // カウントアップか図る角度変更でリセット
        initializeCounterIfNeeded(msg->yaw);

        previous_yaw_ = msg->yaw;

        yaw_corrected_ = calculateCorrectedYaw();

        publishCounter();
        checkAndUpdateBoundary(yaw_corrected_);
    }

    void initializeCounterIfNeeded(double current_yaw)
    {
        if (is_first_counter_)
        {
            start_yaw_ = current_yaw;
            over_2pi_ = 0;
            yaw_offset_ = 0;
            is_first_counter_ = false;
        }
    }

    double calculateCorrectedYaw()
    {
        // 回転量が連続になるように正に1回転したら+2π、負に1回転したら-2πする
        yaw_offset_ = 2 * M_PI * over_2pi_;

        // Yawの-π~πの値を回転数分ずらす
        return previous_yaw_ + yaw_offset_;
    }

    void resetCounterState()
    {
        is_first_counter_ = true;
        last_time_ = this->now();
    }

    void checkAndUpdateBoundary(double yaw_corrected)
    {
        rclcpp::Time current_time = this->now();
        rclcpp::Duration time_diff = current_time - last_time_for_several_rotate_;

        // 境界値に近づいたら実行される
        if (rotation_direction_ == 1 &&
            time_diff.seconds() > COUNT_DUPLICATION_TIME_CRITERIA &&
            yaw_corrected > M_PI * (1 + 2 * over_2pi_) - BEYOND_BOUNDARY_CRITERIA)
        {
            // 左回転で境界値を超えたので一回転分正にオフセット
            over_2pi_++;
            last_time_for_several_rotate_ = this->now();
        }
        else if (rotation_direction_ == -1 &&
                 time_diff.seconds() > COUNT_DUPLICATION_TIME_CRITERIA &&
                 yaw_corrected < M_PI * (-1 + 2 * over_2pi_) + BEYOND_BOUNDARY_CRITERIA)
        {
            // 右回転で境界値を超えたので一回転分負にオフセット
            over_2pi_--;
            last_time_for_several_rotate_ = this->now();
        }
    }

    void publishCounter()
    {
        std_msgs::msg::Float32 serialized_yaw_angle_msg;
        serialized_yaw_angle_msg.data = yaw_corrected_;
        yaw_angle_serialization_pub_->publish(serialized_yaw_angle_msg);
    }

    void angular_velocity_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        updateYawSpeedHistory(msg->angular_velocity.z);
        calculateAverageRotationSpeed();
        updateRotationDirection();
    }

    void updateYawSpeedHistory(double yaw_speed)
    {
        yaw_speed_history_.push_back(yaw_speed);
        if (yaw_speed_history_.size() > YAW_SPEED_HISTORY_SIZE)
        {
            yaw_speed_history_.pop_front();
        }
    }

    void calculateAverageRotationSpeed()
    {
        rotation_speed_ave_ = std::accumulate(yaw_speed_history_.begin(), yaw_speed_history_.end(), 0.0) / yaw_speed_history_.size();
    }

    void updateRotationDirection()
    {
        if (rotation_speed_ave_ > 0)
        {
            rotation_direction_ = 1; // 値が増加するのは左旋回
        }
        else if (rotation_speed_ave_ < 0)
        {
            rotation_direction_ = -1; // 値が減少するのは右旋回
        }
    }

    void counter_reset_callback(const std_msgs::msg::Float32::SharedPtr /* msg */)
    {
        resetCounter();
    }

    void resetCounter()
    {
        is_first_counter_ = true;
        publishCounter();
    }

    // サブスクライバー
    rclcpp::Subscription<ksenos_ground_msgs::msg::Rpy>::SharedPtr rpy_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr angular_velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr counter_reset_subscriber_;

    // パブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_angle_serialization_pub_;

    // 角度関連
    double start_yaw_;
    double previous_yaw_;
    double yaw_offset_;
    double yaw_corrected_;

    // 回転関連
    double rotation_speed_ave_;
    int rotation_direction_;

    // 履歴管理
    std::deque<double> yaw_speed_history_;
    int over_2pi_;
    bool is_first_counter_;

    // 時間管理
    rclcpp::Time last_time_;
    rclcpp::Time last_time_for_several_rotate_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotationCounterNode>());
    rclcpp::shutdown();
    return 0;
}