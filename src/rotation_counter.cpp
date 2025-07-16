#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <cmath>
#include <deque>

#include "ksenos_ground_msgs/msg/rpy.hpp"
#include "ksenos_ground_msgs/msg/command.hpp"

#include "std_msgs/msg/float32.hpp"

class RotationCounterNode : public rclcpp::Node
{
public:
    RotationCounterNode()
        : Node("rotation_counter_node")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_angular_topic_name", "/rpy");
        this->declare_parameter<std::string>("input_angular_velocity_topic_name", "/angular_velocity");
        this->declare_parameter<std::string>("input_counter_reset_topic_name", "/counter_reset");
        this->declare_parameter<std::string>("output_rotation_counter_topic_name", "/rotation_counter");

        // パラメータの取得
        std::string input_angular_topic_name;
        this->get_parameter("input_angular_topic_name", input_angular_topic_name);
        std::string input_angular_velocity_topic_name;
        this->get_parameter("input_angular_velocity_topic_name", input_angular_velocity_topic_name);
        std::string input_counter_reset_topic_name;
        this->get_parameter("input_counter_reset_topic_name", input_counter_reset_topic_name);
        std::string output_rotation_counter_topic_name;
        this->get_parameter("output_rotation_counter_topic_name", output_rotation_counter_topic_name);

        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        rpy_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::Rpy>(input_angular_topic_name, qos, std::bind(&RotationCounterNode::rpy_callback, this, std::placeholders::_1));
        angular_velocity_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::Rpy>(input_angular_velocity_topic_name, qos, std::bind(&RotationCounterNode::angular_velocity_callback, this, std::placeholders::_1));

        rclcpp::QoS qos_rest = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        counter_reset_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(input_counter_reset_topic_name, qos_rest, std::bind(&RotationCounterNode::counter_reset_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "rotation_counter_node has been started.");

        rotation_counter_pub_ = this->create_publisher<std_msgs::msg::Float32>(output_rotation_counter_topic_name, qos_rest);

        beyond_boundary_judgment_criteria = 0.1;
        lap_judgment_criteria = 0.1;
        count_duplication_time_judgment_criteria = 1.0;

        rotation_standard = 1 * 2 * M_PI; // 1 Rotation:2π [rad]

        rotation_direction_ = 0;
        last_time_ = this->now();
        last_time_for_several_rotate_ = this->now();
    }

private:
    void rpy_callback(const ksenos_ground_msgs::msg::Rpy::SharedPtr msg)
    {
        // プラスが左回転、マイナスが右回転
        // カウントアップか図る角度変更でリセット
        if (is_first_counter_)
        {
            start_yaw_ = msg->yaw;
            over_2pi_ = 0;
            yaw_offset_ = 0;
            is_first_counter_ = false;
        }

        previous_yaw_ = msg->yaw;

        // 回転量が連続になるように正に1回転したら+2π、負に1回転したら-2πする
        yaw_offset_ = 2 * M_PI * over_2pi_;

        // Yawの-π~πの値を回転数分ずらす
        double yaw_correct = previous_yaw_ + yaw_offset_;

        // gaolの値を決定する。最初の値に何度で1回転とするかの値に回転方向をかけて足す。これでプラス回転で初期角度から1回転分増えた値に、マイナス回転で初期角度から1回転分減った値になる
        goal_yaw_ = start_yaw_ + rotation_standard * rotation_direction_;
        // 　
        if (over_2pi_ > 0)
        {
            goal_yaw_ += 2 * M_PI * (over_2pi_ - 1);
        }
        else if (over_2pi_ < 0)
        {
            goal_yaw_ -= 2 * M_PI * (over_2pi_ + 1);
        }

        rclcpp::Time current_time = this->now();
        rclcpp::Duration diff = current_time - last_time_;

        // 回転方向が正かつ範囲内
        if (rotation_direction_ == 1 &&
            goal_yaw_ - lap_judgment_criteria < yaw_correct && yaw_correct < goal_yaw_ &&
            diff.seconds() > count_duplication_time_judgment_criteria)
        {
            rotation_counter_++;
            is_first_counter_ = true;
            last_time_ = this->now();
        }
        // 回転方向が負かつ範囲内
        else if (rotation_direction_ == -1 &&
                 goal_yaw_ < yaw_correct && yaw_correct < goal_yaw_ + lap_judgment_criteria &&
                 diff.seconds() > count_duplication_time_judgment_criteria)
        {
            rotation_counter_--;
            is_first_counter_ = true;
            last_time_ = this->now();
        }

        pub_counter();

        current_time = this->now();
        diff = current_time - last_time_for_several_rotate_;

        // 境界値に近づいたら実行される
        if (rotation_direction_ == 1 &&
            diff.seconds() > count_duplication_time_judgment_criteria &&
            yaw_correct > M_PI * (1 + 2 * over_2pi_) - beyond_boundary_judgment_criteria)
        {
            // 左回転で境界値を超えたので一回転分正にオフセット
            over_2pi_++;
            last_time_for_several_rotate_ = this->now();
        }
        else if (rotation_direction_ == -1 &&
                 diff.seconds() > count_duplication_time_judgment_criteria &&
                 yaw_correct < M_PI * (-1 + 2 * over_2pi_) + beyond_boundary_judgment_criteria)
        {
            // 右回転で境界値を超えたので一回転分負にオフセット
            over_2pi_--;
            last_time_for_several_rotate_ = this->now();
        }
    }

    void pub_counter()
    {

        std_msgs::msg::Float32 rotation_counter_msg;
        rotation_counter_msg.data = rotation_counter_;
        rotation_counter_pub_->publish(rotation_counter_msg);
    }

    void angular_velocity_callback(const ksenos_ground_msgs::msg::Rpy::SharedPtr msg)
    {
        yaw_speed_history_.push_back(msg->yaw);
        if (yaw_speed_history_.size() > 5)
        {
            yaw_speed_history_.pop_front();
        }

        rotation_speed_ave_ = std::accumulate(yaw_speed_history_.begin(), yaw_speed_history_.end(), 0.0) / yaw_speed_history_.size();

        if (rotation_speed_ave_ > 0)
        {
            rotation_direction_ = 1; // 値が増加するのは左旋回
        }
        else if (rotation_speed_ave_ < 0)
        {
            rotation_direction_ = -1; // 値が減少するのは右旋回
        }
    }

    void counter_reset_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        rotation_standard = msg->data * 2 * M_PI; // dataは何回転で1つ出力するかを示す。
        rotation_counter_ = 0;
        is_first_counter_ = true;
        pub_counter();
    }

    rclcpp::Subscription<ksenos_ground_msgs::msg::Rpy>::SharedPtr rpy_subscriber_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::Rpy>::SharedPtr angular_velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr counter_reset_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rotation_counter_pub_;

    double start_yaw_;
    double goal_yaw_;

    double rotation_speed_ave_;
    double rotation_direction_;

    double rotation_counter_;
    // 境界値をまたいだかの判断基準
    double beyond_boundary_judgment_criteria;
    // 一周したかの判断基準
    double lap_judgment_criteria;
    // 時間的に回転数の計測がかぶらないようにフィルターをかける
    double count_duplication_time_judgment_criteria;

    std::deque<double>
        yaw_speed_history_;

    double yaw_offset_;
    double previous_yaw_;
    int over_2pi_;
    bool is_first_counter_;

    double rotation_standard;

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