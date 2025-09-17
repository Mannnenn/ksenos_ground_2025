#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "ksenos_ground_msgs/msg/sbus_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class ControlToJointPublisher : public rclcpp::Node
{
public:
    ControlToJointPublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("control_to_joint_publisher", options), propeller_angle_(0.0), is_manual_mode_(true)
    {
        last_update_time_ = this->now();

        // モード管理用のサブスクライバー（/sbus_dataからモード情報を取得）
        mode_subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "/sbus_data", 10, std::bind(&ControlToJointPublisher::mode_callback, this, _1));

        // 常に両方のサブスクライバーを作成
        manual_subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_manual", 10, std::bind(&ControlToJointPublisher::manual_callback, this, _1));

        auto_subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_auto", 10, std::bind(&ControlToJointPublisher::auto_callback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        RCLCPP_INFO(this->get_logger(), "Joint Publisher node started. Subscribing to both sbus_manual and sbus_auto.");
    }

private:
    void mode_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        bool new_manual_mode = (msg->autopilot_mode == "manual");

        if (new_manual_mode != is_manual_mode_)
        {
            is_manual_mode_ = new_manual_mode;
            RCLCPP_INFO(this->get_logger(), "Mode changed to: %s",
                        is_manual_mode_ ? "manual" : "auto");
        }
    }

    void manual_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // manualモードの時のみパブリッシュ
        if (is_manual_mode_)
        {
            publish_joint_state(msg);
            RCLCPP_DEBUG(this->get_logger(), "Published joint states from manual mode");
        }
    }

    void auto_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // autoモードの時のみパブリッシュ
        if (!is_manual_mode_)
        {
            publish_joint_state(msg);
            RCLCPP_DEBUG(this->get_logger(), "Published joint states from auto mode");
        }
    }

    void publish_joint_state(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // 時刻差dtの計算
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // スケール係数（必要に応じて調整）
        constexpr double scale_vel = 15.0;
        // throttleに比例した回転速度
        double propeller_velocity = static_cast<double>(msg->throttle) * scale_vel;
        // 経過時間分、propeller角度を更新
        propeller_angle_ += propeller_velocity * dt;

        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = current_time;
        // Jointの名前を設定
        joint_msg.name = {"joint_propeller", "joint_elevator", "joint_rudder", "joint_aileron_left", "joint_aileron_right"};

        // 各関節のpositionを設定
        joint_msg.position = {-propeller_angle_,
                              -static_cast<double>(msg->elevator),
                              -static_cast<double>(msg->rudder),
                              -static_cast<double>(msg->aileron_l),
                              -static_cast<double>(msg->aileron_r)};

        // continuous rotationのjoint_propeller用にvelocityを設定
        joint_msg.velocity = {propeller_velocity, 0.0, 0.0, 0.0, 0.0};

        publisher_->publish(joint_msg);
    }

    // メンバ変数
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr mode_subscription_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr manual_subscription_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr auto_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    double propeller_angle_;
    rclcpp::Time last_update_time_;
    bool is_manual_mode_;
};

// コンポーネントの登録
RCLCPP_COMPONENTS_REGISTER_NODE(ControlToJointPublisher)
