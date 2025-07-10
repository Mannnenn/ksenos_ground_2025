#include "rclcpp/rclcpp.hpp"
#include "ksenos_ground_msgs/msg/sbus_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class ControlToJointPublisher : public rclcpp::Node
{
public:
    ControlToJointPublisher()
        : Node("control_to_joint_publisher"), propeller_angle_(0.0)
    {
        last_update_time_ = this->now();

        subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_offset", 10, std::bind(&ControlToJointPublisher::sbus_data_callback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

private:
    void sbus_data_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
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

    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    double propeller_angle_;
    rclcpp::Time last_update_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlToJointPublisher>());
    rclcpp::shutdown();
    return 0;
}