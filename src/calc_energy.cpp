#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "ksenos_ground_msgs/msg/flow_rate_data.hpp"
#include "ksenos_ground_msgs/msg/plane_energy.hpp"

#include <memory>

class CalcEnergyNode : public rclcpp::Node
{
public:
    CalcEnergyNode() : Node("calc_energy_node")
    {
        // パラメータ宣言（質量m、重力加速度g、トピック名）
        this->declare_parameter("mass", 1.0);
        this->declare_parameter("gravity", 9.81);
        this->declare_parameter("velocity_topic", "velocity");
        this->declare_parameter("altitude_topic", "altitude");
        this->declare_parameter("energy_topic", "kinetic_energy");

        mass_ = this->get_parameter("mass").as_double();
        gravity_ = this->get_parameter("gravity").as_double();
        std::string velocity_topic = this->get_parameter("velocity_topic").as_string();
        std::string altitude_topic = this->get_parameter("altitude_topic").as_string();
        std::string energy_topic = this->get_parameter("energy_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "質量: %.2f kg, 重力加速度: %.2f m/s²", mass_, gravity_);
        RCLCPP_INFO(this->get_logger(), "速度トピック: %s, 高度トピック: %s", velocity_topic.c_str(), altitude_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "エネルギートピック: %s", energy_topic.c_str());

        // サブスクライバー
        velocity_sub_ = this->create_subscription<ksenos_ground_msgs::msg::FlowRateData>(
            velocity_topic, 10,
            std::bind(&CalcEnergyNode::velocity_callback, this, std::placeholders::_1));

        altitude_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            altitude_topic, 10,
            std::bind(&CalcEnergyNode::altitude_callback, this, std::placeholders::_1));

        // パブリッシャー
        energy_pub_ = this->create_publisher<ksenos_ground_msgs::msg::PlaneEnergy>(energy_topic, 10);

        // 初期化
        velocity_ = 0.0;
        altitude_ = 0.0;
        has_velocity_ = false;
        has_altitude_ = false;

        RCLCPP_INFO(this->get_logger(), "エネルギー計算ノードが開始されました");
    }

private:
    void velocity_callback(const ksenos_ground_msgs::msg::FlowRateData::SharedPtr msg)
    {
        velocity_ = msg->flow_rate; // Assuming flow_rate is in m/s
        has_velocity_ = true;

        // 両方のデータが揃った場合はエネルギーを計算・パブリッシュ
        if (has_velocity_ && has_altitude_)
        {
            calculate_and_publish_energy();
        }
    }

    void altitude_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        altitude_ = msg->data;
        has_altitude_ = true;

        // 両方のデータが揃った場合はエネルギーを計算・パブリッシュ
        if (has_velocity_ && has_altitude_)
        {
            calculate_and_publish_energy();
        }
    }

    void calculate_and_publish_energy()
    {
        // 運動エネルギー: K = 1/2 * m * v²
        float kinetic_energy = 0.5 * mass_ * velocity_ * velocity_;

        // 位置エネルギー: U = mgh
        float potential_energy = mass_ * gravity_ * altitude_;

        // 総エネルギー: E = K + U
        float total_energy = kinetic_energy + potential_energy;

        // メッセージ作成・パブリッシュ
        auto energy_msg = ksenos_ground_msgs::msg::PlaneEnergy();
        energy_msg.kinetic_energy = kinetic_energy;
        energy_msg.potential_energy = potential_energy;
        energy_msg.total_energy = total_energy;

        energy_pub_->publish(energy_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "速度: %.2f m/s, 高度: %.2f m, 運動エネルギー: %.2f J, 位置エネルギー: %.2f J, 総エネルギー: %.2f J",
                     velocity_, altitude_, kinetic_energy, potential_energy, total_energy);
    }

    // サブスクライバー
    rclcpp::Subscription<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_sub_;

    // パブリッシャー
    rclcpp::Publisher<ksenos_ground_msgs::msg::PlaneEnergy>::SharedPtr energy_pub_;

    // パラメータ
    double mass_;
    double gravity_;

    // データ保存用変数
    float velocity_;
    float altitude_;
    bool has_velocity_;
    bool has_altitude_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalcEnergyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}