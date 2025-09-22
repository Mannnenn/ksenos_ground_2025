#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ksenos_ground_msgs/msg/sbus_data.hpp>

class SbusOffsetApply : public rclcpp::Node
{
public:
    SbusOffsetApply(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("sbus_offset_apply_node", options)
    {
        // パラメータ宣言
        this->declare_parameter("input_topic", "sbus_raw");
        this->declare_parameter("offset_topic", "sbus_offset_values");
        this->declare_parameter("output_topic", "sbus_offset");
        this->declare_parameter("offset_operation", "subtract"); // "subtract" or "add"

        // パラメータ取得
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string offset_topic = this->get_parameter("offset_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string offset_operation = this->get_parameter("offset_operation").as_string();

        // オフセット操作モードの検証と設定
        if (offset_operation == "subtract" || offset_operation == "add")
        {
            offset_operation_ = offset_operation;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid offset_operation parameter '%s'. Using default 'subtract'.",
                        offset_operation.c_str());
            offset_operation_ = "subtract";
        }

        // 初期化
        has_offset_ = false;
        aileron_r_offset_ = 0.0;
        elevator_offset_ = 0.0;
        rudder_offset_ = 0.0;
        aileron_l_offset_ = 0.0;

        // Subscribers and Publisher
        sbus_sub_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            input_topic, 10,
            std::bind(&SbusOffsetApply::sbus_callback, this, std::placeholders::_1));

        offset_sub_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            offset_topic, 10,
            std::bind(&SbusOffsetApply::offset_callback, this, std::placeholders::_1));

        sbus_pub_ = this->create_publisher<ksenos_ground_msgs::msg::SbusData>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "SBUS Offset Apply Node started.");
        RCLCPP_INFO(this->get_logger(), "Offset operation mode: %s", offset_operation_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for offset values...");
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        if (!has_offset_)
        {
            // オフセット値がまだ受信されていない場合は何もしない
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for offset values from calibration node...");
            return;
        }

        // オフセット処理済みのメッセージを作成
        auto offset_msg = std::make_shared<ksenos_ground_msgs::msg::SbusData>(*msg);

        // オフセット適用
        if (offset_operation_ == "add")
        {
            offset_msg->aileron_r += aileron_r_offset_;
            offset_msg->elevator += elevator_offset_;
            offset_msg->rudder += rudder_offset_;
            offset_msg->aileron_l += aileron_l_offset_;
        }
        else
        { // default is "subtract"
            offset_msg->aileron_r -= aileron_r_offset_;
            offset_msg->elevator -= elevator_offset_;
            offset_msg->rudder -= rudder_offset_;
            offset_msg->aileron_l -= aileron_l_offset_;
        }

        offset_msg->dropping_device = msg->dropping_device; // ドロップ信号はそのままコピー

        // パブリッシュ
        sbus_pub_->publish(*offset_msg);
    }

    void offset_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // オフセット値を更新
        aileron_r_offset_ = msg->aileron_r;
        elevator_offset_ = msg->elevator;
        rudder_offset_ = msg->rudder;
        aileron_l_offset_ = msg->aileron_l;

        if (!has_offset_)
        {
            has_offset_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Offset values received and applied (operation: %s):", offset_operation_.c_str());
            RCLCPP_INFO(this->get_logger(),
                        "  aileron_r: %.4f, elevator: %.4f, rudder: %.4f, aileron_l: %.4f",
                        aileron_r_offset_, elevator_offset_, rudder_offset_, aileron_l_offset_);
        }
    }

    // Subscribers and Publisher
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_sub_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr offset_sub_;
    rclcpp::Publisher<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_pub_;

    // オフセット関連
    bool has_offset_;
    double aileron_r_offset_;
    double elevator_offset_;
    double rudder_offset_;
    double aileron_l_offset_;
    std::string offset_operation_; // "subtract" or "add"
};

RCLCPP_COMPONENTS_REGISTER_NODE(SbusOffsetApply)