#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ksenos_ground_msgs/msg/sbus_data.hpp"
#include "ksenos_ground_msgs/msg/flow_rate_data.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <chrono>
#include <string>

class ModeAutoLandingBackup : public rclcpp::Node
{
public:
    ModeAutoLandingBackup(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("mode_auto_landing_backup", options),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          previous_mode_(""),
          is_auto_landing_mode_(false),
          tf_check_completed_(false),
          backup_mode_active_(false)
    {
        // パラメータの宣言と初期値設定
        this->declare_parameter<std::string>("world_frame", "map");
        this->declare_parameter<std::string>("base_frame", "ksenos_smooth0");
        this->declare_parameter<double>("tf_check_timeout", 5.0); // 5秒待機
        this->declare_parameter<double>("v_max", 5.0);            // 最大速度 [m/s]

        // パラメータの取得
        world_frame_ = this->get_parameter("world_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        tf_check_timeout_ = this->get_parameter("tf_check_timeout").as_double();
        v_max_ = this->get_parameter("v_max").as_double();

        // パブリッシャーの初期化
        speed_publisher_ = this->create_publisher<ksenos_ground_msgs::msg::FlowRateData>(
            "target_speed", 10);

        // サブスクライバーの初期化
        sbus_subscriber_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_data", 10,
            std::bind(&ModeAutoLandingBackup::sbus_callback, this, std::placeholders::_1));

        // TFチェック用タイマー（最初は停止状態）
        tf_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ModeAutoLandingBackup::tf_check_callback, this));
        tf_check_timer_->cancel();

        // 速度パブリッシュ用タイマー（最初は停止状態）
        speed_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ModeAutoLandingBackup::speed_publish_callback, this));
        speed_publish_timer_->cancel();

        RCLCPP_INFO(this->get_logger(),
                    "Mode Auto Landing Backup node initialized. Waiting for mode switch to auto_landing.");
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        std::string current_mode = msg->autopilot_mode;

        // manualからauto_landingに切り替わったときを検知
        if (previous_mode_ == "manual" && current_mode == "auto_landing")
        {
            RCLCPP_INFO(this->get_logger(),
                        "Mode switched from manual to auto_landing. Starting TF availability check.");

            // TFチェックの開始
            is_auto_landing_mode_ = true;
            tf_check_completed_ = false;
            backup_mode_active_ = false;
            tf_check_start_time_ = this->now();

            // TFチェックタイマーを開始
            tf_check_timer_->reset();
        }
        else if (current_mode != "auto_landing")
        {
            // auto_landingモード以外になった場合、すべてリセット
            if (is_auto_landing_mode_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Mode switched away from auto_landing. Stopping backup system.");
                reset_backup_system();
            }
            is_auto_landing_mode_ = false;
        }

        previous_mode_ = current_mode;
    }

    void tf_check_callback()
    {
        if (!is_auto_landing_mode_ || tf_check_completed_)
        {
            tf_check_timer_->cancel();
            return;
        }

        // タイムアウトチェック
        rclcpp::Duration elapsed = this->now() - tf_check_start_time_;
        if (elapsed.seconds() >= tf_check_timeout_)
        {
            // タイムアウト：TFが利用できないのでバックアップモードを開始
            RCLCPP_WARN(this->get_logger(),
                        "TF %s->%s not available after %.1f seconds. Starting backup speed control.",
                        world_frame_.c_str(), base_frame_.c_str(), tf_check_timeout_);

            start_backup_mode();
            return;
        }

        // TFの可用性をチェック
        try
        {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_.lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);

            // TFが利用可能
            RCLCPP_INFO(this->get_logger(),
                        "TF %s->%s is available. Backup system not needed.",
                        world_frame_.c_str(), base_frame_.c_str());

            tf_check_completed_ = true;
            tf_check_timer_->cancel();
        }
        catch (const tf2::TransformException &ex)
        {
            // TFがまだ利用できない、チェックを継続
            RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    void start_backup_mode()
    {
        tf_check_completed_ = true;
        backup_mode_active_ = true;
        tf_check_timer_->cancel();

        // 速度パブリッシュタイマーを開始
        speed_publish_timer_->reset();

        RCLCPP_INFO(this->get_logger(), "Backup speed control mode activated.");
    }

    void speed_publish_callback()
    {
        if (!backup_mode_active_)
        {
            speed_publish_timer_->cancel();
            return;
        }

        ksenos_ground_msgs::msg::FlowRateData speed_msg;
        speed_msg.flow_rate = static_cast<float>(v_max_);
        speed_publisher_->publish(speed_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published backup speed: %.2f m/s", v_max_);
    }

    void reset_backup_system()
    {
        tf_check_timer_->cancel();
        speed_publish_timer_->cancel();
        tf_check_completed_ = false;
        backup_mode_active_ = false;
    }

    // メンバ変数
    rclcpp::Publisher<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr speed_publisher_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscriber_;

    rclcpp::TimerBase::SharedPtr tf_check_timer_;
    rclcpp::TimerBase::SharedPtr speed_publish_timer_;

    // TF関連
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // パラメータ
    std::string world_frame_;
    std::string base_frame_;
    double tf_check_timeout_;
    double v_max_;

    // 状態管理
    std::string previous_mode_;
    bool is_auto_landing_mode_;
    bool tf_check_completed_;
    bool backup_mode_active_;
    rclcpp::Time tf_check_start_time_;
};

// コンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(ModeAutoLandingBackup)
