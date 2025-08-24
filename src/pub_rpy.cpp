#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <tuple>
#include <optional>

#include "ksenos_ground_msgs/msg/rpy.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/float32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/**
 * @brief IMUのTF情報からRPY角度と角速度を計算・配信するノード
 */
class RpyPublisherNode : public rclcpp::Node
{
public:
    RpyPublisherNode()
        : Node("rpy_publisher_node")
    {
        initializeParameters();
        initializeTfListeners();
        initializePublishers();
        setupTimer();

        RCLCPP_INFO(this->get_logger(), "RpyPublisherNode has been started.");
    }

private:
    // 定数定義
    static constexpr int TIMER_PERIOD_MS = 20; // 50Hz
    static constexpr double ANGULAR_VELOCITY_SCALE = 1000.0;
    static constexpr size_t HISTORY_SIZE_THRESHOLD = 3;
    static constexpr int QUEUE_SIZE = 10;

    /**
     * @brief パラメータを初期化
     */
    void initializeParameters()
    {
        this->declare_parameter<std::string>("output_angular_topic_name", "/rpy");
        this->declare_parameter<std::string>("output_angular_velocity_topic_name", "/angular_velocity");

        this->get_parameter("output_angular_topic_name", output_angular_topic_name_);
        this->get_parameter("output_angular_velocity_topic_name", output_angular_velocity_topic_name_);
    }

    /**
     * @brief TFリスナーを初期化
     */
    void initializeTfListeners()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // imu_linkが利用可能になるまで待機
        waitForTransform();
    }

    /**
     * @brief 必要なTransformが利用可能になるまで待機
     */
    void waitForTransform()
    {
        while (rclcpp::ok())
        {
            try
            {
                tf_buffer_->lookupTransform("imu_link_projected", "imu_link", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "imu_link is now available.");
                break;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for imu_link to become available: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }

    /**
     * @brief パブリッシャーを初期化
     */
    void initializePublishers()
    {
        rclcpp::QoS qos_settings(QUEUE_SIZE);
        qos_settings.reliable();

        // RPY関連のパブリッシャー
        rpy_pub_ = this->create_publisher<ksenos_ground_msgs::msg::Rpy>(output_angular_topic_name_, QUEUE_SIZE);
        angular_velocity_pub_ = this->create_publisher<ksenos_ground_msgs::msg::Rpy>(output_angular_velocity_topic_name_, QUEUE_SIZE);

        // UI用のパブリッシャー
        roll_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ui/roll", qos_settings);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ui/pitch", qos_settings);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ui/yaw", qos_settings);
        roll_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ui/roll_speed", qos_settings);
        pitch_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ui/pitch_speed", qos_settings);
        yaw_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ui/yaw_speed", qos_settings);
    }

    /**
     * @brief タイマーを設定
     */
    void setupTimer()
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(TIMER_PERIOD_MS),
            std::bind(&RpyPublisherNode::timer_callback, this));
    }
    /**
     * @brief タイマーコールバック関数
     */
    void timer_callback()
    {
        // Transform取得とRPY計算
        double roll, pitch, yaw;
        if (!getRpyFromTransforms(roll, pitch, yaw))
        {
            return; // Transform取得失敗時は処理終了
        }

        // RPYメッセージの配信
        publishRpyMessages(roll, pitch, yaw);

        // 履歴の更新
        updateHistory(roll, pitch, yaw);

        // 角速度の計算と配信
        calculateAndPublishAngularVelocity();
    }

    /**
     * @brief TransformからRPY角度を取得
     * @param roll 出力用Roll角度
     * @param pitch 出力用Pitch角度
     * @param yaw 出力用Yaw角度
     * @return 成功時true、失敗時false
     */
    bool getRpyFromTransforms(double &roll, double &pitch, double &yaw)
    {
        // Roll/Pitch用のTransform取得
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("imu_link_projected", "imu_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return false;
        }

        // Yaw用のTransform取得
        geometry_msgs::msg::TransformStamped transform_stamped_yaw;
        try
        {
            transform_stamped_yaw = tf_buffer_->lookupTransform("map", "imu_link_projected", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return false;
        }

        // QuaternionからRPYへの変換
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);

        tf2::Quaternion q_yaw(
            transform_stamped_yaw.transform.rotation.x,
            transform_stamped_yaw.transform.rotation.y,
            transform_stamped_yaw.transform.rotation.z,
            transform_stamped_yaw.transform.rotation.w);

        tf2::Matrix3x3 m(q);
        tf2::Matrix3x3 m_yaw(q_yaw);

        double unused_yaw;
        m.getRPY(roll, pitch, unused_yaw);

        double unused_roll, unused_pitch;
        m_yaw.getRPY(unused_roll, unused_pitch, yaw);

        // 飛行機の回転方向に合わせる(ピッチアップが正、右翼上げがロール正、時計回りがヨー正)
        roll = -roll;
        pitch = -pitch;
        yaw = -yaw;

        return true;
    }

    /**
     * @brief RPYメッセージを配信
     */
    void publishRpyMessages(double roll, double pitch, double yaw)
    {
        // RPYメッセージの作成と配信
        ksenos_ground_msgs::msg::Rpy rpy_msg;
        rpy_msg.roll = roll;
        rpy_msg.pitch = pitch;
        rpy_msg.yaw = yaw;
        rpy_msg.header.stamp = this->now();
        rpy_pub_->publish(rpy_msg);

        // 個別のFloat32メッセージの配信
        publishFloat32Message(roll_pub_, roll);
        publishFloat32Message(pitch_pub_, pitch);
        publishFloat32Message(yaw_pub_, yaw);
    }

    /**
     * @brief Float32メッセージを配信
     */
    void publishFloat32Message(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &publisher, double value)
    {
        std_msgs::msg::Float32 msg;
        msg.data = value;
        publisher->publish(msg);
    }

    /**
     * @brief 履歴を更新
     */
    void updateHistory(double roll, double pitch, double yaw)
    {
        roll_history_.push_back(roll);
        pitch_history_.push_back(pitch);
        yaw_history_.push_back(yaw);

        updateTimestamp();
    }

    /**
     * @brief タイムスタンプを更新
     */
    void updateTimestamp()
    {
        rclcpp::Time current_time = this->now();
        if (last_time_.seconds() == 0)
        {
            last_time_ = current_time;
            return;
        }

        rclcpp::Duration diff = current_time - last_time_;
        time_diff_ = diff;
        last_time_ = current_time;
    }

    /**
     * @brief 角速度を計算して配信
     */
    void calculateAndPublishAngularVelocity()
    {
        if (roll_history_.size() <= HISTORY_SIZE_THRESHOLD)
            return;

        // 中心差分法で角速度を計算
        double dt = 2.0 * time_diff_.seconds();
        double roll_diff = angle_diff(roll_history_[2], roll_history_[0]) / dt;
        double pitch_diff = angle_diff(pitch_history_[2], pitch_history_[0]) / dt;
        double yaw_diff = angle_diff(yaw_history_[2], yaw_history_[0]) / dt;

        // UI用角速度メッセージの配信（スケール適用）
        publishFloat32Message(roll_speed_pub_, roll_diff * ANGULAR_VELOCITY_SCALE);
        publishFloat32Message(pitch_speed_pub_, pitch_diff * ANGULAR_VELOCITY_SCALE);
        publishFloat32Message(yaw_speed_pub_, yaw_diff * ANGULAR_VELOCITY_SCALE);

        // 角速度メッセージの配信
        ksenos_ground_msgs::msg::Rpy angular_velocity_msg;
        angular_velocity_msg.roll = roll_diff;
        angular_velocity_msg.pitch = pitch_diff;
        angular_velocity_msg.yaw = yaw_diff;
        angular_velocity_pub_->publish(angular_velocity_msg);

        // 履歴をクリーンアップ
        cleanupHistory();
    }

    /**
     * @brief 履歴をクリーンアップ
     */
    void cleanupHistory()
    {
        roll_history_.pop_front();
        pitch_history_.pop_front();
        yaw_history_.pop_front();
    }

    // 角度の差を補正する関数
    double angle_diff(double angle1, double angle2)
    {
        double diff = angle1 - angle2;
        while (diff > M_PI)
            diff -= 2 * M_PI;
        while (diff < -M_PI)
            diff += 2 * M_PI;
        return diff;
    }

    // メンバ変数
    rclcpp::TimerBase::SharedPtr timer_;

    // TF関連
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // パブリッシャー
    rclcpp::Publisher<ksenos_ground_msgs::msg::Rpy>::SharedPtr rpy_pub_;
    rclcpp::Publisher<ksenos_ground_msgs::msg::Rpy>::SharedPtr angular_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_speed_pub_;

    // データ履歴
    std::deque<double> roll_history_;
    std::deque<double> pitch_history_;
    std::deque<double> yaw_history_;

    // 時間管理
    rclcpp::Time last_time_;
    rclcpp::Duration time_diff_ = rclcpp::Duration(0, 0);

    // パラメータ
    std::string output_angular_topic_name_;
    std::string output_angular_velocity_topic_name_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpyPublisherNode>());
    rclcpp::shutdown();
    return 0;
}