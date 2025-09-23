#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

// Linux serial functions
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class ImuLevelingNode : public rclcpp::Node
{
public:
    ImuLevelingNode() : Node("imu_leveling_node"), serial_fd_(-1), is_calibrated_(false)
    {
        // パラメータの宣言と取得
        this->declare_parameter("device_name", "/dev/ttyIMU");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("base_frame", "map");
        this->declare_parameter("level_frame", "motor_base");
        this->declare_parameter("calibration_time_sec", 5.0);
        this->declare_parameter("publish_rate_hz", 50.0);

        device_name_ = this->get_parameter("device_name").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        base_frame_ = this->get_parameter("base_frame").as_string();
        level_frame_ = this->get_parameter("level_frame").as_string();
        calibration_time_sec_ = this->get_parameter("calibration_time_sec").as_double();
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

        // TFブロードキャスターの作成
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 平均値初期化
        accel_sum_ = {0.0, 0.0, 0.0};
        sample_count_ = 0;
        level_roll_ = 0.0;
        level_pitch_ = 0.0;

        // シリアルポートの設定
        serial_fd_ = openSerial(device_name_.c_str());

        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "シリアルポートを開けませんでした: %s", device_name_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "シリアルポートが正常に開かれました: %s", device_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "キャリブレーション開始: %.1f秒間のデータを収集します", calibration_time_sec_);

        // キャリブレーション開始時刻を記録
        calibration_start_time_ = this->get_clock()->now();

        // タイマーでシリアルデータを読み込み
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ImuLevelingNode::readSerialData, this));

        // TF配信タイマー（キャリブレーション完了後に有効）
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_)),
            std::bind(&ImuLevelingNode::publishLevelTransform, this));

        RCLCPP_INFO(this->get_logger(), "IMU Leveling ノードが開始されました");
    }

    ~ImuLevelingNode()
    {
        // タイマーをキャンセル
        if (timer_)
        {
            timer_->cancel();
        }
        if (tf_timer_)
        {
            tf_timer_->cancel();
        }

        // シリアルポートを閉じる
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

private:
    int openSerial(const char *device_name)
    {
        int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd < 0)
        {
            return -1;
        }

        // ノンブロッキングモードを維持
        fcntl(fd, F_SETFL, O_NONBLOCK);
        struct termios conf_tio;
        tcgetattr(fd, &conf_tio);

        // ボーレート設定
        speed_t baudrate;
        switch (baud_rate_)
        {
        case 9600:
            baudrate = B9600;
            break;
        case 19200:
            baudrate = B19200;
            break;
        case 38400:
            baudrate = B38400;
            break;
        case 57600:
            baudrate = B57600;
            break;
        case 115200:
            baudrate = B115200;
            break;
        case 230400:
            baudrate = B230400;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "サポートされていないボーレート: %d, 115200を使用します", baud_rate_);
            baudrate = B115200;
            break;
        }

        cfsetispeed(&conf_tio, baudrate);
        cfsetospeed(&conf_tio, baudrate);

        // シリアル設定
        conf_tio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
        conf_tio.c_iflag &= ~(ICRNL | INPCK | ISTRIP | IXON | BRKINT);
        conf_tio.c_oflag &= ~OPOST;
        conf_tio.c_cflag |= CS8;
        conf_tio.c_cc[VMIN] = 0;
        conf_tio.c_cc[VTIME] = 0; // 完全ノンブロッキング

        tcsetattr(fd, TCSANOW, &conf_tio);
        return fd;
    }

    void readSerialData()
    {
        // キャリブレーション完了後は何もしない
        if (is_calibrated_ || serial_fd_ < 0)
        {
            return;
        }

        char buf[512] = {0};
        static std::string line_buffer;
        line_buffer.reserve(1024);

        // 利用可能なデータを一度に読み込み
        int recv_data = read(serial_fd_, buf, sizeof(buf) - 1);
        if (recv_data <= 0)
        {
            return;
        }

        // 受信データをバッファに追加
        line_buffer.append(buf, recv_data);

        // 複数行を一度に処理
        size_t start_pos = 0;
        size_t newline_pos;

        while ((newline_pos = line_buffer.find('\n', start_pos)) != std::string::npos)
        {
            std::string line = line_buffer.substr(start_pos, newline_pos - start_pos);
            start_pos = newline_pos + 1;

            // 空行をスキップ
            if (line.empty())
                continue;

            // 改行文字を削除
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }

            // データ処理
            processLine(line);
        }

        // 処理済みの行をバッファから削除
        if (start_pos > 0)
        {
            line_buffer.erase(0, start_pos);
        }

        // バッファが大きくなりすぎた場合はクリア
        if (line_buffer.size() > 2048)
        {
            line_buffer.clear();
            RCLCPP_WARN(this->get_logger(), "バッファをクリアしました");
        }
    }

    void processLine(const std::string &line)
    {
        // タブまたは空白文字で分割してdoubleに変換
        std::vector<double> values;
        values.reserve(7);

        std::istringstream iss(line);
        std::string token;

        while (iss >> token)
        {
            try
            {
                double value = std::stod(token);
                values.push_back(value);
            }
            catch (const std::exception &)
            {
                return; // 変換エラーの場合は処理を中断
            }
        }

        // データ数チェック（7個の値：accel_xyz, gyro_xyz, temp?）
        if (values.size() != 7)
        {
            return;
        }

        // キャリブレーション中の場合、加速度データを蓄積
        if (!is_calibrated_)
        {
            collectCalibrationData(values);
        }
    }

    void collectCalibrationData(const std::vector<double> &values)
    {
        // 加速度データ（最初の3つの値）を加算
        accel_sum_[0] += values[0];
        accel_sum_[1] += values[1];
        accel_sum_[2] += -values[2];
        sample_count_++;

        // キャリブレーション時間をチェック
        rclcpp::Time current_time = this->get_clock()->now();
        double elapsed_sec = (current_time - calibration_start_time_).seconds();

        if (elapsed_sec >= calibration_time_sec_)
        {
            completeCalibration();
        }
    }

    void completeCalibration()
    {
        if (sample_count_ == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "キャリブレーション中にデータが取得できませんでした");
            return;
        }

        // 平均加速度を計算
        double avg_accel_x = accel_sum_[0] / sample_count_;
        double avg_accel_y = accel_sum_[1] / sample_count_;
        double avg_accel_z = accel_sum_[2] / sample_count_;

        // 重力ベクトルから水平を計算
        // ロール角とピッチ角を計算（重力方向を-Z方向とする）
        level_roll_ = atan2(avg_accel_y, avg_accel_z);
        level_pitch_ = atan2(-avg_accel_x, sqrt(avg_accel_y * avg_accel_y + avg_accel_z * avg_accel_z));

        is_calibrated_ = true;

        // キャリブレーション完了後、シリアルデータ読み取りタイマーを停止
        timer_->cancel();
        timer_.reset();

        // シリアルポートを閉じる
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
        }

        RCLCPP_INFO(this->get_logger(),
                    "キャリブレーション完了: %d サンプル, Roll: %.3f deg, Pitch: %.3f deg",
                    sample_count_,
                    level_roll_ * 180.0 / M_PI,
                    level_pitch_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "シリアルデータ読み取りを停止しました。TF配信のみを継続します。");
    }

    void publishLevelTransform()
    {
        if (!is_calibrated_)
        {
            return;
        }

        // 水平化のための逆変換を適用
        tf2::Quaternion q;
        q.setRPY(-level_roll_, level_pitch_, 0.0);

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = base_frame_;
        transform_stamped.child_frame_id = level_frame_;
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.15;
        transform_stamped.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    // メンバ変数
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    int serial_fd_;
    std::string device_name_;
    int baud_rate_;
    std::string base_frame_;
    std::string level_frame_;
    double calibration_time_sec_;
    double publish_rate_hz_;

    bool is_calibrated_;
    rclcpp::Time calibration_start_time_;
    std::array<double, 3> accel_sum_;
    int sample_count_;
    double level_roll_;
    double level_pitch_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuLevelingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
