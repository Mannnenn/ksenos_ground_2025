/**
 * ESP32からWi-Fi（UDP）でセンサーデータを受信するC++プログラム
 * ROS2対応版・パフォーマンス最適化
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <std_msgs/msg/bool.hpp>

#include <ksenos_ground_msgs/msg/flow_rate_data.hpp>
#include <ksenos_ground_msgs/msg/pressure_data.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>

// センサーパケットのデータ構造
#pragma pack(push, 1)
struct SensorPacket
{
    uint8_t header[2];            // ヘッダー [0xAA, 0x55]
    uint32_t sequence;            // シーケンス番号
    uint64_t timestamp_millis;    // タイムスタンプ（ミリ秒）
    float acc_x, acc_y, acc_z;    // 加速度 (G、ROS2発行時にm/s²に変換)
    float gyro_x, gyro_y, gyro_z; // 角速度 (dps、ROS2発行時にrad/sに変換)
    float tof_distance;           // ToF距離 (mm)
    float baro_pressure;          // 気圧 (hPa)
    float flow_rate;              // 流量 (L/min)
    uint8_t servo_enable;         // サーボ有効フラグ
    uint8_t checksum;             // チェックサム
};
#pragma pack(pop)

class UDPReceiver : public rclcpp::Node
{
public:
    UDPReceiver() : Node("udp_sensor_receiver")
    {
        // パラメータの宣言
        // this->declare_parameter("local_ip", "192.168.11.2");
        this->declare_parameter("local_ip", "10.42.0.1");
        this->declare_parameter("local_port", 8888);
        this->declare_parameter("publish_rate", 100.0); // Hz

        // パラメータの取得
        local_ip_ = this->get_parameter("local_ip").as_string();
        local_port_ = this->get_parameter("local_port").as_int();
        // publish_rate は将来の機能拡張のために宣言されているが、現在は使用されていない
        // double publish_rate = this->get_parameter("publish_rate").as_double();

        // パブリッシャーの作成
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("sensor/imu", 10);
        tof_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("sensor/tof", 10);
        pressure_publisher_ = this->create_publisher<ksenos_ground_msgs::msg::PressureData>("sensor/pressure", 10);
        flow_rate_publisher_ = this->create_publisher<ksenos_ground_msgs::msg::FlowRateData>("sensor/flow_rate", 10);
        servo_enable_publisher_ = this->create_publisher<std_msgs::msg::Bool>("sensor/servo_enable", 10);

        // 統計変数の初期化
        packet_count_ = 0;
        last_sequence_ = 0;
        lost_packets_ = 0;
        valid_packets_ = 0;
        error_packets_ = 0;
        is_first_packet_ = true;

        // UDPソケットの初期化
        if (!initialize_socket())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
            return;
        }

        // タイマーの作成（非同期でUDPパケットを受信）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), // 1ms間隔でチェック
            std::bind(&UDPReceiver::receive_packets, this));

        RCLCPP_INFO(this->get_logger(), "UDP受信を開始しました: %s:%d",
                    local_ip_.c_str(), local_port_);
        RCLCPP_INFO(this->get_logger(), "ESP32からのデータを待機中...");
    }

    ~UDPReceiver()
    {
        if (socket_fd_ >= 0)
        {
            close(socket_fd_);
        }
    }

private:
    bool initialize_socket()
    {
        // UDPソケットの作成
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return false;
        }

        // ソケットバッファサイズを増やしてパケットロスを防ぐ
        int buffer_size = 65536;
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set socket buffer size");
        }

        // ソケットを非ブロッキングモードに設定
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        // アドレス構造体の設定
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = inet_addr(local_ip_.c_str());
        server_addr_.sin_port = htons(local_port_);

        // ソケットのバインド
        if (bind(socket_fd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket to %s:%d",
                         local_ip_.c_str(), local_port_);
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        return true;
    }

    void receive_packets()
    {
        char buffer[1024];
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        // 利用可能なパケットをすべて処理
        while (true)
        {
            ssize_t received_bytes = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                              (struct sockaddr *)&client_addr, &client_addr_len);

            if (received_bytes < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    // データが利用できない（正常）
                    break;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to receive UDP packet: %s", strerror(errno));
                    break;
                }
            }

            if (received_bytes == sizeof(SensorPacket))
            {
                process_packet(buffer, received_bytes, client_addr);
            }
            else
            {
                error_packets_++;
                RCLCPP_WARN(this->get_logger(), "Invalid packet size: %zd, expected: %zu",
                            received_bytes, sizeof(SensorPacket));
            }
        }
    }

    void process_packet(const char *data, size_t data_size, [[maybe_unused]] const struct sockaddr_in &client_addr)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        const SensorPacket *packet = reinterpret_cast<const SensorPacket *>(data);

        // ヘッダーの確認
        if (packet->header[0] != 0xAA || packet->header[1] != 0x55)
        {
            error_packets_++;
            RCLCPP_WARN(this->get_logger(), "Invalid header: [0x%02X, 0x%02X]",
                        packet->header[0], packet->header[1]);
            return;
        }

        // チェックサムの確認
        if (!verify_checksum(data, data_size))
        {
            error_packets_++;
            RCLCPP_WARN(this->get_logger(), "Checksum error in packet %u", packet->sequence);
            return;
        }

        packet_count_++;
        valid_packets_++;

        // パケットロスの検出
        if (!is_first_packet_)
        {
            uint32_t expected_sequence = (last_sequence_ + 1);
            if (packet->sequence != expected_sequence)
            {
                uint32_t lost = packet->sequence - expected_sequence;
                if (lost < 1000)
                { // 大きすぎる場合はオーバーフロー
                    lost_packets_ += lost;
                    RCLCPP_WARN(this->get_logger(), "*** %u packets lost ***", lost);
                }
            }
        }
        else
        {
            is_first_packet_ = false;
        }

        last_sequence_ = packet->sequence;

        // ROS2メッセージとして発行
        publish_sensor_data(*packet);

        // 処理時間の計算
        auto end_time = std::chrono::high_resolution_clock::now();
        auto process_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // 統計情報の表示（1000パケットごと）
        if (valid_packets_ % 1000 == 0)
        {
            display_statistics(process_time.count() / 1000.0);
        }
    }

    bool verify_checksum(const char *data, size_t data_size)
    {
        uint8_t calculated_checksum = 0;
        for (size_t i = 0; i < data_size - 1; i++)
        {
            calculated_checksum += static_cast<uint8_t>(data[i]);
        }
        calculated_checksum &= 0xFF;

        uint8_t received_checksum = static_cast<uint8_t>(data[data_size - 1]);
        return calculated_checksum == received_checksum;
    }

    rclcpp::Time convert_packet_timestamp_to_ros_time(uint64_t timestamp_millis)
    {
        // タイムスタンプがUNIXタイムベース（ミリ秒）の場合
        if (timestamp_millis > 1000000000000ULL)
        {
            // ミリ秒をナノ秒に変換してROS時刻に変換
            return rclcpp::Time(timestamp_millis * 1000000ULL);
        }
        else
        {
            // ESP32の起動からの相対時間の場合は現在時刻を使用
            return this->get_clock()->now();
        }
    }

    void publish_sensor_data(const SensorPacket &packet)
    {
        // 送信されてきたタイムスタンプを使用してROS時刻に変換
        rclcpp::Time sensor_time = convert_packet_timestamp_to_ros_time(packet.timestamp_millis);

        // IMUデータの発行
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = sensor_time;
        imu_msg.header.frame_id = "imu_link";

        // 加速度の単位変換: G → m/s² (1G = 9.80665 m/s²)
        // ICM42688P座標系からROS2座標系への変換（y軸を反転）
        imu_msg.linear_acceleration.x = packet.acc_x * 9.80665;
        imu_msg.linear_acceleration.y = packet.acc_y * 9.80665; // y軸反転
        imu_msg.linear_acceleration.z = packet.acc_z * 9.80665;

        // 角速度の単位変換: degrees/s → rad/s (π/180)
        // ICM42688P座標系からROS2座標系への変換（y軸を反転）
        imu_msg.angular_velocity.x = packet.gyro_x * M_PI / 180.0;
        imu_msg.angular_velocity.y = packet.gyro_y * M_PI / 180.0; // y軸反転
        imu_msg.angular_velocity.z = -packet.gyro_z * M_PI / 180.0;

        // 共分散行列は不明のため、すべて-1に設定
        for (int i = 0; i < 9; i++)
        {
            imu_msg.linear_acceleration_covariance[i] = -1.0;
            imu_msg.angular_velocity_covariance[i] = -1.0;
            imu_msg.orientation_covariance[i] = -1.0;
        }

        imu_publisher_->publish(imu_msg);

        // ToFセンサーデータの発行
        auto tof_msg = sensor_msgs::msg::Range();
        tof_msg.header.stamp = sensor_time;
        tof_msg.header.frame_id = "tof_link";
        tof_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        tof_msg.field_of_view = 0.471;                // 約27度（一般的なToFセンサー）
        tof_msg.min_range = 0.03;                     // 3cm
        tof_msg.max_range = 4.0;                      // 4m
        tof_msg.range = packet.tof_distance / 1000.0; // mmからmに変換

        tof_publisher_->publish(tof_msg);

        // 気圧データの発行
        auto pressure_msg = ksenos_ground_msgs::msg::PressureData();
        pressure_msg.header.stamp = sensor_time;
        pressure_msg.header.frame_id = "pressure_link";
        pressure_msg.pressure = packet.baro_pressure; // hPa
        pressure_publisher_->publish(pressure_msg);

        // 流量データの発行
        auto flow_msg = ksenos_ground_msgs::msg::FlowRateData();
        flow_msg.header.stamp = sensor_time;
        flow_msg.header.frame_id = "flow_rate_link";
        flow_msg.flow_rate = packet.flow_rate; // L/min
        flow_rate_publisher_->publish(flow_msg);

        // サーボ有効フラグの発行
        auto servo_msg = std_msgs::msg::Bool();
        servo_msg.data = packet.servo_enable != 0;
        servo_enable_publisher_->publish(servo_msg);
    }

    void display_statistics(double process_time_ms)
    {
        uint32_t total_packets = valid_packets_ + lost_packets_;
        double loss_rate = total_packets > 0 ? (lost_packets_ * 100.0 / total_packets) : 0.0;

        RCLCPP_INFO(this->get_logger(),
                    "*** 統計: 有効: %u, エラー: %u, "
                    "ロス: %u (%.2f%%), "
                    "処理時間: %.2fms ***",
                    valid_packets_, error_packets_, lost_packets_, loss_rate, process_time_ms);
    }

    // メンバ変数
    std::string local_ip_;
    int local_port_;
    int socket_fd_;
    struct sockaddr_in server_addr_;

    // ROS2パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_publisher_;
    rclcpp::Publisher<ksenos_ground_msgs::msg::PressureData>::SharedPtr pressure_publisher_;
    rclcpp::Publisher<ksenos_ground_msgs::msg::FlowRateData>::SharedPtr flow_rate_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_enable_publisher_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;

    // 統計変数
    uint32_t packet_count_;
    uint32_t last_sequence_;
    uint32_t lost_packets_;
    uint32_t valid_packets_;
    uint32_t error_packets_;
    bool is_first_packet_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "ESP32 センサーデータ UDP受信プログラム（C++/ROS2版）");
    RCLCPP_INFO(rclcpp::get_logger("main"), "パフォーマンス最適化:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - 非ブロッキングUDP受信");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - ソケットバッファサイズ拡張");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - 処理時間監視");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - ROS2メッセージ発行");

    auto node = std::make_shared<UDPReceiver>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}