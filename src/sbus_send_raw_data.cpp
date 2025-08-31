#include <rclcpp/rclcpp.hpp>
#include <ksenos_ground_msgs/msg/sbus_raw_data.hpp>
#include <ksenos_ground_msgs/msg/sbus_data.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

class SbusUdpSender : public rclcpp::Node
{
public:
    SbusUdpSender() : Node("sbus_udp_sender"), is_manual_mode_(true)
    {
        // パラメータの宣言
        this->declare_parameter("udp_ip", "10.42.0.2");
        this->declare_parameter("udp_port", 9999);

        // パラメータの取得
        udp_ip_ = this->get_parameter("udp_ip").as_string();
        udp_port_ = this->get_parameter("udp_port").as_int();

        // UDPソケットの初期化
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
            return;
        }

        // サーバーアドレスの設定
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(udp_port_);
        inet_pton(AF_INET, udp_ip_.c_str(), &server_addr_.sin_addr);

        // サブスクライバーの作成
        subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusRawData>(
            "sbus_raw_data", 10,
            std::bind(&SbusUdpSender::sbus_raw_data_callback, this, std::placeholders::_1));

        // SbusDataのサブスクライバー（モード管理用）
        sbus_subscription_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "sbus_manual", 10,
            std::bind(&SbusUdpSender::sbus_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "SBUS UDP Sender started. Sending to %s:%d",
                    udp_ip_.c_str(), udp_port_);
        RCLCPP_INFO(this->get_logger(), "Only sends data when autopilot_mode is NOT 'manual'");
    }

    ~SbusUdpSender()
    {
        if (sock_ >= 0)
        {
            close(sock_);
        }
    }

private:
    void sbus_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // autopilot_modeが"manual"でない場合のみ送信を許可
        is_manual_mode_ = (msg->autopilot_mode == "manual");

        if (!is_manual_mode_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Auto mode detected. Ready to send UDP data.");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Manual mode detected (current: %s). UDP sending disabled.",
                         msg->autopilot_mode.c_str());
        }
    }

    void sbus_raw_data_callback(const ksenos_ground_msgs::msg::SbusRawData::SharedPtr msg)
    {
        // manual モードでない場合のみデータを送信
        if (is_manual_mode_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Manual mode active. UDP sending disabled.");
            return;
        }

        // 6チャンネルのデータを準備（aileron_r, aileron_l, elevator, throttle, rudder, dropping_device）
        uint16_t channels[6];
        channels[0] = htons(msg->aileron_r);       // チャンネル1: aileron_r
        channels[1] = htons(msg->aileron_l);       // チャンネル2: aileron_l
        channels[2] = htons(msg->elevator);        // チャンネル3: elevator
        channels[3] = htons(msg->throttle);        // チャンネル4: throttle
        channels[4] = htons(msg->rudder);          // チャンネル5: rudder
        channels[5] = htons(msg->dropping_device); // チャンネル6: dropping_device

        // UDPでデータを送信
        ssize_t sent_bytes = sendto(sock_, channels, sizeof(channels), 0,
                                    (struct sockaddr *)&server_addr_, sizeof(server_addr_));

        if (sent_bytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send UDP data: %s", strerror(errno));
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Sent RC channels - aileron_r: %u, aileron_l: %u, elevator: %u, "
                         "throttle: %u, rudder: %u, dropping_device: %u",
                         msg->aileron_r, msg->aileron_l, msg->elevator,
                         msg->throttle, msg->rudder, msg->dropping_device);
        }
    }

    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusRawData>::SharedPtr subscription_;
    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_subscription_;
    int sock_;
    struct sockaddr_in server_addr_;
    std::string udp_ip_;
    int udp_port_;
    bool is_manual_mode_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SbusUdpSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}