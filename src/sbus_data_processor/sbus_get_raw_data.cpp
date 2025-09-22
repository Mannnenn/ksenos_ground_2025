#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ksenos_ground_msgs/msg/sbus_raw_data.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>
#include <sstream>

// Linux serial functions
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class SbusSerialReader : public rclcpp::Node
{
public:
    SbusSerialReader(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("sbus_serial_reader", options), serial_fd_(-1)
    {
        // パブリッシャーの作成（キューサイズを増やして高頻度パブリッシュに対応）
        publisher_ = this->create_publisher<ksenos_ground_msgs::msg::SbusRawData>(
            "sbus_raw_data", 10);

        // シリアルポートの設定
        const char *device_name = "/dev/ttyESP32S3";
        serial_fd_ = openSerial(device_name);

        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "シリアルポートを開けませんでした: %s", device_name);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "シリアルポートが正常に開かれました: %s", device_name);

        // タイマーでシリアルデータを読み込み (2ms間隔で高頻度読み込み)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SbusSerialReader::readSerialData, this));

        RCLCPP_INFO(this->get_logger(), "SBUS Serial Reader ノードが開始されました");
    }

    ~SbusSerialReader()
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
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

        speed_t BAUDRATE = B230400;
        cfsetispeed(&conf_tio, BAUDRATE);
        cfsetospeed(&conf_tio, BAUDRATE);

        // より効率的なシリアル設定
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
        if (serial_fd_ < 0)
        {
            return;
        }

        char buf[512] = {0}; // バッファサイズを増加
        static std::string line_buffer;
        line_buffer.reserve(1024); // メモリ再割り当てを減らす

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

            // データ処理を試行（成功・失敗に関わらず続行）
            processLine(line);
        }

        // 処理済みの行をバッファから削除
        if (start_pos > 0)
        {
            line_buffer.erase(0, start_pos);
        }

        // バッファが大きくなりすぎた場合はクリア（メモリリーク防止）
        if (line_buffer.size() > 2048)
        {
            line_buffer.clear();
            RCLCPP_WARN(this->get_logger(), "バッファをクリアしました");
        }
    }

    void processLine(const std::string &line)
    {
        // 空白文字で分割して整数に変換
        std::vector<uint16_t> numbers;
        numbers.reserve(18); // メモリ再割り当てを避ける

        std::istringstream iss(line);
        std::string token;

        while (iss >> token)
        {
            try
            {
                int value = std::stoi(token);
                if (value < 0 || value > 65535)
                {
                    return; // 無効なデータの場合は処理を中断
                }
                numbers.push_back(static_cast<uint16_t>(value));
            }
            catch (const std::exception &)
            {
                return; // 変換エラーの場合は処理を中断
            }
        }

        // データ数チェック
        if (numbers.size() != 18)
        {
            return; // データ数が正しくない場合は処理を中断
        }

        // SbusRawDataメッセージの作成と配信
        publishSbusData(numbers);
    }

    void publishSbusData(const std::vector<uint16_t> &numbers)
    {
        auto msg = ksenos_ground_msgs::msg::SbusRawData();

        // ヘッダーの設定
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "sbus_frame";

        // データの格納（メッセージ定義の順序に従って）
        msg.aileron_r = numbers[0];
        msg.elevator = numbers[1];
        msg.throttle = numbers[2];
        msg.rudder = numbers[3];
        msg.button1 = numbers[4];
        msg.aileron_l = numbers[5];
        msg.is_autopilot = numbers[6];
        msg.dropping_device = numbers[7];
        msg.is_autolanding_enabled = numbers[8];
        msg.turning_mode = numbers[9];

        // 終端のデータ
        msg.is_lost_frame = numbers[16];
        msg.is_failsafe = numbers[17];

        // メッセージの配信
        publisher_->publish(msg);
    }

    rclcpp::Publisher<ksenos_ground_msgs::msg::SbusRawData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_fd_;
};

// コンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(SbusSerialReader)