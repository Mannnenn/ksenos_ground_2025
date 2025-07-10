#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ksenos_ground_msgs/msg/sbus_data.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>

using namespace std::chrono_literals;
using boost::numeric::odeint::runge_kutta4;
using state_type = std::array<double, 12>; // 0-3: longitudinal, 4-8: lateral, 9-11: position

// 定数パラメータ
namespace Param
{
    // 基本定数
    constexpr double g = 9.81;
    constexpr double U0 = 3.83957;
    constexpr double theta0 = 6.69947 * M_PI / 180.0; // 初期ピッチ角（ラジアン）

    // 縦の安定微係数: キセノス
    constexpr double Xu = -0.085105763;
    constexpr double Xw = 0.332310218;
    constexpr double Zu = -1.214609154;
    constexpr double Zw = -2.398803902;
    constexpr double Zq = -0.353214251;
    constexpr double Mu = -2.80647E-07;
    constexpr double Mw = -0.008513734;
    constexpr double Mq = -0.063500393;

    constexpr double Xde = -0.106129818; // エレベーター
    constexpr double Zde = -0.903110858; // エレベーター
    constexpr double Mde = -0.386900555; // エレベーター
    constexpr double Xdt = 0.1;          // スロットル
    constexpr double Zdt = 0;            // スロットル
    constexpr double Mdt = 0;            // スロットル

    // 横の安定微係数: キセノス
    constexpr double Yv = -0.14479;
    constexpr double Yp = -0.055776;
    constexpr double Yr = 0.10746;
    constexpr double Lv = -0.083652;
    constexpr double Lp = -0.166;
    constexpr double Lr = 0.10719;
    constexpr double Nv = 0.022437;
    constexpr double Np = -0.06557;
    constexpr double Nr = -0.023672;

    constexpr double Yda = 0.45311;      // エルロン
    constexpr double Lda = 1.3608;       // エルロン
    constexpr double Nda = 0.14392;      // エルロン
    constexpr double Ydr = 0.322163839;  // ラダー
    constexpr double Ldr = -0.00469461;  // ラダー
    constexpr double Ndr = -0.152660157; // ラダー
} // namespace Param

// 回転行列 R = Rz(psi)*Ry(theta)*Rx(phi)
Eigen::Matrix3d rotation_matrix(double psi, double theta, double phi)
{
    Eigen::Matrix3d Rz, Ry, Rx;
    Rz << std::cos(psi), -std::sin(psi), 0,
        std::sin(psi), std::cos(psi), 0,
        0, 0, 1;
    Ry << std::cos(theta), 0, std::sin(theta),
        0, 1, 0,
        -std::sin(theta), 0, std::cos(theta);
    Rx << 1, 0, 0,
        0, std::cos(phi), -std::sin(phi),
        0, std::sin(phi), std::cos(phi);
    return Rz * Ry * Rx;
}

// 運動方程式クラス（Boost Odeint 用）
class AircraftDynamics
{
public:
    AircraftDynamics()
    {
        // A_long (4x4)
        A_long_.resize(4, 4);
        A_long_ << Param::Xu, Param::Xw, 0, -Param::g,
            Param::Zu, Param::Zw, Param::U0, 0,
            Param::Mu, Param::Mw, Param::Mq, 0,
            0, 0, 1, 0;

        // B_long (4x2)
        B_long_.resize(4, 2);
        B_long_ << Param::Xde, Param::Xdt,
            Param::Zde, Param::Zdt,
            Param::Mde, Param::Mdt,
            0, 0;

        // A_lat (5x5)
        A_lat_.resize(5, 5);
        A_lat_ << Param::Yv, Param::Yp, Param::Yr - Param::U0, Param::g, 0,
            Param::Lv, Param::Lp, Param::Lr, 0, 0,
            Param::Nv, Param::Np, Param::Nr, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0;

        // B_lat (5x2)
        B_lat_.resize(5, 2);
        B_lat_ << Param::Yda, Param::Ydr,
            Param::Lda, Param::Ldr,
            Param::Nda, Param::Ndr,
            0, 0,
            0, 0;
    }

    // ROS2 から更新される制御入力
    // u_long: [delta_e, delta_t], u_lat: [delta_a, delta_r]
    void set_sbus_datas(double delta_e, double delta_t, double delta_a, double delta_r)
    {
        u_long_[0] = delta_e;
        u_long_[1] = delta_t;
        u_lat_[0] = delta_a;
        u_lat_[1] = delta_r;
        // std::cout << "Control inputs: " << u_long_[0] << ", " << u_long_[1] << ", " << u_lat_[0] << ", " << u_lat_[1] << std::endl;
    }

    // ODE用評価関数 operator():
    // x: state (長さ12)
    // dxdt: derivative
    // t: 時刻
    void operator()(const state_type &x, state_type &dxdt, const double /* t */)
    {
        // 縦運動部分
        Eigen::Vector4d x_long;
        for (int i = 0; i < 4; i++)
        {
            // x[0]:u,前方速度の変動, x[1]:w下降速度の変動, x[2]:q,ピッチ角速度, x[3]:θ,ピッチ角
            x_long(i) = x[i];
        }
        Eigen::Vector2d u_long;
        u_long << u_long_[0], u_long_[1];
        Eigen::Vector4d dx_long = A_long_ * x_long + B_long_ * u_long;

        // 横運動部分
        Eigen::VectorXd x_lat(5);
        for (int i = 0; i < 5; i++)
        {
            // x[4]:β,よこすべり速度の変動, x[5]:p,ロール角速度, x[6]:r,ヨー角速度, x[7]:φ,ロール角, x[8]:ψ,ヨー角
            x_lat(i) = x[4 + i];
        }
        Eigen::Vector2d u_lat;
        u_lat << u_lat_[0], u_lat_[1];
        Eigen::VectorXd dx_lat = A_lat_ * x_lat + B_lat_ * u_lat;

        // 機体座標系での速度ベクトル
        double u_b = Param::U0 + x[0]; // x[0] は前方速度の変動
        double v_b = x[4];             // x[4] はよこすべり速度
        double w_b = x[1];             // x[1] は上下方向の速度変動
        Eigen::Vector3d vel_b(u_b, v_b, w_b);

        // 機体釣り合い座標系での速度ベクトルを全体座標系に変換
        double phi = x[7];   // 横のφ（ロール）
        double theta = x[3]; // 縦のθ（ピッチ）
        double psi = x[8];   // 横のψ（ヨー）
        Eigen::Vector3d vel_e = rotation_matrix(psi, theta, phi) * vel_b;

        // dxdt の組み立て
        for (int i = 0; i < 4; i++)
        {
            dxdt[i] = dx_long(i);
        }
        for (int i = 0; i < 5; i++)
        {
            dxdt[4 + i] = dx_lat(i);
        }
        for (int i = 0; i < 3; i++)
        {
            dxdt[9 + i] = vel_e(i);
        }
    }

private:
    // 行列メンバ
    Eigen::Matrix<double, 4, 4> A_long_;
    Eigen::Matrix<double, 4, 2> B_long_;
    Eigen::Matrix<double, 5, 5> A_lat_;
    Eigen::Matrix<double, 5, 2> B_lat_;

    // 制御入力（ロング：delta_e, delta_t; ラット：delta_a, delta_r）
    std::array<double, 2> u_long_{0.0, 0.0};
    std::array<double, 2> u_lat_{0.0, 0.0};
};

// ROS2ノード
class DynamicsSimulator : public rclcpp::Node
{
public:
    DynamicsSimulator()
        : Node("dynamics_simulator"), dynamics_()
    {
        // 初期状態 (全要素0とする)
        state_.fill(0.0);
        sbus_data_sub_ = this->create_subscription<ksenos_ground_msgs::msg::SbusData>(
            "/sbus_offset", 10,
            std::bind(&DynamicsSimulator::control_callback, this, std::placeholders::_1));

        // タイマコールバックでODE１ステップ実行（dt秒）
        timer_ = this->create_wall_timer(
            10ms, std::bind(&DynamicsSimulator::timer_callback, this));

        // tf2 ブロードキャスターの生成
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void control_callback(const ksenos_ground_msgs::msg::SbusData::SharedPtr msg)
    {
        // 制御入力の更新
        // twist.angular.y -> elevator (delta_e)
        // twist.linear.x  -> throttle (delta_t)
        // twist.linear.y  -> aileron (delta_a)
        // twist.angular.z -> rudder (delta_r)
        double delta_e = msg->elevator;  // elevator
        double delta_t = msg->throttle;  // throttle
        double delta_a = msg->aileron_l; // aileron
        double delta_r = msg->rudder;    // rudder

        dynamics_.set_sbus_datas(delta_e, delta_t, delta_a, delta_r);
    }

    void timer_callback()
    {
        // 時間刻み
        double dt = 0.01; // 10ms
        // Boost Odeint RK4 による１ステップ
        runge_kutta4<state_type> stepper;
        stepper.do_step(dynamics_, state_, t_, dt);
        t_ += dt;

        // NED系のEuler角 (roll: state_[7], pitch: state_[3], yaw: state_[8]) からクォータニオンを計算
        Eigen::Quaterniond q_ned = Eigen::AngleAxisd(state_[7], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(state_[3], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(state_[8], Eigen::Vector3d::UnitZ());

        // NED -> ROS2(ENU) 変換行列
        Eigen::Matrix3d T;
        T << 0, 1, 0,
            1, 0, 0,
            0, 0, -1;
        Eigen::Quaterniond q_conv(T);
        // ROS2座標系に変換されたクォータニオン
        Eigen::Quaterniond q_ros = q_conv * q_ned;

        // tf の TransformStamped の作成
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "aircraft_stability_axes";

        // 位置の変換: NED(x,y,z) -> ENU(x,y,z)
        transformStamped.transform.translation.x = state_[10];  // x_ros = y_ned
        transformStamped.transform.translation.y = state_[9];   // y_ros = x_ned
        transformStamped.transform.translation.z = -state_[11]; // z_ros = -z_ned

        transformStamped.transform.rotation.w = q_ros.w();
        transformStamped.transform.rotation.x = q_ros.x();
        transformStamped.transform.rotation.y = q_ros.y();
        transformStamped.transform.rotation.z = q_ros.z();
        // tf ブロードキャスターによる送信
        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<ksenos_ground_msgs::msg::SbusData>::SharedPtr sbus_data_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    AircraftDynamics dynamics_;
    state_type state_;
    double t_{0.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicsSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}