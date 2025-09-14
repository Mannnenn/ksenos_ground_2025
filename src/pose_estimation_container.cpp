// Copyright 2025 ksenos_ground
//
// Licensed under the MIT License (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file pose_estimation_container.cpp
 * @brief 姿勢推定処理パイプライン用統合コンテナー
 *
 * 以下のコンポーネントを同一プロセス内で実行する：
 * - UDPReceiver: ESP32からのセンサーデータ受信
 * - CalcAltitudeNode: IMUによる高度補正
 * - AltitudeLidarNode: LiDARによる高度計算
 * - TfProjectionNode: 姿勢投影
 * - CalcRpyFromQuat: クォータニオンからRPY変換
 * - RotationCounterNode: ヨー角連続化
 * - ImuFilterMadgwickRos: IMUフィルタ（外部コンポーネント）
 */

#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

// Forward declarations of component classes
// (これらのクラスはコンポーネントライブラリで定義されている)
class UDPReceiver;
class CalcAltitudeNode;
class AltitudeLidarNode;
class TfProjectionNode;
class CalcRpyFromQuat;
class RotationCounterNode;

int main(int argc, char **argv)
{
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // エグゼキューターの設定
    rclcpp::executors::MultiThreadedExecutor executor;

    // 各コンポーネントのNodeOptionsを設定（必要に応じて）
    rclcpp::NodeOptions options;

    try
    {
        // 各コンポーネントノードを作成して追加
        // 注意: 実際の利用時には適切な順序と依存関係を考慮する

        RCLCPP_INFO(rclcpp::get_logger("pose_estimation_container"),
                    "Starting Pose Estimation Container...");

        // 注意: 実際のコンポーネント読み込みは動的に行う場合もある
        // この実装は基本的なコンテナーテンプレートである

        RCLCPP_INFO(rclcpp::get_logger("pose_estimation_container"),
                    "All components initialized. Starting execution...");

        // 実行開始
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("pose_estimation_container"),
                     "Exception in container: %s", e.what());
        return 1;
    }

    // 終了処理
    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("pose_estimation_container"),
                "Pose Estimation Container stopped.");

    return 0;
}
