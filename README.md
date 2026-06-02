# ksenos_ground_2025

ROS 2ベースの飛行機自動操縦制御システムです。LiDARセンサを使用した地形認識と、複数の飛行モードをサポートしています。

## 概要

このプロジェクトは、無人飛行機の自動操縦を実現するための統合制御システムです。LiDAR（Hesai QT）を用いた点群処理、IMUデータの活用、そしてS.BUS信号の処理により、複数の自動飛行モードを提供します。

### 主な機能

- **複数の飛行モード**
  - 手動操縦: マニュアル操作
  - 自動旋回: 定常円旋回
  - 8の字: 8の字軌道で周回
  - 上昇旋回: 低空での定常円旋回後、徐々に高度を上昇
  - 自動離着陸: 目標軌道に沿った移動と自動着陸

- **点群処理パイプライン**
  - LiDAR点群の取得と変換
  - 地面推定と補正
  - 移動物体検出
  - 初期スキャン地図化

- **制御システム**
  - 横方向制御: エルロン制御、ラダー制御
  - 縦方向制御: スロットル制御、エレベーター制御
  - L1経路追従制御による自動離着陸

- **センサ統合**
  - LiDAR（Hesai QT）からの点群データ
  - IMUデータの処理と高度推定
  - 気圧センサ、ToFセンサによる高度計測
  - UDP通信による外部ポーズ推定

## 必要環境

- ROS 2 (Humble以上推奨)
- C++17対応のコンパイラ
- CMake 3.8以上

### 依存ライブラリ

主な依存ライブラリ:
- PCL (Point Cloud Library)
- Eigen3
- Boost (thread)
- tf2 (Transform Library)
- ament_cmake_auto

詳細は`package.xml`を参照してください。

## ビルド方法

```bash
# colcon workspaceルートで実行
colcon build --symlink-install

# ビルド後、セットアップスクリプトを読み込む
source install/setup.bash
```

## 主なノード構成

### 点群処理パイプライン
- `init_scans_component`: 初期スキャン地図化
- `pointcloud_transform_component`: 点群のモーター座標系への変換
- `detect_movable_objects_component`: 移動物体検出
- `correction_ground_component`: 地面推定と補正
- `pub_pointcloud_component`: LiDAR点群のパブリッシュ

### センサ処理
- `leveling_imu`: IMU水平補正
- `tf_smoother`: TF平滑化ノード
- `calc_altitude_imu_component`: IMUベースの高度計算
- `calc_altitude_lidar_component`: LiDARベースの高度計算
- `calc_altitude_pressure`: 気圧計による高度計算

### S.BUSデータ処理
- `sbus_format_data_to_radian_component`: Raw → Radian変換
- `sbus_format_data_to_raw_component`: Radian → Raw変換
- `sbus_calibration_component`: S.BUSキャリブレーション
- `sbus_get_raw_data_component`: S.BUS生データ読み出し
- `sbus_send_raw_data_component`: UDP経由のS.BUS送信

### 制御ノード

**横方向制御:**
- `turningRadius_to_latAcc_component`: 旋回半径から横加速度を計算
- `latAcc_to_targetRollAngle_component`: 横加速度から目標ロール角を計算
- `control_aileron_component`: エルロン制御（PD制御）
- `control_rudder_component`: ラダー制御（FF+PI制御）

**縦方向制御:**
- `control_throttle_component`: スロットル制御（PI制御）
- `control_elevator_component`: エレベーター制御
- `control_speed_component`: 速度ターゲット生成
- `mode_target_altitude_selector_component`: モード別目標高度セレクター

**自動離着陸:**
- `path_generator_component`: 経路生成
- `control_L1_law_component`: L1経路追従制御
- `calc_flight_distance_component`: 飛行距離計測
- `control_drop_component`: 投下制御

### モード制御ノード
- `mode_auto_turning`: 自動旋回モード
- `mode_eight_turning_angle`: 8の字旋回モード
- `mode_rise_turning`: 上昇旋回モード
- `mode_auto_landing_backup`: 自動離着陸バックアップモード

### その他
- `unity_control_input_component`: 制御入力統合
- `calc_rpy_from_quat_component`: クォータニオンからRPY計算
- `yaw_angle_serialization_component`: 回転回数カウンター
- `publish_joint_airplane`: 動翼角度パブリッシャー
- `project_orientation_component`: 角度投影パブリッシャー

## 飛行モードの詳細

### 手動操縦
何もしないモード。外部からのS.BUS信号を直接処理します。

### 自動旋回
定常円旋回を実現するモード。
- 一定旋回半径 → 一定横加速度 → 一定ロール角で制御
- 目標速度・高度は一定に保持

### 8の字
8の字軌道を周回するモード。
- 8の字軌道上の曲率に応じて旋回半径を動的に調整
- グラフ中心で θ=270°となる角度基準の制御

### 上昇旋回
低空から高度を段階的に上昇させるモード。
- 低空で720°旋回（2周）
- その後、指定度数（N°）かけて徐々に高度を上昇
- 上昇完了後は高度一定で旋回継続

### 自動離着陸
目標経路に沿った移動と自動着陸を実現するモード。
- L1制御をベースに必要な横加速度を算出
- ロール角は可変（横加速度に応じて動的に調整）
- 経路からの最近傍点探索と L1 距離を用いた制御則

## ディレクトリ構成

```
.
├── CMakeLists.txt           # ビルド設定
├── package.xml              # ROS 2パッケージ定義
├── README.md                # このファイル
├── CONTRIBUTING.md          # 貢献ガイドライン
├── LICENSE                  # MITライセンス
├── include/                 # ヘッダーファイル
│   └── pcd_proc_cpp/        # LiDAR SDK含む
├── src/                     # ソースコード
│   ├── pointcloud_pipeline/ # 点群処理
│   ├── sbus_data_processor/ # S.BUSデータ処理
│   ├── pose_estimation/     # ポーズ推定
│   ├── lat_control/         # 横方向制御
│   ├── long_control/        # 縦方向制御
│   ├── auto_landing/        # 自動離着陸制御
│   └── mode_control/        # モード制御
├── config/                  # 設定ファイル
├── launch/                  # Launch ファイル
├── urdf/                    # URDFモデル
└── meshes/                  # メッシュファイル
```

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。詳細は[LICENSE](LICENSE)ファイルを参照してください。

## 貢献

このプロジェクトへの貢献を歓迎します。[CONTRIBUTING.md](CONTRIBUTING.md)を参照して、貢献ガイドラインに従ってください。

## 関連情報

- **言語構成**: C++ (84.9%), Python (11.3%), CMake (3.8%)
- **保守者**: [@Mannnenn](https://github.com/Mannnenn)
- **メールアドレス**: 86047097+Mannnenn@users.noreply.github.com

## 使用ハードウェア

- **LiDAR**: Hesai Qt（Hesai General SDK使用）
- **通信**: S.BUS（フライトコントローラーとの通信）、UDP（外部センサ/制御との通信）
