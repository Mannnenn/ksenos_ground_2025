# ksenos_ground

## 目的

このパッケージは、体育館のような環境で LiDAR 点群データから水平面（床面）を推定し、LiDAR の傾きと高さを補正するための TF（座標変換）をパブリッシュします。

## 機能

### correction_ground

LiDAR 点群から床面を推定し、以下の情報を含む TF をパブリッシュします：

- **ロール（Roll）**: Y 軸周りの回転（横方向の傾き）
- **ピッチ（Pitch）**: X 軸周りの回転（縦方向の傾き）
- **高さ（Z 軸移動）**: 床面から LiDAR までの高さ

#### アルゴリズム

1. **前処理**

   - ボクセルフィルタによるダウンサンプリング
   - 統計的外れ値除去

2. **平面推定**

   - RANSAC（Random Sample Consensus）を使用
   - 床面として最も適切な平面を検出

3. **TF 計算**
   - 推定された平面の法線ベクトルからロール・ピッチを計算
   - 平面までの距離から高さを算出

## 使用方法

### ビルド

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select ksenos_ground
source install/setup.bash
```

### 実行

#### 基本実行

```bash
ros2 launch ksenos_ground ground_correction.launch.py
```

#### パラメータ指定実行

```bash
ros2 launch ksenos_ground ground_correction.launch.py \
    input_topic:=/velodyne_points \
    base_frame:=map \
    lidar_frame:=velodyne
```

#### ノード単体実行

```bash
ros2 run ksenos_ground correction_ground --ros-args \
    --params-file config/ground_correction_params.yaml \
    -p input_topic:=/input_pointcloud
```

## パラメータ

### 設定可能パラメータ（config/ground_correction_params.yaml）

| パラメータ                  | デフォルト値        | 説明                               |
| --------------------------- | ------------------- | ---------------------------------- |
| `input_topic`               | `/input_pointcloud` | 入力点群トピック                   |
| `base_frame`                | `base_link`         | ベースフレーム（地面基準）         |
| `lidar_frame`               | `lidar`             | LiDAR フレーム                     |
| `voxel_size`                | `0.05`              | ボクセルフィルタのサイズ（m）      |
| `plane_distance_threshold`  | `0.02`              | RANSAC 距離閾値（m）               |
| `max_iterations`            | `1000`              | RANSAC 最大反復回数                |
| `min_inliers`               | `1000`              | 平面として認識する最小インライア数 |
| `publish_rate`              | `10.0`              | TF 配信レート（Hz）                |
| `statistical_filter_mean_k` | `50`                | 統計的外れ値除去の k 値            |
| `statistical_filter_stddev` | `1.0`               | 統計的外れ値除去の標準偏差倍数     |

## 入出力

### 入力

- **点群データ**: `sensor_msgs/msg/PointCloud2`
  - LiDAR から取得した生の点群データ

### 出力

- **TF 変換**: `geometry_msgs/msg/TransformStamped`
  - `base_frame` から `lidar_frame` への座標変換
  - ロール、ピッチ、高さの補正情報を含む

## システム要件

- ROS2 Humble 以降
- PCL（Point Cloud Library）
- Eigen3
- C++17 対応コンパイラ

## 依存パッケージ

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- `tf2_eigen`
- `pcl_conversions`
- `pcl_ros`

## トラブルシューティング

### 床面が検出されない場合

1. `min_inliers` パラメータを下げる
2. `plane_distance_threshold` パラメータを調整する
3. 点群データに十分な床面ポイントが含まれているか確認する

### TF が不安定な場合

1. `voxel_size` を調整してダウンサンプリング度合いを変更
2. `statistical_filter_*` パラメータで外れ値除去を強化
3. `publish_rate` を下げて TF 更新頻度を調整

## tf_smoother

TF（座標変換）を平滑化するノードです。急激な変化を除去し、なめらかな TF をパブリッシュします。

### 機能

- **TF サブスクリプション**: 指定されたフレーム間の TF を監視
- **フィルタリング**: 設定された閾値を超える急激な変化を無視
- **平滑化**: 線形補間と SLERP（球面線形補間）による平滑化
- **タイムアウト処理**: 長時間更新がない場合の再初期化

### パラメータ

| パラメータ名                | デフォルト値       | 説明                    |
| --------------------------- | ------------------ | ----------------------- |
| `source_frame`              | "map"              | 基準フレーム            |
| `target_frame`              | "base_link"        | ターゲットフレーム      |
| `smoothed_target_frame`     | "base_link_smooth" | 平滑化後のフレーム名    |
| `max_translation_threshold` | 0.5                | 並進移動の最大閾値 [m]  |
| `max_rotation_threshold`    | 0.5                | 回転の最大閾値 [rad]    |
| `timeout_duration`          | 1.0                | タイムアウト時間 [s]    |
| `publish_rate`              | 50.0               | パブリッシュレート [Hz] |
| `smoothing_factor`          | 0.1                | 平滑化係数 (0.0-1.0)    |

### 使用方法

#### 1. パラメータ指定での実行

```bash
ros2 launch ksenos_ground tf_smoother.launch.py source_frame:=map target_frame:=base_link
```

#### 2. 設定ファイルを使用した実行

```bash
ros2 launch ksenos_ground tf_smoother_with_config.launch.py
```

#### 3. 直接実行

```bash
ros2 run ksenos_ground tf_smoother
```

### アルゴリズム

1. **TF 監視**: 指定されたフレーム間の TF を定期的に取得
2. **変化量計算**: 前回の TF と比較して並進・回転の変化量を計算
3. **閾値判定**: 設定された閾値を超える場合は変化を無視
4. **平滑化適用**: 線形補間（位置）と SLERP（回転）で平滑化
5. **TF パブリッシュ**: 平滑化された TF をブロードキャスト
6. **タイムアウト処理**: 長時間更新がない場合は再初期化

## ライセンス

MIT License
