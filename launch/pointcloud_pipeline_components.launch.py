#!/usr/bin/env python3
# Copyright 2025 ksenos_ground
#
# Licensed under the MIT License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://opensource.org/licenses/MIT
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ポイントクラウド処理パイプライン用コンポーネント起動設定

5つのノードを同一プロセス内でコンポーネントとして実行する
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Launch引数の宣言
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='pointcloud_pipeline_container',
        description='Name of the component container'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    # コンポーネントコンテナーの設定
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # LiDAR点群パブリッシャー
            ComposableNode(
                package='ksenos_ground',
                plugin='PointCloudPublisher',
                name='pointcloud_publisher',
                parameters=[{
                    # LiDAR設定パラメータ（必要に応じて追加）
                }],
                remappings=[
                    ('lidar_points', '/lidar_points'),
                ]
            ),
            
            # 点群座標変換
            ComposableNode(
                package='ksenos_ground',
                plugin='PointCloudTransformer',
                name='pointcloud_transform_node',
                parameters=[{
                    'input_topic': '/lidar_points',
                    'output_topic': '/transformed_points',
                    'target_frame': 'motor_base',
                    'source_frame': 'hesai_lidar',
                    'timeout_seconds': 0.01,
                    'queue_size': 10,
                }],
                remappings=[
                    ('lidar_points', '/lidar_points'),
                    ('transformed_points', '/transformed_points'),
                ]
            ),
            
            # 地面補正ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='GroundCorrectionNode',
                name='ground_correction_node',
                parameters=[{
                    'input_topic': '/transformed_points',
                    'base_frame': 'map',
                    'lidar_frame': 'motor_base',
                    'voxel_size': 0.05,
                    'plane_distance_threshold': 0.02,
                    'max_iterations': 100,
                    'min_inliers': 1000,
                    'statistical_filter_mean_k': 50,
                    'statistical_filter_stddev': 1.0,
                }],
                remappings=[
                    ('input_pointcloud', '/transformed_points'),
                ]
            ),
            
            # 初期スキャン・地図作成
            ComposableNode(
                package='ksenos_ground',
                plugin='LidarScanNode',
                name='lidar_scan_node',
                parameters=[{
                    'scan_duration': 10.0,
                    'lower_limit_rad': -1.5708,  # -π/2
                    'upper_limit_rad': 1.5708,   # π/2
                    'init_angle_rad': 0.0,
                    'voxel_leaf_size': 0.1,
                    'timer_period_ms': 100,
                    'angle_topic': '/target_pitch_angle',
                    'input_pointcloud_topic': '/transformed_points',
                    'output_pointcloud_topic': '/map_points',
                }],
                remappings=[
                    ('target_pitch_angle', '/target_pitch_angle'),
                    ('transformed_points', '/transformed_points'),
                    ('map_points', '/map_points'),
                ]
            ),
            
            # 移動物体検出
            ComposableNode(
                package='ksenos_ground',
                plugin='MovableObjectDetector',
                name='movable_object_detector',
                parameters=[{
                    'map_topic': '/map_points',
                    'scan_topic': '/transformed_points',
                    'output_topic': '/detected_objects',
                    'elevation_topic': '/target_pitch_angle',
                    'bbox_marker_topic': '/bbox_marker',
                    'lidar_frame': 'lidar_center',
                    'base_frame': 'motor_base',
                    'map_frame': 'map',
                    'object_frame_prefix': 'movable_object_',
                    'octree_resolution': 1.5,
                    'voxel_leaf_size': 0.025,
                    'bbox_min_x': -10.0,
                    'bbox_min_y': -10.0,
                    'bbox_min_z': -2.0,
                    'bbox_max_x': 10.0,
                    'bbox_max_y': 10.0,
                    'bbox_max_z': 10.0,
                    'cluster_tolerance': 0.25,
                    'min_cluster_size': 10,
                    'max_cluster_size': 250,
                    'object_timeout': 5.0,
                    'max_association_distance': 5.0,
                    'min_tf_cluster_size': 10,
                    'elevation_offset': -0.0523,
                }],
                remappings=[
                    ('map_points', '/map_points'),
                    ('transformed_points', '/transformed_points'),
                    ('detected_objects', '/detected_objects'),
                    ('target_pitch_angle', '/target_pitch_angle'),
                    ('bbox_marker', '/bbox_marker'),
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        container_name_arg,
        namespace_arg,
        container
    ])
