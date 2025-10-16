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
        name="pointcloud_pipeline_container",
        namespace="pointcloud_pipeline",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # LiDAR点群パブリッシャー
            ComposableNode(
                package='ksenos_ground',
                plugin='PointCloudPublisher',
                namespace='pointcloud_pipeline',
                name='pointcloud_publisher',
                parameters=[{
                    # LiDAR設定パラメータ（必要に応じて追加）
                }],
                remappings=[
                    ('lidar_points', '/pointcloud_pipeline/lidar_points'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # 点群座標変換
            ComposableNode(
                package='ksenos_ground',
                plugin='PointCloudTransformer',
                namespace='pointcloud_pipeline',
                name='pointcloud_transform_node',
                parameters=[{
                    'input_topic': '/pointcloud_pipeline/lidar_points',
                    'output_topic': '/pointcloud_pipeline/transformed_points',
                    'target_frame': 'motor_base',
                    'source_frame': 'hesai_lidar',
                    'timeout_seconds': 0.01,
                    'queue_size': 10,
                }],
                remappings=[
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            

            
            # 初期スキャン・地図作成
            ComposableNode(
                package='ksenos_ground',
                plugin='LidarScanNode',
                namespace='pointcloud_pipeline',
                name='init_scan_node',
                parameters=[{
                    'scan_duration': 10.0,
                    'lower_limit_rad': -0.5,  
                    'upper_limit_rad': 1.25,   
                    'init_angle_rad': 0.0,
                    'voxel_leaf_size': 0.1,
                    'timer_period_ms': 100,
                    'angle_topic': '/motor/target_pitch_angle',
                    'input_pointcloud_topic': '/pointcloud_pipeline/transformed_points',
                    'output_pointcloud_topic': '/pointcloud_pipeline/map_points',
                }],
                remappings=[
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # 移動物体検出
            ComposableNode(
                package='ksenos_ground',
                plugin='MovableObjectDetector',
                namespace='pointcloud_pipeline',
                name='movable_object_detector',
                parameters=[{
                    'map_topic': '/pointcloud_pipeline/map_points',
                    'scan_topic': '/pointcloud_pipeline/transformed_points',
                    'output_topic': '/pointcloud_pipeline/detected_objects',
                    'elevation_topic': '/motor/target_pitch_angle',
                    'bbox_marker_topic': '/bbox_marker',
                    'lidar_frame': 'lidar_center',
                    'base_frame': 'motor_base',
                    'map_frame': 'map',
                    'object_frame_prefix': 'movable_object_',
                    'octree_resolution': 0.15,
                    'voxel_leaf_size': 0.015,
                    'bbox_min_x': 0.5,
                    'bbox_min_y': -8.0,
                    'bbox_min_z': 0.6,
                    'bbox_max_x': 30.0,
                    'bbox_max_y': 12.0,
                    'bbox_max_z': 9.0,
                    'min_z_x_threshold' : 32.0,
                    'min_z_lowering' : 1.5,
                    'cluster_tolerance': 1.2,
                    'min_cluster_size': 10,
                    'max_cluster_size': 500,
                    'object_timeout': 10.0,
                    'max_association_distance': 2.0,
                    'min_tf_cluster_size': 10,
                    'elevation_offset': -0.0523,
                    'elevation_min': -0.5,
                    'elevation_max': 1.5,
                }],
                remappings=[
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        container_name_arg,
        namespace_arg,
        container
    ])
