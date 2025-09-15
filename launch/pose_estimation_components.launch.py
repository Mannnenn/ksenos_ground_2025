from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch引数の宣言
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='pose_estimation_container',
        description='Name of the component container'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    # コンポーネントコンテナーの設定
    container = ComposableNodeContainer(
        name="pose_estimation_container",
        namespace="",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # UDP受信ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='UDPReceiver',
                namespace='sensor',
                name='udp_sensor_receiver',
                parameters=[{
                    'local_ip': '10.42.0.1',
                    'local_port': 8888,
                    'publish_rate': 100.0,
                }],
                remappings=[
                    ('imu', '/sensor/imu'),
                    ('tof', '/sensor/tof'),
                    ('pressure', '/sensor/pressure'),
                    ('flow_rate', '/sensor/flow_rate'),
                    ('servo_enable', '/sensor/servo_enable'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # IMU高度計算ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='CalcAltitudeNode',
                namespace='sensor/altitude',
                name='calc_altitude_imu',
                parameters=[],
                remappings=[
                    ('/sensor/tof', '/sensor/tof'),
                    ('/imu/data', '/sensor/orientation/imu/data'),
                    ('/altitude_imu', '/sensor/altitude/altitude_imu'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # LiDAR高度計算ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='AltitudeLidarNode',
                namespace='sensor/altitude',
                name='calc_altitude_lidar',
                parameters=[],
                remappings=[
                    ('/altitude_lidar', '/sensor/altitude/altitude_lidar'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # # 姿勢投影ノード
            # ComposableNode(
            #     package='ksenos_ground',
            #     plugin='TfProjectionNode',
            #     namespace='sensor/orientation',
            #     name='tf_projection_node',
            # ),
            
            # RPY計算ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='CalcRpyFromQuat',
                namespace='sensor/orientation',
                name='calc_rpy_from_quat',
                parameters=[{
                    'input_topic': '/sensor/orientation/imu/data',
                    'output_topic': '/sensor/orientation/euler_angles',
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # ヨー角連続化ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='RotationCounterNode',
                namespace='sensor/orientation',
                name='yaw_angle_serialization_node',
                parameters=[{
                    'input_angular_topic_name': '/sensor/orientation/euler_angles',
                    'input_counter_reset_topic_name': '/sensor/counter/reset',
                    'output_serialized_yaw_angle_topic_name': '/sensor/serialized_yaw_angle',
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # IMUフィルタ（Madgwick）- 外部コンポーネント
            ComposableNode(
                package='imu_filter_madgwick',
                plugin='ImuFilterMadgwickRos',
                namespace='sensor/orientation',
                name='imu_filter_madgwick_node',
                parameters=[{
                    'use_mag': False,
                    'publish_tf': True,
                    'world_frame': 'enu',
                    'fixed_frame': 'odom',
                    'mag_bias_x': 0.0,
                    'mag_bias_y': 0.0,
                    'mag_bias_z': 0.0,
                    'orientation_stddev': 0.0,
                    'gain': 0.1,
                    'zeta': 0.0,
                }],
                remappings=[
                    ('imu/data_raw', '/sensor/imu'),
                    ('imu/data', '/sensor/orientation/imu/data'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        container_name_arg,
        namespace_arg,
        container,
    ])
