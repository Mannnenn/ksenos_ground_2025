from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch引数の宣言
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='mode_control_container',
        description='Name of the component container'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='controller',
        description='Namespace for all nodes'
    )

    # コンポーネントコンテナーの設定
    container = ComposableNodeContainer(
        name="mode_control_container",
        namespace="",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Auto Turning Mode ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='ModeAutoTurning',
                namespace='mode_selector',
                name='auto_turning_node',
                parameters=[{
                    'target_radius': 5.0,
                }],
                remappings=[
                    ('sbus_data', '/sbus/manual/sbus_data'),
                    ('target_turning_radius', '/controller/lat/calc/turning_radius'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # Eight Turning Mode ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='ModeEightTurningAngle',
                namespace='mode_selector',
                name='eight_turning_node',
                parameters=[{
                    'lobe_span_deg': 270.0,
                    'max_turn_radius': 5.0,
                    'min_turn_radius': 2.5,
                }],
                remappings=[
                    ('sbus_data', '/sbus/manual/sbus_data'),
                    ('serialized_yaw', '/sensor/serialized_yaw_angle'),
                    ('turn_radius', '/controller/lat/calc/turning_radius'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # Rise Turning Mode ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='ModeRiseTurning',
                namespace='mode_selector',
                name='rise_turning_node',
                parameters=[{
                    'altitude_offset': 3.5,
                    'lap_count_initial': 3,
                    'lap_count_transition': 2,
                    'target_radius': -5.0,
                }],
                remappings=[
                    ('sbus_data', '/sbus/manual/sbus_data'),
                    ('serialized_yaw', '/sensor/serialized_yaw_angle'),
                    ('average_altitude', '/controller/long/calc/average_altitude_imu'),
                    ('target_altitude_rise_turning', '/controller/long/calc/dynamic_altitude'),
                    ('target_turning_radius', '/controller/lat/calc/turning_radius'),
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
