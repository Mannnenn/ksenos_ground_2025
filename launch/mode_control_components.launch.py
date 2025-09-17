from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
        default_value='',
        description='Namespace for all nodes'
    )

    # 各コンポーネントノードの有効化引数
    is_auto_landing_mode_arg = DeclareLaunchArgument(
        'is_auto_landing_mode',
        default_value='false',
        description='Enable auto landing mode (affects which nodes are launched)'
    )
    
    def launch_setup(context, *args, **kwargs):
        # 引数値を取得
        is_auto_landing_mode = LaunchConfiguration('is_auto_landing_mode').perform(context) == 'true'
        
        # 条件付きコンポーネントノードリストを構築
        nodes = []
        
        if (is_auto_landing_mode == False):
            # Auto Turning Mode ノード
            nodes.append(ComposableNode(
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
            ))
            
            # Eight Turning Mode ノード
            nodes.append(ComposableNode(
                package='ksenos_ground',
                plugin='ModeEightTurningAngle',
                namespace='mode_selector',
                name='eight_turning_node',
                parameters=[{
                    'max_turn_radius': 4.5,
                    'min_turn_radius': 4.0,
                    'turn_angle_deg': 320.0,
                }],
                remappings=[
                    ('sbus_data', '/sbus/manual/sbus_data'),
                    ('serialized_yaw', '/sensor/serialized_yaw_angle'),
                    ('turn_radius', '/controller/lat/calc/turning_radius'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ))
            
            # Rise Turning Mode ノード
            nodes.append(ComposableNode(
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
            ))
        
        if (is_auto_landing_mode == True):
            # L1 Control Node (auto landing mode)
            nodes.append(ComposableNode(
                package='ksenos_ground',
                plugin='L1ControlNode',
                namespace='controller/lat/calc',
                name='L1_control_node',
                parameters=[{
                    'path_topic': '/controller/lat/calc/path',
                    'velocity_topic': '/sensor/flow_rate',
                    'base_frame': 'ksenos_smooth_0',
                    'eta_topic': '/controller/lat/calc/turning_radius',
                    'marker_topic': '/visualization/L1_marker',
                    'lookahead_gain': 2.0,
                    'lookahead_min': 3.0,
                    'lookahead_max': 50.0,
                    'publish_lateral_acc': False,
                    'lateral_acc_topic': '/controller/lat/calc/lateral_acceleration',
                    'min_speed_for_heading': 0.5,
                    'target_altitude_topic': '/controller/long/calc/target_altitude',
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ))

            # Path Generator Node (auto landing mode)
            nodes.append(ComposableNode(
                package='ksenos_ground',
                plugin='PathGenerator',
                namespace='controller/lat/calc',
                name='path_generator_node',
                parameters=[{
                    'output_path_topic_name': '/controller/lat/calc/path',
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ))

        # コンポーネントコンテナーの設定
        container = ComposableNodeContainer(
            name="mode_control_container",
            namespace="",
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=nodes,
            output='both',
            arguments=['--ros-args', '--log-level', 'info'],
        )
        
        return [container]

    return LaunchDescription([
        container_name_arg,
        namespace_arg,
        is_auto_landing_mode_arg,
        OpaqueFunction(function=launch_setup),
    ])
