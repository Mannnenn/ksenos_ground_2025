from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch引数の宣言
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='long_control_container',
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
            # 平均高度・速度計算ノード
            nodes.append(ComposableNode(
                package='ksenos_ground',
                plugin='AverageAltitudeVelocityNode',
                namespace='controller/long/calc',
                name='calc_average_altitude_velocity_node',
                parameters=[],
                remappings=[
                    ('/average/altitude_imu', '/controller/long/calc/average_altitude_imu'),
                    ('/average/flow_rate', '/controller/long/calc/average_flow_rate'),
                    ('/sensor/altitude_imu', '/sensor/altitude/altitude_imu'),
                    ('/sensor/altitude_lidar', '/sensor/altitude/altitude_lidar'),
                    ('/sbus_data', '/sbus/manual/sbus_data'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ))
        
            # 目標高度セレクターノード
            nodes.append(ComposableNode(
                package='ksenos_ground',
                plugin='ModeTargetAltitudeSelector',
                namespace='controller/long/calc',
                name='mode_target_altitude_selector_node',
                parameters=[],
                remappings=[
                    ('sbus_data', '/sbus/manual/sbus_data'),
                    ('average_altitude', '/controller/long/calc/average_altitude_imu'),
                    ('altitude_dynamic', '/controller/long/calc/dynamic_altitude'),
                    ('altitude_target', '/controller/long/calc/target_altitude'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ))
        if (is_auto_landing_mode == True):
            # 速度生成ノード
            nodes.append(ComposableNode(
                package='ksenos_ground',
                plugin='ControlSpeedNode',
                namespace='controller/long/calc',
                name='control_speed_node',
                parameters=[{
                    'world_frame_id': 'start_point',
                    'base_link_frame_id': 'ksenos_smooth_0',
                    'target_speed_topic': '/controller/long/calc/average_flow_rate',
                    'marker_topic': '/visualization/target_velocity_marker',
                    'x_sustain_end': 14.0,
                    'x_stop': 31.0,
                    'v_max': 5.0,
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ))


        # エネルギー計算ノード
        # 基準エネルギー計算ノード
        nodes.append(ComposableNode(
            package='ksenos_ground',
            plugin='CalcEnergyNode',
            namespace='controller/long/calc',
            name='calc_energy_reference',
            parameters=[{
                'mass': 0.23,
                'gravity': 9.81,
                'velocity_topic': '/controller/long/calc/average_flow_rate',
                'altitude_topic': '/controller/long/calc/target_altitude',
                'energy_topic': '/controller/long/reference_energy',
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        ))
            
        # 現在エネルギー計算ノード
        nodes.append(ComposableNode(
            package='ksenos_ground',
            plugin='CalcEnergyNode',
            namespace='controller/long/calc',
            name='calc_energy_current',
            parameters=[{
                'mass': 0.23,
                'gravity': 9.81,
                'velocity_topic': '/sensor/flow_rate',
                'altitude_topic': '/sensor/altitude/altitude_imu',
                'energy_topic': '/controller/long/current_energy',
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        ))
        
        # スロットル制御ノード
        nodes.append(ComposableNode(
            package='ksenos_ground',
            plugin='ThrottleControl',
            namespace='controller/long/control',
            name='throttle_control_node',
            parameters=[{
                'kp': 0.13,
                # 'kp': 0.0075,
                'ki': 0.0,
                # 'max_throttle': 0.85,
                'max_throttle': 0.7,
                'min_throttle': 0.0,
                # 'steady_throttle': 0.46,
                'steady_throttle': 0.4,
                'max_integral': 0.5,
            }],
            remappings=[
                ('/throttle_input', '/controller/long/throttle_input'),
                ('/airplane/reference_energy', '/controller/long/reference_energy'),
                ('/airplane/current_energy', '/controller/long/current_energy'),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ))
        
        # エレベーター制御ノード
        nodes.append(ComposableNode(
            package='ksenos_ground',
            plugin='ElevatorControl',
            namespace='controller/long/control',
            name='elevator_control_node',
            parameters=[{
                'k_energy_gain': 0.15,
                # 'k_energy_gain': 0.065,
                'kd_pitch_angle': -1.0,
                # 'kd_pitch_angle': -0.5,
                'kd_pitch_rate': 0.55,
                'max_elevator': 0.3,
                'min_elevator': -0.3,
                'balanced_flight_pitch': 0.25,
            }],
            remappings=[
                ('/elevator_input', '/controller/long/elevator_input'),
                ('/airplane/reference_energy', '/controller/long/reference_energy'),
                ('/airplane/current_energy', '/controller/long/current_energy'),
                ('/rpy', '/sensor/orientation/euler_angles'),
                ('/target_pitch', '/controller/lat/calc/target_pitch'),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ))
        
        # 制御入力統合ノード
        nodes.append(ComposableNode(
            package='ksenos_ground',
            plugin='UnityControlInputNode',
            namespace='controller',  # controllerルートレベル
            name='unity_control_input_node',
            parameters=[],
            remappings=[
                ('/throttle_input', '/controller/long/throttle_input'),
                ('/elevator_input', '/controller/long/elevator_input'),
                ('/aileron_input', '/controller/lat/aileron_input'),
                ('/rudder_input', '/controller/lat/rudder_input'),
                ('/drop_signal','/controller/drop_signal'),
                ('/control_input', '/controller/control_input'),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ))
        
        # コンポーネントコンテナーの設定
        container = ComposableNodeContainer(
            name="long_control_container",
            namespace="/controller/long",
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
