from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    # コンポーネントコンテナーの設定
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 平均高度・速度計算ノード
            ComposableNode(
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
                ]
            ),
            
            # 目標高度セレクターノード
            ComposableNode(
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
                ]
            ),
            
            # 基準エネルギー計算ノード
            ComposableNode(
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
            ),
            
            # 現在エネルギー計算ノード
            ComposableNode(
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
            ),
            
            # スロットル制御ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='ThrottleControl',
                namespace='controller/long/control',
                name='throttle_control_node',
                parameters=[{
                    'kp': 0.005,
                    'ki': 0.0,
                    'max_throttle': 0.7,
                    'min_throttle': 0.0,
                    'steady_throttle': 0.45,
                    'max_integral': 0.5,
                }],
                remappings=[
                    ('/throttle_input', '/controller/long/throttle_input'),
                    ('/airplane/reference_energy', '/controller/long/reference_energy'),
                    ('/airplane/current_energy', '/controller/long/current_energy'),
                ]
            ),
            
            # エレベーター制御ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='ElevatorControl',
                namespace='controller/long/control',
                name='elevator_control_node',
                parameters=[{
                    'k_energy_gain': 0.065,
                    'kd_pitch_angle': -0.6,
                    'kd_pitch_rate': 0.5,
                    'max_elevator': 0.3,
                    'min_elevator': -0.3,
                    'balanced_flight_pitch': 0.0,
                }],
                remappings=[
                    ('/elevator_input', '/controller/long/elevator_input'),
                    ('/airplane/reference_energy', '/controller/long/reference_energy'),
                    ('/airplane/current_energy', '/controller/long/current_energy'),
                    ('/rpy', '/sensor/orientation/euler_angles'),
                ]
            ),
            
            # 制御入力統合ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='UnityControlInputNode',
                namespace='',  # controllerルートレベル
                name='unity_control_input_node',
                parameters=[],
                remappings=[
                    ('/throttle_input', '/controller/long/throttle_input'),
                    ('/elevator_input', '/controller/long/elevator_input'),
                    ('/aileron_input', '/controller/lat/aileron_input'),
                    ('/rudder_input', '/controller/lat/rudder_input'),
                ]
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
