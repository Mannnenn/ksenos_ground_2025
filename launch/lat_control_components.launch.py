from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch引数の宣言
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='lat_control_container',
        description='Name of the component container'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    # コンポーネントコンテナーの設定
    container = ComposableNodeContainer(
        name="lat_control_container",
        namespace="controller/lat",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 旋回半径から横加速度を計算するノード
            ComposableNode(
                package='ksenos_ground',
                plugin='LateralAccelerationCalculator',
                namespace='controller/lat/calc',
                name='turning_radius_to_lat_acc',
                parameters=[{
                    'low_pass_filter_alpha': 0.1,
                }],
                remappings=[
                    ('/controller/lat/calc/lateral_acceleration', '/controller/lat/lateral_acceleration'),
                    ('/average/flow_rate', '/sensor/flow_rate'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # 横加速度から目標ロール角を計算するノード
            ComposableNode(
                package='ksenos_ground',
                plugin='LatAccToTargetRollAngle',
                namespace='controller/lat/calc',
                name='lat_acc_to_target_roll_angle',
                parameters=[{
                    'max_roll_angle': 0.6,
                }],
                remappings=[
                    ('/controller/lat/calc/lateral_acceleration', '/controller/lat/lateral_acceleration'),
                    ('/controller/lat/calc/target_roll_angle', '/controller/lat/target_roll_angle'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # エルロン制御ノード（PD制御）
            ComposableNode(
                package='ksenos_ground',
                plugin='AileronControl',
                namespace='controller/lat/control',
                name='aileron_control',
                parameters=[{
                    'kp': -0.55,
                    'kd': 0.025,
                    'max_aileron': 0.6,
                    'min_aileron': -0.6,
                    'kp_right_scale': 1.3,
                    'kp_left_scale': 1.0,
                }],
                remappings=[
                    ('/controller/lat/calc/target_roll_angle', '/controller/lat/target_roll_angle'),
                    ('/rpy', '/sensor/orientation/euler_angles'),
                    ('/aileron_input', '/controller/lat/aileron_input'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # ラダー制御ノード（FF+PI制御）
            ComposableNode(
                package='ksenos_ground',
                plugin='RudderControl',
                namespace='controller/lat/control',
                name='rudder_control',
                parameters=[{
                    'kp': 0.00,
                    'ki': 0.00,
                    'ff_lat_acc_gain': 0.045,
                    'ff_lat_acc_right_scale': 1.85,
                    'ff_lat_acc_left_scale': 1.0,
                    'ff_aileron_gain': 0.01,
                    'max_rudder': 0.6,
                    'min_rudder': -0.6,
                    'max_integral': 0.1,
                }],
                remappings=[
                    ('/aileron_input', '/controller/lat/aileron_input'),
                    ('/controller/lat/calc/lateral_acceleration', '/controller/lat/lateral_acceleration'),
                    ('/rudder_input', '/controller/lat/rudder_input'),
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
