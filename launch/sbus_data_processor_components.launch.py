from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Robot description setup
    packages_name = "ksenos_ground"
    xacro_file_name = "ksenos_model.urdf"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Launch引数の宣言
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='sbus_data_processor_container',
        description='Name of the component container'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    # コンポーネントコンテナーの設定
    container = ComposableNodeContainer(
        name="sbus_data_processor_container",
        namespace="sbus",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Robot State Publisher
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                parameters=[robot_description],
                namespace='sbus',
                name="robot_state_publisher",
            ),
            
            # Manual SBUS処理グループ
            # SBUS生データ取得ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='SbusSerialReader',
                namespace='sbus/manual',
                name='sbus_get_raw_data',
                parameters=[],
                remappings=[
                    ('sbus_raw_data', 'sbus_raw'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # SBUSデータをラジアン形式に変換
            ComposableNode(
                package='ksenos_ground',
                plugin='SbusDataProcessor',
                namespace='sbus/manual',
                name='sbus_format_data_to_radian',
                parameters=[{
                    'servo_min_count': 352,
                    'servo_max_count': 1696,
                    'servo_min_rad': -0.5236,  # -30 degrees in radians
                    'servo_max_rad': 0.5236,   # 30 degrees in radians
                    'throttle_min_count': 352,
                    'throttle_max_count': 1696,
                    'throttle_min_openness': 0.0,
                    'throttle_max_openness': 1.0,
                    'autopilot_threshold': 1000,
                    'autolanding_threshold': 1000,
                    'dropping_device_threshold_low': 900,
                    'dropping_device_threshold_high': 1500,
                    'autopilot_mode_threshold_low': 500,
                    'autopilot_mode_threshold_high': 1500,
                }],
                remappings=[
                    ('sbus_raw_data', 'sbus_raw'),
                    ('sbus_data', 'sbus_radian_format'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # SBUSキャリブレーションノード
            ComposableNode(
                package='ksenos_ground',
                plugin='SbusCalibration',
                namespace='sbus/manual',
                name='sbus_calibration',
                parameters=[{
                    'calibration_duration': 3.0,
                    'input_topic': 'sbus_radian_format',
                    'offset_topic': '/sbus/sbus_offset_amount',
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # SBUSオフセット適用ノード（コントロール用）
            ComposableNode(
                package='ksenos_ground',
                plugin='SbusOffsetApply',
                namespace='sbus/manual',
                name='sbus_offset_apply_node_to_control',
                parameters=[{
                    'input_topic': 'sbus_radian_format',
                    'offset_topic': '/sbus/sbus_offset_amount',
                    'output_topic': 'sbus_data',
                    'offset_operation': 'subtract',  # オフセットを引く→サーボ中立位置に補正
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # ジョイントパブリッシャーノード
            ComposableNode(
                package='ksenos_ground',
                plugin='ControlToJointPublisher',
                namespace='sbus',
                name='control_to_joint_publisher',
                parameters=[{
                }],
                remappings=[
                    ('/sbus_data', '/sbus/manual/sbus_data'),
                    ('sbus_manual', '/sbus/manual/sbus_data'),
                    ('sbus_auto', '/controller/control_input'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),

            # Auto SBUS処理グループ
            # SBUSオフセット適用ノード（機体用）
            ComposableNode(
                package='ksenos_ground',
                plugin='SbusOffsetApply',
                namespace='sbus/auto',
                name='sbus_offset_apply_node_to_aircraft',
                parameters=[{
                    'input_topic': '/controller/control_input',
                    'offset_topic': '/sbus/sbus_offset_amount',
                    'output_topic': 'sbus_radian_format',
                    'offset_operation': 'add',  # オフセットを足す
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # SBUSデータを生形式に変換
            ComposableNode(
                package='ksenos_ground',
                plugin='SbusDataToRawProcessor',
                namespace='sbus/auto',
                name='sbus_format_data_to_raw',
                parameters=[{
                    'servo_min_count': 352,
                    'servo_max_count': 1696,
                    'servo_min_rad': -0.5236,  # -30 degrees in radians
                    'servo_max_rad': 0.5236,   # 30 degrees in radians
                    'throttle_min_count': 352,
                    'throttle_max_count': 1696,
                    'throttle_min_openness': 0.0,
                    'throttle_max_openness': 1.0,
                }],
                remappings=[
                    ('sbus_data', 'sbus_radian_format'),
                    ('sbus_raw_data', 'sbus_raw_format_input'),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            
            # SBUS生データ送信ノード
            ComposableNode(
                package='ksenos_ground',
                plugin='SbusUdpSender',
                namespace='sbus/auto',
                name='sbus_udp_sender',
                parameters=[{
                    'udp_ip': '192.168.10.251',  # ArduinoのIPアドレス
                    'udp_port': 9999,       # Arduinoの受信ポート
                }],
                remappings=[
                    ('sbus_raw_data', 'sbus_raw_format_input'),
                    ('sbus_manual', '/sbus/manual/sbus_data'),
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
