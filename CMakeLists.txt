cmake_minimum_required(VERSION 3.8)
project(ksenos_ground)

# Set policy CMP0074 to NEW to use <PackageName>_ROOT variables
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(rclcpp_components REQUIRED)

# LiDAR SDKのサブディレクトリを追加
# HesaiLidar_General_SDKでは特定のwarningを無視する
set(CMAKE_CXX_FLAGS_BACKUP ${CMAKE_CXX_FLAGS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare -Wno-unused-variable -Wno-missing-field-initializers")
add_subdirectory(include/pcd_proc_cpp/HesaiLidar_General_SDK)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_BACKUP})


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include/pcd_proc_cpp/HesaiLidar_General_SDK/include)
include_directories(include/pcd_proc_cpp/HesaiLidar_General_SDK/src/PandarGeneralRaw/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 初期スキャン地図化
ament_auto_add_library(init_scans_component SHARED src/pointcloud_pipeline/init_scans.cpp)
target_link_libraries(init_scans_component ${PCL_LIBRARIES})
ament_target_dependencies(init_scans_component pcl_conversions)
rclcpp_components_register_nodes(init_scans_component "LidarScanNode")
install(TARGETS init_scans_component
  DESTINATION lib/${PROJECT_NAME})

# 点群モーター座標系変換
ament_auto_add_library(pointcloud_transform_component SHARED src/pointcloud_pipeline/pointcloud_transform.cpp)
rclcpp_components_register_nodes(pointcloud_transform_component "PointCloudTransformer")
install(TARGETS pointcloud_transform_component
  DESTINATION lib/${PROJECT_NAME})

# 点群飛行機検出
ament_auto_add_library(detect_movable_objects_component SHARED src/pointcloud_pipeline/detect_movable_objects.cpp)
target_compile_features(detect_movable_objects_component PUBLIC c_std_99 cxx_std_17)
target_link_libraries(detect_movable_objects_component ${PCL_LIBRARIES})
ament_target_dependencies(detect_movable_objects_component pcl_conversions)
rclcpp_components_register_nodes(detect_movable_objects_component "MovableObjectDetector")
install(TARGETS detect_movable_objects_component
  DESTINATION lib/${PROJECT_NAME})

# 地面推定
ament_auto_add_library(correction_ground_component SHARED src/pointcloud_pipeline/correction_ground.cpp)
target_link_libraries(correction_ground_component ${PCL_LIBRARIES})
ament_target_dependencies(correction_ground_component pcl_conversions)
rclcpp_components_register_nodes(correction_ground_component "GroundCorrectionNode")
install(TARGETS correction_ground_component
  DESTINATION lib/${PROJECT_NAME})

# IMU水平補正
ament_auto_add_executable(leveling_imu src/pointcloud_pipeline/leveling_imu.cpp)
ament_target_dependencies(leveling_imu tf2_geometry_msgs)
install(TARGETS leveling_imu
  DESTINATION lib/${PROJECT_NAME})

# TF平滑化ノード
ament_auto_add_executable(tf_smoother src/tf_smoother.cpp)
install(TARGETS tf_smoother
  DESTINATION lib/${PROJECT_NAME})

# S.BUSデータ処理ノード（Raw to Rad）
ament_auto_add_library(sbus_format_data_to_radian_component SHARED src/sbus_data_processor/sbus_format_data_to_radian.cpp)
rclcpp_components_register_nodes(sbus_format_data_to_radian_component "SbusDataProcessor")
install(TARGETS sbus_format_data_to_radian_component
  DESTINATION lib/${PROJECT_NAME})

# S.BUSデータ処理ノード（Rad To Raw）
ament_auto_add_library(sbus_format_data_to_raw_component SHARED src/sbus_data_processor/sbus_format_data_to_raw.cpp)
rclcpp_components_register_nodes(sbus_format_data_to_raw_component "SbusDataToRawProcessor")
install(TARGETS sbus_format_data_to_raw_component
  DESTINATION lib/${PROJECT_NAME})

# S.BUSキャリブレーションノード
ament_auto_add_library(sbus_calibration_component SHARED src/sbus_data_processor/sbus_calibration.cpp)
rclcpp_components_register_nodes(sbus_calibration_component "SbusCalibration")
install(TARGETS sbus_calibration_component
  DESTINATION lib/${PROJECT_NAME})

# S.BUSオフセット適用ノード
ament_auto_add_library(sbus_offset_apply_component SHARED src/sbus_data_processor/sbus_offset_apply.cpp)
rclcpp_components_register_nodes(sbus_offset_apply_component "SbusOffsetApply")
install(TARGETS sbus_offset_apply_component
  DESTINATION lib/${PROJECT_NAME})

# S.BUS生データ読み出しノード
ament_auto_add_library(sbus_get_raw_data_component SHARED src/sbus_data_processor/sbus_get_raw_data.cpp)
rclcpp_components_register_nodes(sbus_get_raw_data_component "SbusSerialReader")
install(TARGETS sbus_get_raw_data_component
  DESTINATION lib/${PROJECT_NAME})

# S.BUS生データ送信ノード
ament_auto_add_library(sbus_send_raw_data_component SHARED src/sbus_data_processor/sbus_send_raw_data.cpp)
rclcpp_components_register_nodes(sbus_send_raw_data_component "SbusUdpSender")
install(TARGETS sbus_send_raw_data_component
  DESTINATION lib/${PROJECT_NAME})

# UDPパケット受信ノード
ament_auto_add_library(receive_udp_packet_component SHARED src/pose_estimation/receive_udp_packet.cpp)
rclcpp_components_register_nodes(receive_udp_packet_component "UDPReceiver")
install(TARGETS receive_udp_packet_component
  DESTINATION lib/${PROJECT_NAME})

# シミュレーションノード
ament_auto_add_executable(simulate_airplane_motion src/simulate_airplane_motion.cpp)
install(TARGETS simulate_airplane_motion
  DESTINATION lib/${PROJECT_NAME})

# 動翼角度パブリッシャー
ament_auto_add_library(publish_joint_airplane SHARED src/sbus_data_processor/publish_joint_airplane.cpp)
rclcpp_components_register_nodes(publish_joint_airplane "ControlToJointPublisher")
install(TARGETS publish_joint_airplane
  DESTINATION lib/${PROJECT_NAME})

# フィールド図面パブリッシャー
ament_auto_add_executable(pub_area src/auto_landing/pub_area.cpp)
install(TARGETS pub_area
  DESTINATION lib/${PROJECT_NAME})

# 角度投影パブリッシャー
ament_auto_add_library(project_orientation_component SHARED src/project_orientation.cpp)
rclcpp_components_register_nodes(project_orientation_component "TfProjectionNode")
install(TARGETS project_orientation_component
  DESTINATION lib/${PROJECT_NAME})


# Roll Pitch Yawパブリッシャー
ament_auto_add_executable(pub_rpy src/pub_rpy.cpp)
install(TARGETS pub_rpy
  DESTINATION lib/${PROJECT_NAME})

# ポイントクラウドパブリッシャー
ament_auto_add_library(pub_pointcloud_component SHARED src/pointcloud_pipeline/pub_pointcloud.cpp)
target_compile_features(pub_pointcloud_component PUBLIC c_std_99 cxx_std_17)
target_link_libraries(pub_pointcloud_component ${PCL_LIBRARIES} PandarGeneralSDK ${Boost_LIBRARIES})
ament_target_dependencies(pub_pointcloud_component pcl_conversions)
set_target_properties(pub_pointcloud_component PROPERTIES
    BUILD_RPATH "${CMAKE_BINARY_DIR}/include/pcd_proc_cpp/HesaiLidar_General_SDK"
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)
rclcpp_components_register_nodes(pub_pointcloud_component "PointCloudPublisher")

install(TARGETS pub_pointcloud_component
  DESTINATION lib/${PROJECT_NAME})



# クォータニオンからRPY計算ノード
ament_auto_add_library(calc_rpy_from_quat_component SHARED src/pose_estimation/calc_rpy_from_quat.cpp)
rclcpp_components_register_nodes(calc_rpy_from_quat_component "CalcRpyFromQuat")

install(TARGETS calc_rpy_from_quat_component
  DESTINATION lib/${PROJECT_NAME})

# 回転回数カウンター
ament_auto_add_library(yaw_angle_serialization_component SHARED src/pose_estimation/yaw_angle_serialization.cpp)
rclcpp_components_register_nodes(yaw_angle_serialization_component "RotationCounterNode")
install(TARGETS yaw_angle_serialization_component
  DESTINATION lib/${PROJECT_NAME})

# 旋回半径から横加速度を計算するノード
ament_auto_add_library(turningRadius_to_latAcc_component SHARED src/lat_control/turningRadius_to_latAcc.cpp)
rclcpp_components_register_nodes(turningRadius_to_latAcc_component "LateralAccelerationCalculator")
install(TARGETS turningRadius_to_latAcc_component
  DESTINATION lib/${PROJECT_NAME})

# 横加速度から目標ロール角を計算するノード
ament_auto_add_library(latAcc_to_targetRollAngle_component SHARED src/lat_control/latAcc_to_targetRollAngle.cpp)
rclcpp_components_register_nodes(latAcc_to_targetRollAngle_component "LatAccToTargetRollAngle")
install(TARGETS latAcc_to_targetRollAngle_component
  DESTINATION lib/${PROJECT_NAME})

# エルロン制御ノード（PD制御）
ament_auto_add_library(control_aileron_component SHARED src/lat_control/control_aileron.cpp)
rclcpp_components_register_nodes(control_aileron_component "AileronControl")
install(TARGETS control_aileron_component
  DESTINATION lib/${PROJECT_NAME})

# ラダー制御ノード（FF+PI制御）
ament_auto_add_library(control_rudder_component SHARED src/lat_control/control_rudder.cpp)
rclcpp_components_register_nodes(control_rudder_component "RudderControl")
install(TARGETS control_rudder_component
  DESTINATION lib/${PROJECT_NAME})

# 速度ターゲット生成ノード（位置xに応じて速度指令とYZ平面マーカーを出力）
ament_auto_add_library(control_speed_component SHARED src/auto_landing/control_speed.cpp)
rclcpp_components_register_nodes(control_speed_component "ControlSpeedNode")
install(TARGETS control_speed_component
  DESTINATION lib/${PROJECT_NAME})

# 投下ノード（位置xに応じて投下指令とYZ平面マーカーを出力）
ament_auto_add_library(control_drop_component SHARED src/auto_landing/control_drop.cpp)
rclcpp_components_register_nodes(control_drop_component "ControlDropNode")
install(TARGETS control_drop_component
  DESTINATION lib/${PROJECT_NAME})

# スロットル制御ノード（PI制御）
ament_auto_add_library(control_throttle_component SHARED src/long_control/control_throttle.cpp)
rclcpp_components_register_nodes(control_throttle_component "ThrottleControl")
install(TARGETS control_throttle_component
  DESTINATION lib/${PROJECT_NAME})


# エレベーター制御ノード
ament_auto_add_library(control_elevator_component SHARED src/long_control/control_elevator.cpp)
rclcpp_components_register_nodes(control_elevator_component "ElevatorControl")
install(TARGETS control_elevator_component
  DESTINATION lib/${PROJECT_NAME})

# 飛行距離計測ノード
ament_auto_add_library(calc_flight_distance_component SHARED src/auto_landing/calc_flight_distance.cpp)
rclcpp_components_register_nodes(calc_flight_distance_component "CalcFlightDistance")
install(TARGETS calc_flight_distance_component
  DESTINATION lib/${PROJECT_NAME})

# 自動旋回モードノード
ament_auto_add_library(mode_auto_turning SHARED src/mode_control/mode_auto_turning.cpp)
rclcpp_components_register_nodes(mode_auto_turning "ModeAutoTurning")
install(TARGETS mode_auto_turning
  DESTINATION lib/${PROJECT_NAME})


# 8の字旋回（角度基準）モードノード
ament_auto_add_library(mode_eight_turning_angle SHARED src/mode_control/mode_eight_turning_angle.cpp)
rclcpp_components_register_nodes(mode_eight_turning_angle "ModeEightTurningAngle")
install(TARGETS mode_eight_turning_angle
  DESTINATION lib/${PROJECT_NAME})

# 上昇旋回モード
ament_auto_add_library(mode_rise_turning SHARED src/mode_control/mode_rise_turning.cpp)
rclcpp_components_register_nodes(mode_rise_turning "ModeRiseTurning")
install(TARGETS mode_rise_turning
  DESTINATION lib/${PROJECT_NAME})


# モード別目標高度セレクター
ament_auto_add_library(mode_target_altitude_selector_component SHARED src/long_control/mode_target_altitude_selector.cpp)
rclcpp_components_register_nodes(mode_target_altitude_selector_component "ModeTargetAltitudeSelector")
install(TARGETS mode_target_altitude_selector_component
  DESTINATION lib/${PROJECT_NAME})


# エネルギー計算ノード
ament_auto_add_library(calc_energy_component SHARED src/long_control/calc_energy.cpp)
rclcpp_components_register_nodes(calc_energy_component "CalcEnergyNode")
install(TARGETS calc_energy_component
  DESTINATION lib/${PROJECT_NAME})


# 高度計算ノード（ToF傾き補正）
ament_auto_add_library(calc_altitude_imu_component SHARED src/pose_estimation/calc_altitude_imu.cpp)
rclcpp_components_register_nodes(calc_altitude_imu_component "CalcAltitudeNode")
install(TARGETS calc_altitude_imu_component
  DESTINATION lib/${PROJECT_NAME})

# 高度計算ノード（LiDAR）
ament_auto_add_library(calc_altitude_lidar_component SHARED src/pose_estimation/calc_altitude_lidar.cpp)
rclcpp_components_register_nodes(calc_altitude_lidar_component "AltitudeLidarNode")
install(TARGETS calc_altitude_lidar_component
  DESTINATION lib/${PROJECT_NAME})

# 高度計算ノード（気圧計）
ament_auto_add_executable(calc_altitude_pressure src/pose_estimation/calc_altitude_pressure.cpp)
install(TARGETS calc_altitude_pressure
  DESTINATION lib/${PROJECT_NAME})

# 平均高度・速度計算ノード
ament_auto_add_library(calc_average_altitude_velocity_component SHARED src/long_control/calc_average_altitude_velocity.cpp)
rclcpp_components_register_nodes(calc_average_altitude_velocity_component "AverageAltitudeVelocityNode")
install(TARGETS calc_average_altitude_velocity_component
  DESTINATION lib/${PROJECT_NAME})

# 経路生成ノード
ament_auto_add_library(path_generator_component SHARED src/auto_landing/path_generator.cpp)
rclcpp_components_register_nodes(path_generator_component "PathGenerator")
install(TARGETS path_generator_component
  DESTINATION lib/${PROJECT_NAME})

# L1 path following (eta/markers/optional lateral acceleration)
ament_auto_add_library(control_L1_law_component SHARED src/auto_landing/control_L1_law.cpp)
rclcpp_components_register_nodes(control_L1_law_component "L1ControlNode")
install(TARGETS control_L1_law_component
  DESTINATION lib/${PROJECT_NAME})

# Yaw角度修正器
ament_auto_add_executable(yaw_angle_correction src/auto_landing/yaw_angle_correction.cpp)
install(TARGETS yaw_angle_correction
  DESTINATION lib/${PROJECT_NAME})


# L1 path following (eta/markers/optional lateral acceleration)
ament_auto_add_executable(test_path_follow_with_joy src/auto_landing/test_path_follow_with_joy.cpp)
install(TARGETS test_path_follow_with_joy
  DESTINATION lib/${PROJECT_NAME})

# 制御入力統合ノード
ament_auto_add_library(unity_control_input_component SHARED src/long_control/unity_control_input.cpp)
rclcpp_components_register_nodes(unity_control_input_component "UnityControlInputNode")
install(TARGETS unity_control_input_component
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# コンポーネントライブラリのエクスポート
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  PCL
  Eigen3
  Boost
  serial
)

ament_auto_package()