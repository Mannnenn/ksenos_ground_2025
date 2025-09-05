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


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 初期スキャン地図化
ament_auto_add_executable(init_scans src/init_scans.cpp)
target_link_libraries(init_scans ${PCL_LIBRARIES})
ament_target_dependencies(init_scans pcl_conversions)
install(TARGETS init_scans
  DESTINATION lib/${PROJECT_NAME})

# 点群モーター座標系変換
ament_auto_add_executable(pointcloud_transform src/pointcloud_transform.cpp)
install(TARGETS pointcloud_transform
  DESTINATION lib/${PROJECT_NAME})

# 点群飛行機検出
ament_auto_add_executable(detect_movable_objects src/detect_movable_objects.cpp)
target_compile_features(detect_movable_objects PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(detect_movable_objects ${PCL_LIBRARIES})
ament_target_dependencies(detect_movable_objects pcl_conversions)
install(TARGETS detect_movable_objects
  DESTINATION lib/${PROJECT_NAME})

# 地面推定
ament_auto_add_executable(correction_ground src/correction_ground.cpp)
target_link_libraries(correction_ground ${PCL_LIBRARIES})
ament_target_dependencies(correction_ground pcl_conversions)
install(TARGETS correction_ground
  DESTINATION lib/${PROJECT_NAME})

# 速度推定ノード
ament_auto_add_executable(estimate_vel src/estimate_vel.cpp)
target_link_libraries(estimate_vel Eigen3::Eigen)
install(TARGETS estimate_vel
  DESTINATION lib/${PROJECT_NAME})

# TF平滑化ノード
ament_auto_add_executable(tf_smoother src/tf_smoother.cpp)
install(TARGETS tf_smoother
  DESTINATION lib/${PROJECT_NAME})

# S.BUS生データ読み出しノード
ament_auto_add_executable(sbus_get_raw_data src/sbus_get_raw_data.cpp)
install(TARGETS sbus_get_raw_data
  DESTINATION lib/${PROJECT_NAME})

# S.BUSデータ処理ノード（Raw to Rad）
ament_auto_add_executable(sbus_format_data_to_radian src/sbus_format_data_to_radian.cpp)
install(TARGETS sbus_format_data_to_radian
  DESTINATION lib/${PROJECT_NAME})

# S.BUSデータ処理ノード（Rad To Raw）
ament_auto_add_executable(sbus_format_data_to_raw src/sbus_format_data_to_raw.cpp)
install(TARGETS sbus_format_data_to_raw
  DESTINATION lib/${PROJECT_NAME})

# S.BUSキャリブレーションノード
ament_auto_add_executable(sbus_calibration_node src/sbus_calibration_node.cpp)
install(TARGETS sbus_calibration_node
  DESTINATION lib/${PROJECT_NAME})

# S.BUSオフセット適用ノード
ament_auto_add_executable(sbus_offset_apply_node src/sbus_offset_apply_node.cpp)
install(TARGETS sbus_offset_apply_node
  DESTINATION lib/${PROJECT_NAME})

# UDPパケット受信ノード
ament_auto_add_executable(receive_udp_packet src/receive_udp_packet.cpp)
install(TARGETS receive_udp_packet
  DESTINATION lib/${PROJECT_NAME})

# シミュレーションノード
ament_auto_add_executable(simulate_airplane_motion src/simulate_airplane_motion.cpp)
install(TARGETS simulate_airplane_motion
  DESTINATION lib/${PROJECT_NAME})

# 動翼角度パブリッシャー
ament_auto_add_executable(publish_joint_airplane src/publish_joint_airplane.cpp)
install(TARGETS publish_joint_airplane
  DESTINATION lib/${PROJECT_NAME})

# フィールド図面パブリッシャー
ament_auto_add_executable(pub_area src/pub_area.cpp)
install(TARGETS pub_area
  DESTINATION lib/${PROJECT_NAME})

# 角度投影パブリッシャー
ament_auto_add_executable(project_orientation src/project_orientation.cpp)
install(TARGETS project_orientation
  DESTINATION lib/${PROJECT_NAME})


# Roll Pitch Yawパブリッシャー
ament_auto_add_executable(pub_rpy src/pub_rpy.cpp)
install(TARGETS pub_rpy
  DESTINATION lib/${PROJECT_NAME})

# クォータニオンからRPY計算ノード
ament_auto_add_executable(calc_rpy_from_quat src/calc_rpy_from_quat.cpp)
install(TARGETS calc_rpy_from_quat
  DESTINATION lib/${PROJECT_NAME})

# 回転回数カウンター
# ament_auto_add_executable(rotation_counter src/rotation_counter.cpp)
# install(TARGETS rotation_counter
#  DESTINATION lib/${PROJECT_NAME})

# 回転回数カウンター
ament_auto_add_executable(yaw_angle_serialization src/yaw_angle_serialization.cpp)
install(TARGETS yaw_angle_serialization
  DESTINATION lib/${PROJECT_NAME})

# 旋回半径から横加速度を計算するノード
ament_auto_add_executable(turningRadius_to_latAcc src/turningRadius_to_latAcc.cpp)
install(TARGETS turningRadius_to_latAcc
  DESTINATION lib/${PROJECT_NAME})

# 横加速度から目標ロール角を計算するノード
ament_auto_add_executable(latAcc_to_targetRollAngle src/latAcc_to_targetRollAngle.cpp)
install(TARGETS latAcc_to_targetRollAngle
  DESTINATION lib/${PROJECT_NAME})

# エルロン制御ノード（PD制御）
ament_auto_add_executable(control_aileron src/control_aileron.cpp)
install(TARGETS control_aileron
  DESTINATION lib/${PROJECT_NAME})

# ラダー制御ノード（FF+PI制御）
ament_auto_add_executable(control_rudder src/control_rudder.cpp)
install(TARGETS control_rudder
  DESTINATION lib/${PROJECT_NAME})

# スロットル制御ノード（PI制御）
ament_auto_add_executable(control_throttle src/control_throttle.cpp)
install(TARGETS control_throttle
  DESTINATION lib/${PROJECT_NAME})

# エレベーター制御ノード
ament_auto_add_executable(control_elevator src/control_elevator.cpp)
install(TARGETS control_elevator
  DESTINATION lib/${PROJECT_NAME})

# 飛行距離計測ノード
ament_auto_add_executable(calc_flight_distance src/calc_flight_distance.cpp)
install(TARGETS calc_flight_distance
  DESTINATION lib/${PROJECT_NAME})

# 自動旋回モードノード
ament_auto_add_executable(mode_auto_turning src/mode_auto_turning.cpp)
install(TARGETS mode_auto_turning
  DESTINATION lib/${PROJECT_NAME})

# 8の字旋回モードノード
ament_auto_add_executable(mode_eight_turning src/mode_eight_turning.cpp)
install(TARGETS mode_eight_turning
  DESTINATION lib/${PROJECT_NAME})

# 上昇旋回モード
ament_auto_add_executable(mode_rise_turning src/mode_rise_turning.cpp)
install(TARGETS mode_rise_turning
  DESTINATION lib/${PROJECT_NAME})


# エネルギー計算ノード
ament_auto_add_executable(calc_energy src/calc_energy.cpp)
install(TARGETS calc_energy
  DESTINATION lib/${PROJECT_NAME})

# 高度計算ノード（ToF傾き補正）
ament_auto_add_executable(calc_altitude_imu src/calc_altitude_imu.cpp)
install(TARGETS calc_altitude_imu
  DESTINATION lib/${PROJECT_NAME})

# 高度計算ノード（LiDAR）
ament_auto_add_executable(calc_altitude_lidar src/calc_altitude_lidar.cpp)
install(TARGETS calc_altitude_lidar
  DESTINATION lib/${PROJECT_NAME})

# 高度計算ノード（気圧計）
ament_auto_add_executable(calc_altitude_pressure src/calc_altitude_pressure.cpp)
install(TARGETS calc_altitude_pressure
  DESTINATION lib/${PROJECT_NAME})

# 平均高度・速度計算ノード
ament_auto_add_executable(calc_average_altitude_velocity src/calc_average_altitude_velocity.cpp)
install(TARGETS calc_average_altitude_velocity
  DESTINATION lib/${PROJECT_NAME})

# 制御入力統合ノード
ament_auto_add_executable(unity_control_input src/unity_control_input.cpp)
install(TARGETS unity_control_input
  DESTINATION lib/${PROJECT_NAME})

# SBUS UDP送信ノード
ament_auto_add_executable(sbus_send_raw_data src/sbus_send_raw_data.cpp)
install(TARGETS sbus_send_raw_data
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


ament_auto_package()