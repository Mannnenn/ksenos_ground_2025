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


ament_auto_add_executable(init_scans src/init_scans.cpp)
target_compile_features(init_scans PUBLIC c_std_99 cxx_std_17)
target_link_libraries(init_scans ${PCL_LIBRARIES})
ament_target_dependencies(init_scans pcl_conversions)
install(TARGETS init_scans
  DESTINATION lib/${PROJECT_NAME})


ament_auto_add_executable(pointcloud_transform src/pointcloud_transform.cpp)
install(TARGETS pointcloud_transform
  DESTINATION lib/${PROJECT_NAME})

ament_auto_add_executable(detect_movable_objects src/detect_movable_objects.cpp)
target_compile_features(detect_movable_objects PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(detect_movable_objects ${PCL_LIBRARIES})
ament_target_dependencies(detect_movable_objects pcl_conversions)
install(TARGETS detect_movable_objects
  DESTINATION lib/${PROJECT_NAME})

ament_auto_add_executable(correction_ground src/correction_ground.cpp)
target_compile_features(correction_ground PUBLIC c_std_99 cxx_std_17)
target_link_libraries(correction_ground ${PCL_LIBRARIES})
ament_target_dependencies(correction_ground pcl_conversions)
install(TARGETS correction_ground
  DESTINATION lib/${PROJECT_NAME})

# 速度推定ノード
ament_auto_add_executable(estimate_vel src/estimate_vel.cpp)
target_compile_features(estimate_vel PUBLIC c_std_99 cxx_std_17)
target_link_libraries(estimate_vel Eigen3::Eigen)
install(TARGETS estimate_vel
  DESTINATION lib/${PROJECT_NAME})

# TF平滑化ノード
ament_auto_add_executable(tf_smoother src/tf_smoother.cpp)
target_compile_features(tf_smoother PUBLIC c_std_99 cxx_std_17)
install(TARGETS tf_smoother
  DESTINATION lib/${PROJECT_NAME})

# S.BUS生データ読み出しノード
ament_auto_add_executable(get_sbus_raw_data src/get_sbus_raw_data.cpp)
target_compile_features(get_sbus_raw_data PUBLIC c_std_99 cxx_std_17)
install(TARGETS get_sbus_raw_data
  DESTINATION lib/${PROJECT_NAME})

# S.BUSデータ処理ノード
ament_auto_add_executable(proc_sbus_data src/proc_sbus_data.cpp)
target_compile_features(proc_sbus_data PUBLIC c_std_99 cxx_std_17)
install(TARGETS proc_sbus_data
  DESTINATION lib/${PROJECT_NAME})

# S.BUSデータオフセットノード
ament_auto_add_executable(offset_sbus_data src/offset_sbus_data.cpp)
target_compile_features(offset_sbus_data PUBLIC c_std_99 cxx_std_17)
install(TARGETS offset_sbus_data
  DESTINATION lib/${PROJECT_NAME})

# UDPパケット受信ノード
ament_auto_add_executable(receive_udp_packet src/receive_udp_packet.cpp)
target_compile_features(receive_udp_packet PUBLIC c_std_99 cxx_std_17)
install(TARGETS receive_udp_packet
  DESTINATION lib/${PROJECT_NAME})

# 姿勢推定ノード（カルマンフィルター）
ament_auto_add_executable(estimate_pose src/estimate_pose.cpp)
target_compile_features(estimate_pose PUBLIC c_std_99 cxx_std_17)
target_link_libraries(estimate_pose Eigen3::Eigen)
install(TARGETS estimate_pose
  DESTINATION lib/${PROJECT_NAME})

# カルマンゲイン推定ノード
ament_auto_add_executable(estimate_kalman_gain src/estimate_kalman_gain.cpp)
target_compile_features(estimate_kalman_gain PUBLIC c_std_99 cxx_std_17)
target_link_libraries(estimate_kalman_gain Eigen3::Eigen)
install(TARGETS estimate_kalman_gain
  DESTINATION lib/${PROJECT_NAME})

# シミュレーションノード
ament_auto_add_executable(simulate_airplane_motion src/simulate_airplane_motion.cpp)
target_compile_features(simulate_airplane_motion PUBLIC c_std_99 cxx_std_17)
install(TARGETS simulate_airplane_motion
  DESTINATION lib/${PROJECT_NAME})

# 動翼角度パブリッシャー
ament_auto_add_executable(publish_joint_airplane src/publish_joint_airplane.cpp)
target_compile_features(publish_joint_airplane PUBLIC c_std_99 cxx_std_17)
install(TARGETS publish_joint_airplane
  DESTINATION lib/${PROJECT_NAME})

# 気圧による高度推定ノード
ament_auto_add_executable(estimate_altitude_prresure src/estimate_altitude_prresure.cpp)
target_compile_features(estimate_altitude_prresure PUBLIC c_std_99 cxx_std_17)
install(TARGETS estimate_altitude_prresure
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