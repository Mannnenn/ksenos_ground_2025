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

include_directories(${PCL_INCLUDE_DIRS})
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

# Install launch files and config files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_auto_package()