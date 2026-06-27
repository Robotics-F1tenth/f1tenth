cmake_minimum_required(VERSION 3.8)
project(als_ros)

# Require C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(f1_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  
)

# Executables and their source files
add_executable(mcl src/mcl.cpp)
add_executable(tf_test src/tf_test.cpp)
add_executable(map_publisher src/map_publisher.cpp)
add_executable(state_ekf src/state_ekf.cpp)
# add_executable(mrf_failure_detector src/mrf_failure_detector.cpp)

add_library(extended_kalman_filter src/extended_kalman_filter.cpp)
add_library(extended_kalman_filter_odom src/extended_kalman_filter_odom.cpp)



target_link_libraries(mcl 
yaml-cpp
extended_kalman_filter
extended_kalman_filter_odom
)

target_link_libraries(map_publisher
yaml-cpp
)

ament_target_dependencies(mcl
  rclcpp
  cv_bridge
  # pcl_conversions
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  # PCL
  yaml-cpp
  visualization_msgs
  f1_msgs
)

ament_target_dependencies(map_publisher
  rclcpp
  nav_msgs
  yaml-cpp
)

ament_target_dependencies(tf_test
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(extended_kalman_filter
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  Eigen3
  )

ament_target_dependencies(extended_kalman_filter_odom
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  Eigen3
  )

ament_target_dependencies(state_ekf
  rclcpp
  f1_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
)



# ament_target_dependencies(mrf_failure_detector
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   visualization_msgs
# )


# Install targets
install(TARGETS
  mcl
  tf_test
  map_publisher
  state_ekf
  # mrf_failure_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_autosaver.py
  scripts/evaluate_localization.py
  scripts/optimize_mcl_params.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory and include files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)



# # Install directories
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

# Export package
ament_package()
