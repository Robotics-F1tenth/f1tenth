cmake_minimum_required(VERSION 3.8)
project(ekf_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ekf_pose_estimator
    src/main.cpp
    src/ekf_pose_estimator.cpp
)

target_include_directories(ekf_pose_estimator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ekf_pose_estimator
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    Eigen3
)

install(TARGETS ekf_pose_estimator
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(PROGRAMS
    scripts/ekf_evaluator.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ekf_evaluator
)

# IMU 축 규약 검사 — 차 위에서 바로 돌린다 (mocap 불필요).
#   ros2 run ekf_pose imu_frame_test
install(PROGRAMS
    scripts/imu_frame_test.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME imu_frame_test
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
