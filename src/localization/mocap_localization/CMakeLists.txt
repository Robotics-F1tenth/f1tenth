cmake_minimum_required(VERSION 3.8)
project(mocap_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(mocap_localization src/mocap_localization.cpp)
ament_target_dependencies(mocap_localization
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mocap4r2_msgs)

target_include_directories(mocap_localization PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS mocap_localization
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config docs patches
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  calibration_v2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibration_v2)

install(PROGRAMS
  scripts/mocap_timing_probe.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mocap_timing_probe)

install(PROGRAMS
  scripts/mocap_bag_timing_report.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mocap_bag_timing_report)

install(PROGRAMS
  scripts/calibrate_shared_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibrate_shared_map)

install(PROGRAMS
  scripts/collect_shared_map_survey.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collect_shared_map_survey)

install(PROGRAMS
  scripts/collect_mcl_aligned_survey.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collect_mcl_aligned_survey)

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
