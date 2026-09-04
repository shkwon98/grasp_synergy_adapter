# SPDX-FileCopyrightText: 2026 Sunghyun Kwon
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.15)
project(grasp_synergy_adapter)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(grasp_synergy_adapter src/grasp_synergy_adapter.cpp)
target_compile_features(grasp_synergy_adapter PRIVATE cxx_std_20)
target_compile_options(grasp_synergy_adapter PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(grasp_synergy_adapter PRIVATE src)
ament_target_dependencies(
  grasp_synergy_adapter
  control_msgs
  rclcpp
  rclcpp_action
  trajectory_msgs)

install(TARGETS grasp_synergy_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md CHANGELOG.rst LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_grasp_profile test/test_grasp_profile.cpp)
  target_compile_features(test_grasp_profile PRIVATE cxx_std_20)
  target_compile_options(test_grasp_profile PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_grasp_profile PRIVATE src)
  ament_target_dependencies(test_grasp_profile trajectory_msgs)

  ament_add_pytest_test(
    test_grasp_synergy_adapter test/test_grasp_synergy_adapter.py)
endif()

ament_package()
