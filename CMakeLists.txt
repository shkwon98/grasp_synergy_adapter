cmake_minimum_required(VERSION 3.15)
project(grasp_synergy_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(grasp_synergy_adapter src/grasp_synergy_adapter.cpp)
target_compile_features(grasp_synergy_adapter PRIVATE cxx_std_20)
target_compile_options(grasp_synergy_adapter PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(grasp_synergy_adapter PRIVATE src)
target_link_libraries(
  grasp_synergy_adapter
  PRIVATE rclcpp::rclcpp
          rclcpp_action::rclcpp_action
          control_msgs::control_msgs__rosidl_typesupport_cpp
          trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp)

install(TARGETS grasp_synergy_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  FILES README.md
        CHANGELOG.rst
        CONTRIBUTING.md
        SECURITY.md
        LICENSE
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE
      "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format")

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_grasp_profile test/test_grasp_profile.cpp)
  target_compile_features(test_grasp_profile PRIVATE cxx_std_20)
  target_compile_options(test_grasp_profile PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_grasp_profile PRIVATE src)
  target_link_libraries(
    test_grasp_profile trajectory_msgs::trajectory_msgs__rosidl_generator_cpp)

  ament_add_pytest_test(
    test_grasp_synergy_adapter test/test_grasp_synergy_adapter.py)
endif()

ament_package()
