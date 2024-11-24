cmake_minimum_required(VERSION 3.8)
project(planner_module)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(yaml-cpp REQUIRED)


include_directories(include ${YAML_CPP_INCLUDE_DIR})

add_executable(nav2_client src/navigation/nav2_client.cpp
               src/main/nav2_main.cpp)
ament_target_dependencies(nav2_client
  rclcpp
  tf2_ros
  yaml-cpp
  nav_msgs
  nav2_msgs
  geometry_msgs
  rclcpp_action
  trajectory_msgs
  builtin_interfaces
)
target_link_libraries(nav2_client
  yaml-cpp
)

add_executable(arm_client src/planner_module/arm_controller.cpp
               src/main/arm_contoller_main.cpp)
ament_target_dependencies(arm_client
  rclcpp
  yaml-cpp
  control_msgs
  geometry_msgs
  rclcpp_action
  trajectory_msgs
  builtin_interfaces
)
target_link_libraries(arm_client
  yaml-cpp
)

install(TARGETS
  arm_client
  nav2_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
