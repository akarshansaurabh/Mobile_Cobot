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
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(map_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ruckig REQUIRED)
find_package(kdl_parser REQUIRED)

include_directories(include 
  ${YAML_CPP_INCLUDE_DIR} 
  ${PCL_INCLUDE_DIR}
  ${ruckig_INCLUDE_DIRS}
  )
link_directories(${PCL_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})

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

add_executable(nav2_client2 src/navigation/nav2_client2.cpp
               src/main/nav2_main2.cpp)
ament_target_dependencies(nav2_client2
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
target_link_libraries(nav2_client2
  yaml-cpp
)


add_executable(pc_processing2 src/pc_processing/pc_processing_2.cpp
               src/main/pc_processing_main2.cpp)
ament_target_dependencies(pc_processing2
  rclcpp
  tf2_ros
  tf2_eigen
  sensor_msgs
  geometry_msgs
  rclcpp_action
  builtin_interfaces
  pcl_conversions
  pcl_ros
  tf2_sensor_msgs
  tf2
  std_msgs
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(pc_processing2
 ${PCL_LIBRARIES}
)

add_executable(pc_processing1 src/pc_processing/pointcloud_processor.cpp
               src/main/pc_processing_main1.cpp src/visualizations/visualization_manager.cpp)
ament_target_dependencies(pc_processing1
  rclcpp
  tf2_ros
  tf2_eigen
  sensor_msgs
  geometry_msgs
  rclcpp_action
  builtin_interfaces
  pcl_conversions
  pcl_ros
  tf2_sensor_msgs
  tf2
  std_msgs
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(pc_processing1
 ${PCL_LIBRARIES}
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

add_executable(obstacle_monitoring src/obstacle/obstacle_monitoring.cpp
               src/main/obstacle_monitoring_main.cpp)
ament_target_dependencies(obstacle_monitoring
  rclcpp
  tf2_ros
  tf2_eigen
  sensor_msgs
  geometry_msgs
  rclcpp_action
  builtin_interfaces
  tf2_sensor_msgs
  nav2_msgs
  nav_msgs
  nav2_costmap_2d
  map_msgs
  tf2
)


install(TARGETS
  pc_processing1
  pc_processing2
  arm_client
  nav2_client
  nav2_client2
  obstacle_monitoring
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
