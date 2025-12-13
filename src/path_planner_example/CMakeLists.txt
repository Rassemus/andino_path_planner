cmake_minimum_required(VERSION 3.8)
project(path_planner_example)

if(NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# ROS 2 AMENT/CMAKE-paketit
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(create_plan_msgs REQUIRED) 
find_package(pluginlib REQUIRED)
include("${pluginlib_DIR}/pluginlib_export_plugin_description_file.cmake")

# ====================================================================
# AMENT_PYTHON - YHTEENSOPIVUUS
# ====================================================================
# ament_python_install_package(path_planner_example)

# ====================================================================
# C++ PLUGININ MÄÄRITTELY JA KÄÄNTÄMINEN
# ====================================================================

add_library(path_planner_example SHARED path_planner_service_client.cpp)

ament_target_dependencies(
  path_planner_example
  rclcpp
  nav2_core
  nav_msgs
  pluginlib
  tf2_ros
  nav2_costmap_2d
  create_plan_msgs
)

install(TARGETS
  path_planner_example
  DESTINATION lib
)

# TÄMÄ ASENTAA XML-TIEDOSTON
install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS path_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(path_planner_example global_planner_plugin.xml)

ament_export_dependencies(pluginlib)

ament_export_targets()

ament_package()