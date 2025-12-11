cmake_minimum_required(VERSION 3.8)
project(path_planner_example)

if(NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
  # Standardit liput
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# ROS 2 AMENT/CMAKE-paketit
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Oman palveluviestipaketin vaatimus
find_package(create_plan_msgs REQUIRED) 

# ====================================================================
# AMENT_PYTHON - YHTEENSOPIVUUS
# Tärkeä: Jotta setup.py toimii
# ====================================================================

# Vaatii tämän, jotta ament_cmake tietää, miten käsitellä Python-asennusta
ament_python_install_package(path_planner_example)


# ====================================================================
# C++ PLUGININ MÄÄRITTELY JA KÄÄNTÄMINEN
# ====================================================================

# 1. Kirjaston luonti src/path_planner_service_client.cpp -tiedostosta
add_library(path_planner_service_client SHARED
  src/path_planner_service_client.cpp
)

# 2. Riippuvuudet C++-kirjastolle
ament_target_dependencies(path_planner_service_client
  rclcpp
  nav2_core
  nav_msgs
  pluginlib
  tf2_ros
  nav2_costmap_2d
  create_plan_msgs
)

# 3. Asenna C++-kirjasto
install(TARGETS
  path_planner_service_client
  DESTINATION lib
)

# 4. Asenna Plugin XML-tiedosto
install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Amentin lopputoiminnot
ament_export_targets(all)