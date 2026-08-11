cmake_minimum_required(VERSION 3.8)
project(drone_mapper)

# Suppress CMake policy warnings from PCL
cmake_policy(SET CMP0144 NEW)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Force C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# Find external dependencies
find_package(PCL REQUIRED COMPONENTS common io)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# --- map_publisher ---
add_executable(map_publisher src/map_publisher.cpp src/map_publisher_main.cpp)
target_include_directories(map_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(map_publisher
  ${PCL_LIBRARIES}
)
ament_target_dependencies(map_publisher
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  ament_index_cpp
  rclcpp_components
)
install(TARGETS map_publisher DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install global config.yaml alongside the package
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config.yaml
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/convert_ply_to_pcd.py
  scripts/convert_ply_to_pcd_with_origin.py
  scripts/downsample_pcd.py
  scripts/check_cloud.py
  scripts/drone_localization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz config
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING
    PATTERN "*.pcd"
    PATTERN "*.json"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_drone_mapper test/test_drone_mapper.cpp)
  # ament_target_dependencies(test_drone_mapper rclcpp)
endif()

ament_package()