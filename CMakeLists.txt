cmake_minimum_required(VERSION 3.8)
project(gazebo_joint_torsional_spring_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

# Build the torsional spring plugin
add_library(gazebo_joint_torsional_spring SHARED
  src/gazebo_joint_torsional_spring.cpp
)

target_link_libraries(gazebo_joint_torsional_spring
  ${GAZEBO_LIBRARIES}
)

ament_target_dependencies(gazebo_joint_torsional_spring
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)

# Install the plugin library
install(TARGETS gazebo_joint_torsional_spring
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# Install launch files, worlds, etc.
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()