cmake_minimum_required(VERSION 3.8)
project(hector_multi_robot_tools)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

# Register scripts hook
if (WIN32)
  message(FATAL_ERROR "Windows is currently not supported.")
else()
  ament_environment_hooks(env_hooks/50.scripts.bash.in)
endif()

include_directories(include)

# add prefix_tf_republisher_node library and executable
add_library(prefix_tf_republisher_node SHARED
  include/${PROJECT_NAME}/prefix_tf_republisher_node.hpp
  src/prefix_tf_republisher_node.cpp
)
ament_target_dependencies(prefix_tf_republisher_node PUBLIC rclcpp tf2_msgs)

add_executable(prefix_tf_republisher src/prefix_tf_republisher.cpp)
target_link_libraries(prefix_tf_republisher PUBLIC prefix_tf_republisher_node)

# Install library, executable and include folder
install(DIRECTORY include/ DESTINATION include)
install(TARGETS prefix_tf_republisher_node EXPORT ${PROJECT_NAME}-targets LIBRARY DESTINATION lib)
install(TARGETS prefix_tf_republisher DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY
  launch
  scripts
  env_hooks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

