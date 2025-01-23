cmake_minimum_required(VERSION 3.8)
project(hector_multi_robot_tools)

# Find necessary packages
find_package(ament_cmake REQUIRED)

if (WIN32)
  message(FATAL_ERROR "Windows is currently not supported.")
else()
  ament_environment_hooks(env_hooks/50.scripts.bash.in)
endif()

# Install directories
install(DIRECTORY
  launch
  scripts
  env_hooks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

