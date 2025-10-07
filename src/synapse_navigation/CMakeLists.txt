cmake_minimum_required(VERSION 3.8)
project(synapse_navigation)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files (Nav2 params, costmaps, etc.)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
