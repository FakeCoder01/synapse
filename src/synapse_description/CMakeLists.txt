cmake_minimum_required(VERSION 3.8)
project(synapse_description)

find_package(ament_cmake REQUIRED)

# Install URDF/Xacro files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes (if any)
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install package manifest for reference
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
