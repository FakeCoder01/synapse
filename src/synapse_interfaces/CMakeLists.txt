cmake_minimum_required(VERSION 3.8)
project(synapse_interfaces)

# Enable common warnings for C++ compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message and service files
set(msg_files
  "msg/DetectedPerson.msg"
  "msg/DetectedPersons.msg"
  "msg/EmotionState.msg"
)

set(srv_files
  "srv/AddPerson.srv"
  "srv/GetPersonByFace.srv"
  "srv/AddConversationMemory.srv"
  "srv/RetrieveRelevantMemories.srv"
  "srv/CaptureFaceEncoding.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# Export runtime deps for packages that consume these interfaces
ament_export_dependencies(rosidl_default_runtime)

# Install interface files for reference
install(DIRECTORY msg srv action
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
