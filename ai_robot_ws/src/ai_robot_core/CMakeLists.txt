cmake_minimum_required(VERSION 3.8)
project(ai_robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)



ament_package()

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}
)
