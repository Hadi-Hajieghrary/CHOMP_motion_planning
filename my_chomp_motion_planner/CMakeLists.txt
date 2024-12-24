cmake_minimum_required(VERSION 3.22)
project(my_chomp_motion_planner)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)


include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/chomp_parameters.cpp
  src/chomp_trajectory.cpp
  src/chomp_optimizer.cpp
  src/chomp_planner.cpp
)
ament_target_dependencies(${PROJECT_NAME} 
  moveit_core
  rclcpp
  trajectory_msgs
  visualization_msgs
)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_package()