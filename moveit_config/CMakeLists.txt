cmake_minimum_required(VERSION 3.8)
project(moveit_config)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

include_directories(
                    include
                    ${EIGEN3_INCLUDE_DIRS}
                  )

## Robot Description Package
set(ROBOT_DESCRIPTION_PKG ur_description)
find_package(${ROBOT_DESCRIPTION_PKG} REQUIRED)


add_executable(chomp_planning src/chomp_planning_node.cpp)

target_include_directories(chomp_planning PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(chomp_planning Open3D::Open3D)

ament_target_dependencies(chomp_planning
                            rclcpp
                            moveit_core
                            moveit_ros_planning
                            moveit_ros_planning_interface
                            moveit_visual_tools
                            geometric_shapes
                            tf2_eigen
                            Python3
                          )

install(TARGETS chomp_planning
          DESTINATION lib/${PROJECT_NAME})


# Install the 'launch' folder
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the 'config' folder
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the 'rviz' folder
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

