cmake_minimum_required(VERSION 3.0.2)
project(ros_industrial_training)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  move_base_msgs
  tf
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs move_base_msgs tf visualization_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ar_navigation src/ar_navigation.cpp)
add_dependencies(ar_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ar_navigation ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS ar_navigation
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)