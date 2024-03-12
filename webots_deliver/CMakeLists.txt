cmake_minimum_required(VERSION 3.6.2)
project(webots_deliver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(WEBOTS_HOME /usr/local/webots)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib

  std_msgs
  ocs2_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy roslib std_msgs ocs2_msgs
  DEPENDS
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${WEBOTS_HOME}/include/controller/cpp
)

link_directories(
        ${WEBOTS_HOME}/lib/controller
)

add_executable(${PROJECT_NAME}
        src/webots_deliver.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  CppController
)
#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
