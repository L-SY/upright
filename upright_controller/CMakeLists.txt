cmake_minimum_required(VERSION 3.0.2)
project(upright_controller)

set(CATKIN_PACKAGE_DEPENDENCIES
        roslib
        visualization_msgs
        ocs2_ros_interfaces
        controller_interface
        position_controllers
        angles
        tf2_ros
        tf2_geometry_msgs
        effort_controllers
        pluginlib

        upright_hw
        upright_common
        upright_control
        upright_control_ros
        arm_pinocchio_interface
)
find_package(pinocchio REQUIRED)
add_definitions(-DBOOST_MPL_LIMIT_LIST_SIZE=30)

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        LIBRARIES
        ${PROJECT_NAME}
        DEPENDS
        Boost
)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Controller library
add_library(upright_controller
        src/UprightController.cpp
)
add_dependencies(upright_controller
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(upright_controller
        ${catkin_LIBRARIES}
)
target_compile_options(upright_controller PUBLIC ${OCS2_CXX_FLAGS})

# Interpolation library
add_library(spline_interpolation
        src/interpolation/CubicSpline.cpp
        src/interpolation/MultiJointTrajectory.cpp
)
add_dependencies(spline_interpolation
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(spline_interpolation
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} spline_interpolation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        FILES upright_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)