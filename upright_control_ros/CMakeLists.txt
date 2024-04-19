cmake_minimum_required(VERSION 3.0.2)
project(upright_control_ros)

set(CATKIN_PACKAGE_DEPENDENCIES
        roslib
        visualization_msgs
        ocs2_ros_interfaces
        controller_interface
        angles
        tf2_ros
        tf2_geometry_msgs
        effort_controllers
        pluginlib

        qz_hw
        upright_common
        upright_control
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

# main library
add_library(${PROJECT_NAME}
        src/MobileManipulatorVisualization.cpp
        src/synchronized_module/RosReferenceManager.cpp
)
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Controller library
add_library(mobile_manipulator_controller
        src/MobileManipulatorController.cpp
)
add_dependencies(mobile_manipulator_controller
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mobile_manipulator_controller
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)
target_compile_options(mobile_manipulator_controller PUBLIC ${OCS2_CXX_FLAGS})


add_library(diablo_qz_controller
        src/DiabloQzController.cpp
)
add_dependencies(diablo_qz_controller
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(diablo_qz_controller
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        pinocchio::pinocchio
)
target_compile_options(diablo_qz_controller PUBLIC ${OCS2_CXX_FLAGS})


# Mobile manipulator target node
add_executable(ddt_mobile_manipulator_target
        src/DDTMobileManipulatorTarget.cpp
)
add_dependencies(ddt_mobile_manipulator_target
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ddt_mobile_manipulator_target
        ${catkin_LIBRARIES}
)
target_compile_options(ddt_mobile_manipulator_target PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(mobile_manipulator_dummy_node
        src/DummyManipulatorNode.cpp
)
add_dependencies(mobile_manipulator_dummy_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mobile_manipulator_dummy_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)
target_compile_options(mobile_manipulator_dummy_node PRIVATE ${OCS2_CXX_FLAGS})

## Mpc node
add_executable(mobile_manipulator_mpc
        src/MobileManipulatorMpcNode.cpp
)
add_dependencies(mobile_manipulator_mpc
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mobile_manipulator_mpc
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)
target_compile_options(mobile_manipulator_mpc PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target rm_balance_control_ros")
    add_clang_tooling(
            TARGETS
            ${PROJECT_NAME}
            mobile_manipulator_mpc
            mobile_manipulator_dummy_node
            ddt_mobile_manipulator_target
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

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
install(TARGETS mobile_manipulator_dummy_node mobile_manipulator_mpc ddt_mobile_manipulator_target
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
