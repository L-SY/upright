cmake_minimum_required(VERSION 3.10.2)
project(upright_control)


# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#add_definitions(-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_definitions(-DBOOST_MPL_LIMIT_LIST_SIZE=30)


set(CATKIN_PACKAGE_DEPENDENCIES
        tf
        urdf
        kdl_parser
        robot_state_publisher
        visualization_msgs
        geometry_msgs
        ocs2_ros_interfaces
        ocs2_core
        ocs2_ddp
        ocs2_mpc
        ocs2_sqp
        ocs2_oc
        ocs2_robotic_tools
        ocs2_pinocchio_interface
        ocs2_self_collision
        ocs2_self_collision_visualization
        roscpp

        mobile_manipulator_assets
        upright_core
)

find_package(catkin REQUIRED COMPONENTS
        roslib
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
)

find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS Boost pinocchio
)

########### ## Build ##
###########

set(FLAGS ${OCS2_CXX_FLAGS} ${pinocchio_CFLAGS_OTHER} -Wno-invalid-partial-specialization -DPINOCCHIO_WITH_HPP_FCL)

# setting CMAKE_CXX_STANDARD is the most "powerful" way to enforce a particular
# C++ version, in that it overwrites other options such as set_compile
# add_compile_options(-std=c++14). It is included here to force the use of
# C++14 over C++11 enforced in the same way by OCS2. We need C++14 to compile
# CGAL.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${hpp-fcl_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

# upright_control interface library
add_library(${PROJECT_NAME} SHARED
        src/InertialAlignment.cpp
        src/MobileManipulatorInterface.cpp
        src/constraint/BoundedBalancingConstraints.cpp
        src/constraint/EndEffectorConstraint.cpp
)
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

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

#############
## Testing ##
#############
add_executable(${PROJECT_NAME}_test
        test/TestConfig.cpp)

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES})
#if (CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(${PROJECT_NAME}_test
#            test/TestConfig.cpp
#    )
#
#    target_link_libraries(${PROJECT_NAME}_test
#            ${PROJECT_NAME}
#            gtest_main
#    )
#
#    target_compile_options(${PROJECT_NAME}_test PUBLIC ${FLAGS})
#endif (CATKIN_ENABLE_TESTING)
