cmake_minimum_required(VERSION 3.0.2)
project(qz_hw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################################################################
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
###################################################################################


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        message_generation
        message_runtime
        tf
)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        hybrid_force.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs message_generation message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCES})

include_directories(
        include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE EXAMPLE_SOURCES "examples/*.cpp")
foreach (source ${EXAMPLE_SOURCES})
    get_filename_component(NAME ${source} NAME_WE)
    add_executable(${NAME} ${source})
    target_link_libraries(${NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endforeach ()
