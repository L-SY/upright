cmake_minimum_required(VERSION 3.10)
project(rm_hw)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-enum-compare)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        geometry_msgs
        ps5_joy
        rm_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        upright_common

)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        geometry_msgs
        roscpp
        ps5_joy
        rm_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        upright_common
        DEPENDS
        LIBRARIES
        rm_hw
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE sources "src/*.cpp")

add_library(rm_hw SHARED ${sources})

target_link_libraries(rm_hw ${catkin_LIBRARIES})

add_executable(testPS5 test/testPS5Joy.cpp)

target_link_libraries(testPS5 ${catkin_LIBRARIES})

add_executable(test_rm test/demo.cpp)

target_link_libraries(test_rm ${catkin_LIBRARIES} rm_hw)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
#install(
#        DIRECTORY doc
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)



