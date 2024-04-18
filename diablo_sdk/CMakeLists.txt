cmake_minimum_required(VERSION 3.6.2)
project(diablo_sdk)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib

        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp roslib std_msgs
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(

)

add_executable(${PROJECT_NAME}
        src/mian.cpp
        src/DiabloCtrl.cpp
        src/OSDKCRC.cpp
        src/OSDKHAL.cpp
        src/OSDKLog.cpp
        src/OSDKMovement.cpp
        src/OSDKPid.cpp
        src/OSDKTelemetry.cpp
        src/OSDKVehicle.cpp
        src/VulcanSerial/SerialPort.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
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

install(DIRECTORY include/VulcanSerial/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
