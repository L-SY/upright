cmake_minimum_required(VERSION 3.6.2)
project(diablo_hw)

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

add_executable(${PROJECT_NAME}
        src/main.cpp
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

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
