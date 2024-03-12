cmake_minimum_required(VERSION 3.0.2)
project(mobile_manipulator_assets)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        ocs2_mobile_manipulator
        ocs2_mobile_manipulator_ros
        urdf
        rviz

)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp ocs2_mobile_manipulator ocs2_mobile_manipulator_ros urdf rviz
        DEPENDS
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

# Mark resource files for installation
install(
        DIRECTORY rviz config description launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark other files for installation
install(
        DIRECTORY rviz config description launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)





