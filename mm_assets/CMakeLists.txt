cmake_minimum_required(VERSION 3.0.2)
project(mm_assets)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        ocs2_mm
        ocs2_mm_ros
        urdf
        rviz
        upright_gazebo

)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp ocs2_mm ocs2_mm_ros urdf rviz upright_gazebo
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





