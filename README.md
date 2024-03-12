# Upright

Use https://github.com/leggedrobotics/ocs2 and https://github.com/ros-controls/ros_control to control ridgeback_ur5(The model is from https://github.com/leggedrobotics/ocs2_robotic_assets) in https://github.com/cyberbotics/webots.

## Introduction

- Upright largely references https://github.com/utiasDSL/upright and https://github.com/qiayuanl/legged_control, them are quite impressive.
- Upright is an MPC mobile manipulator robot controller which is based on OCS2 and simulate in Webots.

## Install

#### Source code

The source code is hosted on GitHub: https://github.com/L-SY/upright

```
# Clone upright use ssh
git clone git@github.com:L-SY/upright.git
```



#### OCS2

OCS2 is a huge monorepo,It is no need to follow the official guidelines to download all assembly.

After clone the `upright`, follow the steps:

```
# Make a new workspace to store the dependent libraries
mkdir -p upright_depend_ws/src

# set install mode
cd depend_ws/src
catkin install

catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build

# Clone OCS2 
# It is better to delete the ocs2_doc, for it may bring error when build.
# The https://github.com/utiasDSL/upright owner change some kinematic code in ocs2.
git clone -b upright https://github.com/utiasDSL/ocs2

# Clone pinocchio
git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
# Clone hpp-fcl
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
# Clone ocs2_robotic_assets
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
# Install dependencies
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev

# At last return your workspace which clone `upright`
catkin config --extend /xxx/depend_ws/install
```
