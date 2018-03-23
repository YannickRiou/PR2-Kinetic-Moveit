#!/bin/bassh

echo "Installing dependancies"
sudo apt-get install libfcl0.5 libfcl-0.5-dev scons libbson-dev libglew-dev libglew1.13 ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server ros-kinetic-geometric-shapes ros-kinetic-warehouse-ros ros-kinetic-srdfdom ros-kinetic-object-recognition-msgs ros-kinetic-navigation ros-kinetic-ivcon ros-kinetic-convex-decomposition pip

sudo apt-get install ros-kinetic-pr2-common
apt-get install ros-kinetic-ompl

echo "Creating catkin Workspace for PR2 and MoveIt packages"
sudo mkdir -p ~/PR2/src
cd ~/PR2
git clone https://github.com/DavidTrimoulet/PR2-Kinetic-Xenial.git
catkin_make
source ~/PR2/devel/setup.bash

echo "Creating catkin Workspace for MoveIt tutorial"
mkdir -p ~/moveItTuto/src
cd ~/moveItTuto/src
git clone https://github.com/ros-planning/moveit_tutorials.git
sudo pip install pyassimp
cd ~/moveItTuto
catkin_make
source ~/mmoveItTuto/devel/setup.bash