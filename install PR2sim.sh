#!/bin/bassh

echo "Installing dependancies"
sudo apt-get install libfcl0.5 libfcl-0.5-dev scons libbson-dev libglew-dev libglew1.13 ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server ros-kinetic-geometric-shapes ros-kinetic-warehouse-ros ros-kinetic-srdfdom ros-kinetic-object-recognition-msgs ros-kinetic-navigation ros-kinetic-ivcon ros-kinetic-convex-decomposition python-pip

sudo apt-get install ros-kinetic-pr2-common
apt-get install ros-kinetic-ompl

echo "Creating catkin Workspace for PR2 and MoveIt packages"
cd ~
git clone --recurse-submodules https://github.com/DavidTrimoulet/PR2-Kinetic-Xenial.git
cd PR2-Kinetic-Xenial
catkin_make
source ~/PR2-Kinetic-Xenial/devel/setup.bash

echo "Creating catkin Workspace for MoveIt tutorial"
mkdir -p ~/moveItTuto/src
cd ~/moveItTuto/src
git clone https://github.com/ros-planning/moveit_tutorials.git
sudo pip install pyassimp
cd ~/moveItTuto
catkin_make
source ~/moveItTuto/devel/setup.bash