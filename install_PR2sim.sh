#!/bin/bassh

echo "Creating catkin Workspace for PR2 and MoveIt packages in your home"
cd ~
git clone --recurse-submodules https://github.com/DavidTrimoulet/PR2-Kinetic-Xenial.git
cd PR2-Kinetic-Xenial
rosdep install --from-paths src/vision_visp/ --rosdistro kinetic
rosdep install --from-paths src/ar_track_alvar/ --rosdistro kinetic

echo "Installing dependencies"
sudo apt -y install libgtk2.0-dev libfcl0.5 libfcl-0.5-dev scons libbson-dev libglew-dev libglew1.13 ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server ros-kinetic-geometric-shapes ros-kinetic-warehouse-ros ros-kinetic-srdfdom ros-kinetic-object-recognition-msgs ros-kinetic-navigation ros-kinetic-ivcon ros-kinetic-convex-decomposition python-pip

sudo apt -y install ros-kinetic-pr2-common
sudo apt -y install ros-kinetic-image-geometry
sudo apt -y install ros-kinetic-cv-bridge
sudo apt -y install ros-kinetic-perceptions-pctl
sudo apt -y install ros-kinetic-gazebo-plugins
sudo apt -y install ros-kinetic-ompl
sudo apt -y install ros-kinetic-pr2-controllers-msgs
sudo apt -y install ros-kinetic-rgbd-launch
sudo apt -y install ros-kinetic-freenect-launch
sudo apt -y install ros-kinetic-robot-model
sudo apt -y install ros-kinetic-rqt-gui
sudo apt -y install ros-kinetic-cmake-modules
sudo apt -y install ros-kinetic-control-msgs
sudo apt -y install ros-kinetic-geometry 
sudo apt -y install ros-kinetic-tf2-geometry-msgs
sudo apt -y install ros-kinetic-roslint
sudo apt -y install ros-kinetic-filters

#gestion des contrler pour teleop
sudo apt -y install libusb-dev
sudo apt -y install libspnav-dev
sudo apt -y install libbluetooth-dev
sudo apt -y install libcwiid-dev

catkin_make
source ~/PR2-Kinetic-Xenial/devel/setup.bash

#intallation du package de vision



#mkdir -p ~/moveItTuto/src
#cd ~/moveItTuto/src
#git clone https://github.com/ros-planning/moveit_tutorials.git
#sudo pip install pyassimp
#cd ~/moveItTuto
#catkin_make
#source ~/moveItTuto/devel/setup.bash