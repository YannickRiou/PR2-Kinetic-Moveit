#!/bin/bash

echo "Creating catkin Workspace for PR2 and MoveIt packages in your home"
cd ~
mkdir ws_pr2
cd ws_pr2
git clone --branch master --recurse-submodules https://github.com/DavidTrimoulet/PR2-Kinetic-Xenial.git .
mkdir -p src/vision_visp/
mkdir -p src/ar_track_alvar/
rosdep install --from-paths src/vision_visp/ --rosdistro kinetic
rosdep install --from-paths src/ar_track_alvar/ --rosdistro kinetic

echo "Installing dependencies"
sudo apt -y install libgtk2.0-dev libfcl0.5 libfcl-0.5-dev scons libbson-dev libglew-dev libglew1.13 ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server ros-kinetic-geometric-shapes ros-kinetic-warehouse-ros ros-kinetic-srdfdom ros-kinetic-object-recognition-msgs ros-kinetic-navigation ros-kinetic-ivcon ros-kinetic-convex-decomposition python-pip

sudo apt -y install ros-kinetic-pr2-common
sudo apt -y install ros-kinetic-pr2-controllers
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

sudo apt -y install flex ros-kinetic-mongodb-store ros-kinetic-tf2-bullet freeglut3-dev python-catkin-tools


#gestion des contrler pour teleop
sudo apt -y install libusb-dev
sudo apt -y install libspnav-dev
sudo apt -y install libbluetooth-dev
sudo apt -y install libcwiid-dev

catkin_make
source ~/ws_pr2/devel/setup.bash
echo "source ~/ws_pr2/devel/setup.bash" >> ~/.bashrc 
