sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade

echo "Creating catkin Workspace for PR2 and MoveIt packages in your home"
cd ~
git clone --recurse-submodules https://github.com/DavidTrimoulet/PR2-Kinetic-Xenial.git
cd PR2-Kinetic-Xenial

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
sudo apt -y install flex ros-kinetic-mongodb-store ros-kinetic-tf2-bullet freeglut3-dev python-catkin-tools
sudo apt -y install ros-kinetic-laser-assembler
sudo apt -y install ros-kinetic-diagnostic-msgs
sudo apt -y install ros-kinetic-pr2-controllers-msgs
sudo apt -y install libsdformat4-dev
sudo apt -y install ros-kinetic-gazebo-ros
sudo apt -y install ros-kinetic-pose-cov-ops
sudo apt -y install ros-kinetic-jsk-recognition-msgs


#gestion des contrler pour teleop



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

echo "KINECT2=true" >> $HOME/.bashrc
echo "ROBOT_INITIAL_POSE="-Y 1.55 -x 4.5 -y 5"" >> $HOME/.bashrc
