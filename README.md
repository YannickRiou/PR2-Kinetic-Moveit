
# Project Setup
## ROS INSTALL
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'	
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
```
## Repo Clonning
```sh
echo "Creating catkin Workspace for PR2 and MoveIt packages in your home"
cd ~
git clone --recurse-submodules https://github.com/DavidTrimoulet/PR2-Kinetic-Xenial.git
cd PR2-Kinetic-Xenial
```
## Dependencies
```sh
echo "Installing dependencies"
sudo apt -y install libgtk2.0-dev libfcl0.5 libfcl-0.5-dev scons libbson-dev libglew-dev libglew1.13 ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server ros-kinetic-geometric-shapes ros-kinetic-warehouse-ros ros-kinetic-srdfdom ros-kinetic-object-recognition-msgs ros-kinetic-navigation ros-kinetic-pr2-common ros-kinetic-image-geometry ros-kinetic-cv-bridge ros-kinetic-perceptions-pctl ros-kinetic-gazebo-plugins ros-kinetic-ompl ros-kinetic-pr2-controllers-msgs ros-kinetic-rgbd-launch ros-kinetic-freenect-launch ros-kinetic-robot-model ros-kinetic-rqt-gui ros-kinetic-cmake-modules ros-kinetic-control-msgs ros-kinetic-geometry  ros-kinetic-tf2-geometry-msgs ros-kinetic-roslint ros-kinetic-filters flex ros-kinetic-mongodb-store ros-kinetic-tf2-bullet freeglut3-dev python-catkin-tools ros-kinetic-laser-assembler ros-kinetic-diagnostic-msgs ros-kinetic-pr2-controllers-msgs libsdformat4-dev ros-kinetic-gazebo-ros ros-kinetic-pose-cov-ops ros-kinetic-jsk-recognition-msgs ros-kinetic-pr2-gazebo ros-kinetic-pr2-moveit-config ros-kinetic-jsk-pcl-ros ros-kinetic-moveit ros-kinetic-moveit-pr2 ros-kinetic-pr2-moveit-config ros-kinetic-pr2-moveit-plugins ros-kinetic-pr2-navigation ros-kinetic-navigation ros-kinetic-tf-tools ros-kinetic-pose-cov-ops ros-kinetic-jsk-recognition-msgs ros-kinetic-ar-track-alvar-msgs ros-kinetic-pr2-gazebo ros-kinetic-pr2-gazebo-plugins ros-kinetic-jsk-pcl-ros ros-kinetic-jsk-pcl-ros-utils
```

## Building
```sh
catkin build
source ~/PR2-Kinetic-Xenial/devel/setup.bash
echo "source ~/PR2-Kinetic-Xenial/devel/setup.bash" >> $HOME/.bashrc
echo "source /opt/ros/kinetic/setup.bash" >> $HOME/.bashrc
echo "export KINECT2="true"" >> $HOME/.bashrc
echo "export ROBOT_INITIAL_POSE=\"-Y 1.55 -x 4.5 -y 5\"" >> $HOME/.bashrc
```
## Copying gazebo models to .gazebo folder
```sh
mkdir ~/.gazebo/models
cp ~/PR2-Kinetic-Xenial/src/pickplacedemo/worlds/models/* ~/.gazebo/models/
```
