cmake_minimum_required(VERSION 2.8.3)
project(actioncontroller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
	actionlib_msgs 
	rospy 
	roscpp
	moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
	geometric_shapes
	move_base_msgs
  message_generation
  std_msgs
  ar_track_alvar_msgs
  shape_tools
		gazebo_moveit_objects_synchroniser
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   actiongoal.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES ActionController.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
	 geometry_msgs
   ar_track_alvar_msgs
   actioncontroller
   # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
		include
  CATKIN_DEPENDS
	move_base_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    actionlib_msgs
    ar_track_alvar_msgs
    gazebo_moveit_objects_synchroniser
  	DEPENDS
    EIGEN3
)

###########
## Build ##
###########


include_directories(
        SYSTEM
		include
        ${THIS_PACKAGE_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${CATKIN_DEVEL_PREFIX}/include)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(action_controller src/ActionController.cpp)
add_dependencies(action_controller actioncontroller_generate_messages_cpp)
target_link_libraries(action_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS action_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(setup src/SetUp.cpp src/ActionControllerTools.cpp)
target_link_libraries(setup ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS setup DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(make_a_pile src/MakeAPile.cpp src/ActionControllerTools.cpp)
target_link_libraries(make_a_pile ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS make_a_pile DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
 
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_actioncontroller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
