//action server
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actioncontroller/ActionControllerAction.h>

//Moveit 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>



class ActionController
{
private:
	void toQuaternion(geometry_msgs::Pose *p ,double pitch, double roll, double yaw){
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	p->orientation.w = cy * cr * cp + sy * sr * sp;
	p->orientation.x = cy * sr * cp - sy * cr * sp;
	p->orientation.y = cy * cr * sp + sy * sr * cp;
	p->orientation.z = sy * cr * cp - cy * sr * sp;
	}
	enum move_groups { LEFT_ARM, RIGHT_ARM, HEAD, BASE };
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<actioncontroller::ActionController> as_;
	std::string action_name_;

	actioncontroller::ActionControllerFeedback feedback_;
	actioncontroller::ActionControllerResult result_;

public:
	ActionController(std::string name) :
	as_(nh_, name, boost::bind(&ActionController::executeCB, this, _1), false), action_name_(name){
		as_.start();
	}

	~ActionController(void){
	}

	void executeCB(const actioncontroller::actioncontrollerGoalConstPtr &goal, const string){
		ros::Rate r(1);
		



	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ActionController"); 
	ActionController _myController("ActionController");

	ros::spin(); 
	return 0;
}