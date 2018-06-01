//action server
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actioncontroller/TuckArmAction.h>

//Moveit 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>



class TuckArmAction
{
private:
	void initMoveIt(){
		static const std::string PLANNING_GROUP = "panda_arm";
	}
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<actioncontroller::TuckArmAction> as_;
	std::string action_name_;

	actioncontroller::TuckArmFeedback feedback_;
	actioncontroller::TuckArmResult result_;

public:
	TuckArmAction(std::string name) :
	as_(nh_, name, boost::bind(&TuckArmAction::executeCB, this, _1), false), action_name_(name){
		as_.start();
	}

	~TuckArmAction(void){
	}

	void executeCB(const actioncontroller::TuckArmGoalConstPtr &goal){
		ros::Rate r(1);
		bool success = true;

		feedback_.sequence.clear();

		ROS_INFO("Starting to compute path to arm tucked");

		//Plannification avec MoveIt

		//Execution du plan

		if(success)
	    {
		    result_.sequence = feedback_.sequence;
	        ROS_INFO("%s: Succeeded", action_name_.c_str());
	        // set the action state to succeeded
			as_.setSucceeded(result_);
		}

	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "TuckArm"); 
	TuckArmAction tuckArm("TuckArm");
	ros::spin(); 
	return 0;
}