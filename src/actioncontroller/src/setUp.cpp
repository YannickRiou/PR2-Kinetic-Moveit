#include <ros/ros.h>
//action client
#include <actionlib/client/simple_action_client.h>
#include <actioncontroller/ActionControllerAction.h>




void callingActionController(std::string part, actioncontroller::ActionControllerGoal &goal){
	actionlib::SimpleActionClient<actioncontroller::ActionControllerAction> ac_("ActionController", true);
	while(!ac_.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the action server to come up");
		}

	ROS_INFO("Sending goal to ActionController");
	ac_.sendGoal(goal);
	ROS_INFO("wating for result");

	ac_.waitForResult();
	
	bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
	if(success)
		ROS_INFO("Move %s success", part.c_str());
	else
		ROS_INFO("Failed to move %s", part.c_str());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_setup_position");
	std::string part;
	actioncontroller::ActionControllerGoal goal;
	

	return 0;
}