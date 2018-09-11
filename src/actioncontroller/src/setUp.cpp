#include <ros/ros.h>
//action client
#include <actionlib/client/simple_action_client.h>
#include <actioncontroller/ActionControllerAction.h>




void createActionControllerMessage(std::string group_id, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &msg){
	msg.goal.action = group_id;
	msg.goal.pose.position.x = x;
	msg.goal.pose.position.y = y;
	msg.goal.pose.position.z = z;
	msg.goal.pose.orientation.x = ox;
	msg.goal.pose.orientation.y = oy;
	msg.goal.pose.orientation.z = oz;
	msg.goal.pose.orientation.w = w;
}

void callingActionController(std::string part, actioncontroller::ActionControllerGoal &goal){
	actionlib::SimpleActionClient<actioncontroller::ActionControllerAction> ac_("ActionController", true);
	while(!ac_.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
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

	part = "torso";
	createActionControllerMessage(part, 0, 0, 0.3, 0, 0, 0, 0, goal);
	callingActionController(part, goal);

	part = "left_arm";
	createActionControllerMessage(part, -0.1, 0.7, 1, 1.0, 0, 0, 0, goal);
	callingActionController(part, goal);


	part = "right_arm";
	createActionControllerMessage(part, -0.1, -0.7, 1, 1.0, 0, 0, 0, goal);
	callingActionController(part, goal);



	part = "base";
	createActionControllerMessage(part, 1, 0, 0, 1.0, 0, 0, 0, goal);
	callingActionController(part, goal);

	part = "stareAt";
	createActionControllerMessage(part, 1, 0, 1.1, 0, 0, 0, 0, goal);
	callingActionController(part, goal);


	return 0;
}