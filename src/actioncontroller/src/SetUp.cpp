#include <ros/ros.h>
//action client
#include <actionlib/client/simple_action_client.h>
#include <actioncontroller/ActionControllerAction.h>
#include <ActionControllerTools.h>




int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_setup_position");
	std::string part;
	actioncontroller::ActionControllerGoal goal;
	actioncontroller::ActionControllerTools tools;

	part = "torso";
    tools.createActionControllerMessage(part, 0, 0, 0.3, 0, 0, 0, 0, goal);
    tools.callingActionController(part, goal);

	part = "left_arm";
    tools.createActionControllerMessage(part, -0.1, 0.7, 1, 1.0, 0, 0, 0, goal);
    tools.callingActionController(part, goal);


	part = "right_arm";
    tools.createActionControllerMessage(part, -0.1, -0.7, 1, 1.0, 0, 0, 0, goal);
    tools.callingActionController(part, goal);

	part = "base";
    tools.createActionControllerMessage(part, 1, 0, 0, 1.0, 0, 0, 0, goal);
    tools.callingActionController(part, goal);

	part = "stareAtObject";
    tools.createActionControllerMessage(part, "tableLaas" , goal);
    tools.callingActionController(part, goal);


	return 0;
}