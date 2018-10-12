//
// Created by dtrimoul on 9/13/18.
//

#include <ros/ros.h>
//action client
#include <actionlib/client/simple_action_client.h>
#include <actioncontroller/ActionControllerAction.h>
#include "ActionControllerTools.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_setup_position");
	std::string action;
	actioncontroller::ActionControllerGoal msg;
	actioncontroller::ActionControllerTools tools;

    std::vector<std::string> objects = {"cube0", "cube0_0", "cube0_1"};

    for(const auto& object : objects){
        action = "pick.right_arm";
        tools.createActionControllerMessage(action, object , msg);
        tools.callingActionController(action, msg);

        action = "place.right_arm";
        tools.createActionControllerMessage(action, object, 1.48, 0.0, 0.90, 1.0, 0, 0, 0, msg);
        tools.callingActionController(action, msg);
    }



	return 0;
}