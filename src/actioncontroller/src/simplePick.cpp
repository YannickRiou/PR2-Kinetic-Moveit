#include <ros/ros.h>
//action client
#include <actionlib/client/simple_action_client.h>
#include <actioncontroller/ActionControllerAction.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

int markerId = -1;

void createActionControllerMessage(std::string group_id, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &goal){	
	goal.goal.move_group_id = group_id;
	goal.goal.pose.pose.position.x = x;
	goal.goal.pose.pose.position.y = y;
	goal.goal.pose.pose.position.z = z;
	goal.goal.pose.pose.orientation.x = ox;
	goal.goal.pose.pose.orientation.y = oy;
	goal.goal.pose.pose.orientation.z = oz;
	goal.goal.pose.pose.orientation.w = w;
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

void markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
	ROS_INFO("Object detected : %s", (std::to_string( msg->markers[0].id) ).c_str() );

	int id = msg->markers[0].id;
	if(id != markerId){
		ROS_INFO("trying to pick : %s", (std::to_string( msg->markers[0].id) ).c_str() );
		markerId = msg->markers[0].id;
		std::string part;
		actioncontroller::ActionControllerGoal goal;
		part = "pick.left_arm";
		goal.goal.move_group_id = part;
		goal.goal.pose.pose = msg->markers[0].pose.pose;
		callingActionController(part, goal);

	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_setup_position");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/ar_pose_marker", 1000, markerCallback);
	ros::spin();
		
	return 0;
}