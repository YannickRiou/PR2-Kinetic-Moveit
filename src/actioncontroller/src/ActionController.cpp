//action server
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actioncontroller/ActionControllerAction.h>

//action client
#include <actionlib/client/simple_action_client.h>

//Moveit 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//MoveBase
#include <move_base_msgs/MoveBaseAction.h>


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

	bool move_body(std::string group, geometry_msgs::Pose pose){
		moveit::planning_interface::MoveGroupInterface move_group(group);
		move_group.setPoseTarget(pose);
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		if(success){
			move_group.move();
		}else{
			ROS_INFO_NAMED("ActionController", "No path found for %s", group.c_str() );
		}
		return success;
	}

	bool move_base(std::string group, geometry_msgs::Pose pose){

		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_("move_base", true);
		while(!ac_.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "odom_combined";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = pose.position.x;
		goal.target_pose.pose.position.y = pose.position.y;
		goal.target_pose.pose.position.z = pose.position.z;
		goal.target_pose.pose.orientation.w = pose.orientation.w;
		ROS_INFO("Sending goal to move base");
		ac_.sendGoal(goal);
		ROS_INFO("wating for result");
		ac_.waitForResult();
		bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

		if(success)
			ROS_INFO("Hooray, the base moved to x: %s, y:%s, z:%s", 
				(std::to_string(pose.position.x)).c_str(), 
				(std::to_string(pose.position.y)).c_str(), 
				(std::to_string(pose.position.z)).c_str()  );
		else
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		
		return success;
	}

protected:
	//Action server
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<actioncontroller::ActionControllerAction> as_;
	actioncontroller::ActionControllerFeedback feedback_;
	actioncontroller::ActionControllerResult result_;
	std::string action_name_;

	//Move base client


public:
	ActionController(std::string name) :
	as_(nh_, name, boost::bind(&ActionController::executeCB, this, _1), false), action_name_(name){
		as_.start();
	}


	~ActionController(void){
	}

	void executeCB(const actioncontroller::ActionControllerGoalConstPtr &goal){
		ros::Rate r(1);
		if(goal->goal.move_group_id == "base"){
			feedback_.success = move_base((std::string)goal->goal.move_group_id, (geometry_msgs::Pose)goal->goal.pose.pose);
		}else{
			feedback_.success = move_body((std::string)goal->goal.move_group_id, (geometry_msgs::Pose)goal->goal.pose.pose);	
		}
		result_.success = feedback_.success;
		as_.setSucceeded(result_);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ActionController"); 
	ActionController _myController("ActionController");

	ros::spin(); 
	return 0;
}