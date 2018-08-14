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
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <shape_tools/solid_primitive_dims.h>

#include <boost/algorithm/string.hpp>


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

	bool move_arms(std::string group, geometry_msgs::Pose pose){
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

	bool move_base(geometry_msgs::Pose pose){

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
			ROS_INFO("The base failed to move for some reason");
		
		return success;
	}

	bool move_head(geometry_msgs::Pose pose){

		actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> ac_("head_traj_controller/point_head_action", true);
		while(!ac_.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		ROS_INFO("moving the head to %s, %s, %s",  
				(std::to_string(pose.position.x)).c_str(), 
				(std::to_string(pose.position.y)).c_str(), 
				(std::to_string(pose.position.z)).c_str()  );

		pr2_controllers_msgs::PointHeadGoal msg;
		//msg.header.stamp = ros::Time::now();
		msg.target.header.frame_id = "odom_combined";
		msg.target.point.x = pose.position.x;
		msg.target.point.y = pose.position.y;
		msg.target.point.z = pose.position.z;
		ac_.sendGoal(msg);

		bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

		if(success)
			ROS_INFO("Hooray, the base moved to x: %s, y:%s, z:%s", 
				(std::to_string(pose.position.x)).c_str(), 
				(std::to_string(pose.position.y)).c_str(), 
				(std::to_string(pose.position.z)).c_str()  );
		else
			ROS_INFO("The base failed to move for some reason");
		
		return success;
	}

	bool pick(std::string group, geometry_msgs::Pose pose){
		std::vector<std::string> tokens;
		boost::split(tokens, group, [](char c){return c == '.';});
		group =  tokens.at(1);
		ROS_INFO(group.c_str());
		moveit::planning_interface::MoveGroupInterface move_group(group);
		//std::vector<manipulation_msgs::Grasp> grasps;
		move_group.pick("cube");
	}

	bool place(std::string group, geometry_msgs::Pose pose){

	}

	void createObject(geometry_msgs::Pose pose){
		ros::NodeHandle nh;
		ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
		moveit_msgs::CollisionObject co;
		co.header.frame_id = "odom_combined";
		co.header.stamp = ros::Time::now();
		//remove the object
		co.id = "cube";
		co.operation = moveit_msgs::CollisionObject::REMOVE;
		pub_co.publish(co);
		//Add the object
		co.operation = moveit_msgs::CollisionObject::ADD;
		co.primitives.resize(1);
		co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.08;
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.08;
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.08;
		co.primitive_poses.resize(1);
		co.primitive_poses[0].position.x = pose.position.x;
		co.primitive_poses[0].position.y = pose.position.y;
		co.primitive_poses[0].position.z = pose.position.z;
		co.primitive_poses[0].orientation.w = pose.orientation.w;
		pub_co.publish(co);

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
			feedback_.success = move_base( (geometry_msgs::Pose)goal->goal.pose.pose );
		}else if(goal->goal.move_group_id == "head"){
			feedback_.success = move_head( (geometry_msgs::Pose)goal->goal.pose.pose);
		}else if(goal->goal.move_group_id == "pick.left_arm" || goal->goal.move_group_id == "pick.right_arm" ){
			feedback_.success = pick( (std::string)goal->goal.move_group_id, (geometry_msgs::Pose)goal->goal.pose.pose);
		}else if(goal->goal.move_group_id == "place.left_arm" || goal->goal.move_group_id == "place.right_arm" ){
			feedback_.success = place( (std::string)goal->goal.move_group_id, (geometry_msgs::Pose)goal->goal.pose.pose);
		}else{
			feedback_.success = move_arms((std::string)goal->goal.move_group_id, (geometry_msgs::Pose)goal->goal.pose.pose);	
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