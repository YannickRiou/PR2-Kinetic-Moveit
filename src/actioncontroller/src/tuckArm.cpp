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
		bool success_r;
		bool success_l;
		bool success_h;
		feedback_.sequence.clear();

		ROS_INFO("Starting to compute path to arm tucked");

		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

		//Plannification avec MoveIt
		static const std::string LEFT_ARM = "left_arm";
		moveit::planning_interface::MoveGroupInterface move_group_left(LEFT_ARM);
		const robot_state::JointModelGroup* joint_model_group_left = move_group_left.getCurrentState()->getJointModelGroup(LEFT_ARM);

		static const std::string RIGHT_ARM = "right_arm";
		moveit::planning_interface::MoveGroupInterface move_group_right(RIGHT_ARM);
		const robot_state::JointModelGroup* joint_model_group_right = move_group_right.getCurrentState()->getJointModelGroup(RIGHT_ARM);

		static const std::string HEAD = "head";
		moveit::planning_interface::MoveGroupInterface move_group_head(HEAD);
		const robot_state::JointModelGroup* joint_model_group_head = move_group_head.getCurrentState()->getJointModelGroup(HEAD);

		ROS_INFO_NAMED("tuckArm", "Reference frame: %s for left arm", move_group_left.getPlanningFrame().c_str());
		ROS_INFO_NAMED("tuckArm", "Reference frame: %s for right arm", move_group_right.getPlanningFrame().c_str());
		ROS_INFO_NAMED("tuckArm", "Reference frame: %s for right arm", move_group_head.getPlanningFrame().c_str());
		geometry_msgs::Pose target_left;
		target_left.orientation.w = 1.0;
  		target_left.position.x = 0.28;
  		target_left.position.y = 0.2;
  		target_left.position.z = 0.5;
		move_group_left.setPoseTarget(target_left);

		//Execution du plan
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		success_l = (move_group_left.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if(success_l){
			move_group_left.move();
		}else{
			ROS_INFO_NAMED("TuckArm", "No path found for left arm");
		}

		geometry_msgs::Pose target_right;
		target_right.orientation.w = 1.0;
  		target_right.position.x = 0.28;
  		target_right.position.y = -0.2;
  		target_right.position.z = 0.5;
		move_group_right.setPoseTarget(target_right);
		
		my_plan = moveit::planning_interface::MoveGroupInterface::Plan();
		success_r = (move_group_right.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		if(success_r){
			move_group_right.move();
		}else{
			ROS_INFO_NAMED("TuckArm", "No path found for right_arm");
		}
		

		my_plan = moveit::planning_interface::MoveGroupInterface::Plan();
		geometry_msgs::Pose target_head;
		//toQuaternion(&target_head, 10, 20, 30 );
		target_head.position.x = move_group_head.getCurrentPose().pose.position.x;
  		target_head.position.y = move_group_head.getCurrentPose().pose.position.y;
  		target_head.position.z = move_group_head.getCurrentPose().pose.position.z;
  		//target_head.orientation.x = -0.05;
  		//target_head.orientation.y = 0.52821;
  		//target_head.orientation.z = 0.0771891;
  		target_head.orientation.w = 1;

		std::stringstream ss;

		ss << " x:" << move_group_head.getCurrentPose().pose.position.x << " y:" << move_group_head.getCurrentPose().pose.position.y << " z:" << move_group_head.getCurrentPose().pose.position.z
		<< " ox:" << move_group_head.getCurrentPose().pose.orientation.x << " oy:" << move_group_head.getCurrentPose().pose.orientation.y << " oz:" << move_group_head.getCurrentPose().pose.orientation.z << " ow:" << move_group_head.getCurrentPose().pose.orientation.w;
		ROS_INFO_NAMED("TuckArm, head", "End effector link: %s", ss.str().c_str());
		move_group_head.setPoseTarget(target_head);
		my_plan = moveit::planning_interface::MoveGroupInterface::Plan();
		success_h = (move_group_head.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		if(success_h){
			move_group_head.move();
		}else{
			ROS_INFO_NAMED("TuckArm", "No path found for head");
		}
		

		if(success_h && success_r && success_l)
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