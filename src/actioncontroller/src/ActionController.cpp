//action server
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actioncontroller/ActionControllerAction.h>
#include "std_msgs/String.h"
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
#include <gazebo_moveit_objects_synchroniser/CollisionObjectArray.h>

//Shared Memory map
#include <mutex>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <functional>
#include <utility>

class ActionController
{
private:



    std::map<std::string, moveit_msgs::CollisionObject> objects;

    std::mutex mutex;

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

	bool move_head(std::string object){

		actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> ac_("head_traj_controller/point_head_action", true);
		while(!ac_.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		//Get the pose of the object and convert it to a head msg
		pr2_controllers_msgs::PointHeadGoal msg;
		//msg.header.stamp = ros::Time::now();
		msg.target.header.frame_id = "odom_combined";
		msg.target.point.x = 0;
		msg.target.point.y = 0;
		msg.target.point.z = 0;
		ac_.sendGoal(msg);

		bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

		if(success)
            ROS_INFO("Success");
		else
            ROS_INFO("FAIL");
		
		return success;
	}

    bool move_head(geometry_msgs::Pose pose){
	    return true;
	}

	bool pick(std::string group, std::string object){
		std::vector<std::string> tokens;
		boost::split(tokens, group, [](char c){return c == '.';});
		group =  tokens.at(1);
		ROS_INFO(group.c_str());
		moveit::planning_interface::MoveGroupInterface move_group(group);
        moveit::planning_interface::PlanningSceneInterface current_scene;
        std::vector<moveit_msgs::CollisionObject> vec;
        //ROS_INFO("Nombre d'objets = %i dans la fonction pick", objects.size());
        {
            std::lock_guard<std::mutex> lock(mutex);
            for(const auto& element : objects){
                vec.push_back(element.second);
                ROS_INFO( element.second.id.c_str() );
            }
        }
		//std::vector<manipulation_msgs::Grasp> grasps;
        current_scene.applyCollisionObjects(vec);
        std::stringstream info2;
        info2 << "Trying to pick " << object << " in " <<  std::string( objects[ object ].id ) << " reference frame";
        ROS_INFO(info2.str().c_str());
		move_group.pick(object);
	}

	bool place(std::string group, std::string object){

	}


protected:
	//Action server
    ros::NodeHandle nh_;
	actionlib::SimpleActionServer<actioncontroller::ActionControllerAction> as_;
	actioncontroller::ActionControllerFeedback feedback_;
	actioncontroller::ActionControllerResult result_;
	std::string action_name_;
	ros::Subscriber sub;

	//Move base client

    void objects_update(const gazebo_moveit_objects_synchroniser::CollisionObjectArray::ConstPtr &msg) {
        //ROS_INFO("message received");
        for(int i=0; i<msg->data.size(); i++){
            std::lock_guard<std::mutex> lock(mutex);
            if(!objects.insert( std::pair<std::string, moveit_msgs::CollisionObject>( (std::string)msg->data[i].id , msg->data[i])).second){
                //std::string info = "object " + msg->data[i].id +  " frame id is " + msg->data[i].header.frame_id;
                //ROS_INFO(info.c_str());
                objects[ msg->data[i].id ] = msg->data[i];
            }
        }
        //ROS_INFO("Nombre d'objets = %i", objects.size());
    }


public:

    ActionController(std::string name) :
	as_(nh_, name, boost::bind(&ActionController::executeCB, this, _1), false), action_name_(name){
        sub = nh_.subscribe("moveit_objects", 1000, &ActionController::objects_update, this );
		as_.start();
	}


	~ActionController(void){
	}

	void executeCB(const actioncontroller::ActionControllerGoalConstPtr &msg){

        //ROS_INFO("Nombre d'objets = %i dans l'action callback", objects.size());

		if(msg->goal.action == "stareAt"){
			feedback_.success = move_head( msg->goal.pose );
		}else if(msg->goal.action == "stareAtObject"){
            feedback_.success = move_head( (std::string)msg->goal.object );
		}else if(msg->goal.action == "pick.left_arm" || msg->goal.action == "pick.right_arm" ){
			feedback_.success = pick( (std::string)msg->goal.action, (std::string)msg->goal.object);
		}else if(msg->goal.action == "place.left_arm" || msg->goal.action == "place.right_arm" ){
			feedback_.success = place( (std::string)msg->goal.action, (std::string)msg->goal.object);
		}else if(msg->goal.action == "base" ){
            feedback_.success = move_base( msg->goal.pose );
        }else{
			feedback_.success = move_arms((std::string)msg->goal.action, msg->goal.pose);
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