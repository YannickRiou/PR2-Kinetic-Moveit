//action server
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actioncontroller/ActionControllerAction.h>
#include "std_msgs/String.h"
//action client
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>

//Moveit 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/ApplyPlanningScene.h>

//MoveBase
#include <move_base_msgs/MoveBaseAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
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

//Grasp generator
#include "GraspGenerator.h"

namespace actioncontroller{

    class ActionController
    {
    private:



        std::map<std::string, moveit_msgs::CollisionObject> objects;

        std::mutex mutex;
        void openGripper(trajectory_msgs::JointTrajectory& posture){
            posture.joint_names.resize(6);
            posture.joint_names[0] = "r_gripper_joint";
            posture.joint_names[1] = "r_gripper_motor_screw_joint";
            posture.joint_names[2] = "r_gripper_l_finger_joint";
            posture.joint_names[3] = "r_gripper_r_finger_joint";
            posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
            posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

            posture.points.resize(1);
            posture.points[0].positions.resize(6);
            posture.points[0].positions[0] = 1;
            posture.points[0].positions[1] = 1;
            posture.points[0].positions[2] = 0.477;
            posture.points[0].positions[3] = 0.477;
            posture.points[0].positions[4] = 0.477;
            posture.points[0].positions[5] = 0.477;
            posture.points[0].time_from_start = ros::Duration(5);
        }

        void closedGripper(trajectory_msgs::JointTrajectory& posture){
            posture.joint_names.resize(6);
            posture.joint_names[0] = "r_gripper_joint";
            posture.joint_names[1] = "r_gripper_motor_screw_joint";
            posture.joint_names[2] = "r_gripper_l_finger_joint";
            posture.joint_names[3] = "r_gripper_r_finger_joint";
            posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
            posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

            posture.points.resize(1);
            posture.points[0].positions.resize(6);
            posture.points[0].positions[0] = 0.0;
            posture.points[0].positions[1] = 0.0;
            posture.points[0].positions[2] = 0.40;
            posture.points[0].positions[3] = 0.40;
            posture.points[0].positions[4] = 0.40;
            posture.points[0].positions[5] = 0.40;
            posture.points[0].time_from_start = ros::Duration(5);
        }

        bool move_arms(std::string group, geometry_msgs::Pose pose){
            moveit::planning_interface::MoveGroupInterface local_move_group(group);
            local_move_group.setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;

            bool success = (local_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if(success){
                local_move_group.move();
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

            if( objects.find(object) == objects.end() ){
                ROS_INFO(std::string("Object do not exist").c_str());
                return false;
            }

            return move_head( objects[ object ].mesh_poses[0] );
        }

        bool move_body(std::string group, geometry_msgs::Pose pose){

            actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> ac_("torso_controller/position_joint_action", true);
            while(!ac_.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the torso action server to come up");
            }
            //Get the pose of the object and convert it to a head msg
            pr2_controllers_msgs::SingleJointPositionGoal msg;
            //msg.header.stamp = ros::Time::now();
            msg.position = pose.position.z;
            msg.max_velocity = 1;
            msg.min_duration = ros::Duration(2.0);

            ac_.sendGoal(msg);

            bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

            if(success)
                ROS_INFO("Success");
            else
                ROS_INFO("FAIL");

            return success;
        }

        bool move_head(geometry_msgs::Pose pose){

            actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> ac_("head_traj_controller/point_head_action", true);
            while(!ac_.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the head_traj_controller action server to come up");
            }
            //Get the pose of the object and convert it to a head msg
            pr2_controllers_msgs::PointHeadGoal msg;
            //msg.header.stamp = ros::Time::now();
            msg.target.header.frame_id = "odom_combined";
            msg.target.point.x = pose.position.x;
            msg.target.point.y = pose.position.y;
            msg.target.point.z = pose.position.z;

            ac_.sendGoal(msg);

            bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

            if(success)
                ROS_INFO(std::string("Success").c_str());
            else
                ROS_INFO(std::string("FAIL").c_str());

            return success;
        }

        bool pick(std::string group, std::string object){

            if( objects.find(object) == objects.end() ){
                ROS_INFO(std::string("Object do not exist").c_str());
                return false;
            }
            move_head(object);
            //Create the planning scene

            std::vector<moveit_msgs::CollisionObject> vec;
            //ROS_INFO("Nombre d'objets = %i dans la fonction pick", objects.size());
            {
                std::lock_guard<std::mutex> lock(mutex);
                for(const auto& element : objects){
                    vec.push_back(element.second);
                    ROS_INFO( element.second.id.c_str() );
                }
            }
            current_scene.applyCollisionObjects(vec);
            std::stringstream ss;

            ss << "Object to pick pose is :\n x: " << objects[ object ].mesh_poses[0].position.x <<
               "\n y: " << objects[ object ].mesh_poses[0].position.y <<
               "\n z: " << objects[ object ].mesh_poses[0].position.z << std::endl;

            ROS_INFO(ss.str().c_str());

            std::stringstream info2;
            info2 << "Trying to pick " << object << " in " <<  std::string( objects[ object ].id ) << " reference frame";
            ROS_INFO(info2.str().c_str());


            geometry_msgs::PoseStamped p;
            p.header.frame_id = "/map";
            p.pose.position.x = objects[object].mesh_poses[0].position.x - 0.175   ;
            p.pose.position.y = objects[object].mesh_poses[0].position.y ;
            p.pose.position.z = objects[object].mesh_poses[0].position.z + 0.01 ;

            std::vector<moveit_msgs::Grasp> grasps;
            GraspGenerator gg("/home/dtrimoul/PR2-Kinetic-Xenial/src/actioncontroller/cfg/grasp.yaml", p );
            grasps = gg.generateGrasp();
            /*
            geometry_msgs::PoseStamped p;
            p.header.frame_id = "/map";
            p.pose.position.x = objects[object].mesh_poses[0].position.x - 0.175   ;
            p.pose.position.y = objects[object].mesh_poses[0].position.y ;
            p.pose.position.z = objects[object].mesh_poses[0].position.z + 0.01 ;
            p.pose.orientation.x = 0;
            p.pose.orientation.y = 0;
            p.pose.orientation.z = 0;
            p.pose.orientation.w = 1;
            moveit_msgs::Grasp g;
            g.grasp_pose = p;


            g.pre_grasp_approach.direction.header.frame_id = "base_footprint";
            g.pre_grasp_approach.direction.vector.z = -1.0;
            g.pre_grasp_approach.min_distance = 0.1;
            g.pre_grasp_approach.desired_distance = 0.20;

            g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
            g.post_grasp_retreat.direction.vector.z = 1.0;
            g.post_grasp_retreat.min_distance = 0.05;
            g.post_grasp_retreat.desired_distance = 0.20;

            openGripper(g.pre_grasp_posture);

            closedGripper(g.grasp_posture);
             grasps.push_back(g);
            */
            std::stringstream grasp_info;
            grasp_info << "number of grasps: " << grasps.size();
            ROS_INFO(grasp_info.str().c_str());
            move_group.allowReplanning(true);
            move_group.setSupportSurfaceName("tableLaas");
            moveit::planning_interface::MoveItErrorCode sucess = move_group.pick(object, grasps);
            if(moveit::planning_interface::MoveItErrorCode::SUCCESS == sucess.val){
                return true;
            }else{
                return false;
            }

        }

        bool place(std::string group, std::string object, geometry_msgs::Pose pose){
            move_head(pose);
            std::vector<moveit_msgs::PlaceLocation> place_location;
            place_location.resize(1);

            // Setting place location pose
            // +++++++++++++++++++++++++++
            place_location[0].place_pose.header.frame_id = "/map";

            /* While placing it is the exact location of the center of the object. */
            place_location[0].place_pose.pose.position.x = pose.position.x ;
            place_location[0].place_pose.pose.position.y = pose.position.y ;
            place_location[0].place_pose.pose.position.z = pose.position.z ;
            place_location[0].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

            // Setting pre-place approach
            // ++++++++++++++++++++++++++
            /* Defined with respect to frame_id */
            place_location[0].pre_place_approach.direction.header.frame_id = "base_footprint";
            /* Direction is set as negative z axis */
            place_location[0].pre_place_approach.direction.vector.z = -1.0;
            place_location[0].pre_place_approach.min_distance = 0.0001;
            place_location[0].pre_place_approach.desired_distance = 0.20;

            // Setting post-grasp retreat
            // ++++++++++++++++++++++++++
            /* Defined with respect to frame_id */
            place_location[0].post_place_retreat.direction.header.frame_id = "base_footprint";
            /* Direction is set as negative y axis */
            place_location[0].post_place_retreat.direction.vector.z = 1.0;
            place_location[0].post_place_retreat.min_distance = 0.0001;
            place_location[0].post_place_retreat.desired_distance = 0.30;

            // Setting posture of eef after placing object
            // +++++++++++++++++++++++++++++++++++++++++++
            /* Similar to the pick case */
            openGripper(place_location[0].post_place_posture);

            // Call place to place the object using the place locations given.
            moveit::planning_interface::MoveItErrorCode sucess = move_group.place(object, place_location);
            if(moveit::planning_interface::MoveItErrorCode::SUCCESS == sucess.val){
                return true;
            }else{
                return false;
            }

        }


    protected:
        //Action server
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<actioncontroller::ActionControllerAction> as_;
        actioncontroller::ActionControllerFeedback feedback_;
        actioncontroller::ActionControllerResult result_;
        std::string action_name_;
        ros::Subscriber sub_moveit_objects;
        ros::Publisher planning_scene_diff_publisher;

        //Moveit
        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface current_scene;

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
                as_(nh_, name, boost::bind(&ActionController::executeCB, this, _1), false),
                action_name_(name),
                move_group("right_arm")
        {
            sub_moveit_objects = nh_.subscribe("moveit_objects", 1000, &ActionController::objects_update, this );
            planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
            move_group.setPlannerId("TRRTkConfigDefault");
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
                feedback_.success = place( (std::string)msg->goal.action, (std::string)msg->goal.object, msg->goal.pose);
            }else if(msg->goal.action == "base" ){
                feedback_.success = move_base( msg->goal.pose );
            }else if(msg->goal.action == "torso" ){
                feedback_.success = move_body( (std::string)msg->goal.action, msg->goal.pose );
            }else if(msg->goal.action == "left_arm" || msg->goal.action == "right_arm" ){
                feedback_.success = move_arms((std::string)msg->goal.action, msg->goal.pose);
            }else{
                ROS_INFO(std::string("The required order do not exist").c_str());
            }
            result_.success = feedback_.success;
            as_.setSucceeded(result_);
        }
    };

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "action_controller");
    actioncontroller::ActionController _myController("action_controller");
    ros::spin();

    return 0;
}
