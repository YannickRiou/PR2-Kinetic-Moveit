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
#include <PlaceGenerator.h>
#include "GraspGenerator.h"

//Grasp generator
#include "PickGenerator.h"

#define GRASP_FILE "/home/dtrimoul/PR2-Kinetic-Xenial/src/actioncontroller/cfg/grasp.yaml"

namespace actioncontroller{

    class ActionController
    {
    private:

        std::map<std::string, moveit_msgs::CollisionObject> objects;
        actioncontroller::ActionControllerTools tools;
        std::mutex mutex;

        bool setup(){
            geometry_msgs::Pose right_arm;
            geometry_msgs::Pose left_arm;
            geometry_msgs::Pose torso;
            tools.posePopulator(right_arm, -0.1, -0.7, 1, 0, 0, 0, 1.0 );
            tools.posePopulator(left_arm,  -0.1, 0.7, 1, 0, 0, 0, 1.0 );
            tools.posePopulator(torso, 0, 0, 0.2, 0, 0, 0, 1.0 );
            move_arms("right_arm", right_arm );
            move_arms("left_arm", left_arm );
            move_body("torso", torso );
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
            ac_.waitForResult();

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

        bool goto_location(std::vector<std::string> command){

            //a remplacer par une liste de position possible à proximité

            geometry_msgs::Pose temporaryPose = objects[command[3]].mesh_poses[0];
            temporaryPose.position.x -= 0.50;
            ROS_INFO("calling move base");
            move_base(temporaryPose);
        }

        bool move_head(std::vector<std::string> command){
            std::string object = command[1];

            if( objects.find(object) == objects.end() ){
                ROS_INFO(std::string("Object do not exist").c_str());
                return false;
            }

            return move_head( objects[ object ].mesh_poses[0] );
        }

        bool pick(std::vector<std::string> command){
            std::string object = command[3];

            if( objects.find(object) == objects.end() ){
                ROS_INFO(std::string("Object do not exist").c_str());
                return false;
            }

            for( auto object : _current_scene.getAttachedObjects() ){
                _move_group.detachObject(object.first);
            }


            move_head(objects[object].mesh_poses[0]);
            _current_scene.removeCollisionObjects(_current_scene.getKnownObjectNames() );
            updateObjectCollisionScene();
            std::stringstream ss;

            ss << "Object to pick pose is :\n x: " << objects[ object ].mesh_poses[0].position.x <<
               "\n y: " << objects[ object ].mesh_poses[0].position.y <<
               "\n z: " << objects[ object ].mesh_poses[0].position.z << std::endl;

            ROS_INFO(ss.str().c_str());

            std::stringstream info2;
            info2 << "Trying to pick " << object << " in " <<  std::string( objects[ object ].id ) << " reference frame";
            ROS_INFO(info2.str().c_str());


            geometry_msgs::PoseStamped p;
            p.header.frame_id = objects[object].header.frame_id;
            p.pose.position.x = objects[object].mesh_poses[0].position.x ;
            p.pose.position.y = objects[object].mesh_poses[0].position.y ;
            p.pose.position.z = objects[object].mesh_poses[0].position.z ;

            std::vector<moveit_msgs::Grasp> grasps;
            PickGenerator gg( GRASP_FILE , p, 100 );
            std::vector<geometry_msgs::PoseStamped> pose;

            grasps = gg.generateGrasp();

            std::stringstream grasp_info;
            grasp_info << "number of grasps: " << grasps.size();
            ROS_INFO(grasp_info.str().c_str());
            ROS_INFO(_move_group.getEndEffectorLink().c_str());
            _move_group.allowReplanning(true);

            _move_group.setSupportSurfaceName("tablelaas");
            moveit::planning_interface::MoveItErrorCode sucess = _move_group.pick(object, grasps);
            if(moveit::planning_interface::MoveItErrorCode::SUCCESS == sucess.val){
                return true;
            }else{
                return false;
            }

        }
        /*
        bool place(std::vector<std::string> command){

            if(_current_scene.getAttachedObjects().empty()){
                ROS_INFO(std::string("No object in hand").c_str());
                return false;
            }
            if(_current_scene.getAttachedObjects().find(object) == _current_scene.getAttachedObjects().end()){
                ROS_INFO(std::string("Trying to place the wrong object").c_str());
                return false;
            }

            //updateObjectCollisionScene();
            ROS_INFO(std::string("Placing").c_str());
            move_head(pose);
            std::vector<moveit_msgs::PlaceLocation> place_location;
            place_location.resize(1);

            // Setting place location pose
            // +++++++++++++++++++++++++++
            place_location[0].place_pose.header.frame_id = "/odom_combined";

            // While placing it is the exact location of the center of the object.
            place_location[0].place_pose.pose.position.x = pose.position.x ;
            place_location[0].place_pose.pose.position.y = pose.position.y ;
            place_location[0].place_pose.pose.position.z = pose.position.z ;
            place_location[0].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

            // Setting pre-place approach
            // ++++++++++++++++++++++++++
            // Defined with respect to frame_id
            place_location[0].pre_place_approach.direction.header.frame_id = "base_footprint";
            // Direction is set as negative z axis
            place_location[0].pre_place_approach.direction.vector.z = -1.0;
            place_location[0].pre_place_approach.min_distance = 0.0001;
            place_location[0].pre_place_approach.desired_distance = 0.20;

            // Setting post-grasp retreat
            // ++++++++++++++++++++++++++
            // Defined with respect to frame_id
            place_location[0].post_place_retreat.direction.header.frame_id = "base_footprint";
            // Direction is set as negative y axis
            place_location[0].post_place_retreat.direction.vector.z = 1.0;
            place_location[0].post_place_retreat.min_distance = 0.0001;
            place_location[0].post_place_retreat.desired_distance = 0.30;

            // Setting posture of eef after placing object
            // +++++++++++++++++++++++++++++++++++++++++++
            // Similar to the pick case

            place_location[0].post_place_posture = _graspGen.getOpenGripper();

            // Call place to place the object using the place locations given.
            moveit::planning_interface::MoveItErrorCode sucess = _move_group.place(object, place_location);

            if(moveit::planning_interface::MoveItErrorCode::SUCCESS == sucess.val){
                return true;
            }else{
                return false;
            }

        }
        */
        bool placeAOnB(std::vector<std::string> command ){
            std::string objectA;
            std::string objectB;

            if(command[0] == "placeat"){
                objectA = command[2];
                objectB = command[3];
            }else{
                objectA = command[3];
                objectB = command[4];
            }

            if( objects.find(objectA) == objects.end() || objects.find(objectB) == objects.end()  ){
                ROS_INFO(std::string("Object do not exist").c_str());
                return false;
            }

            move_head(objects[objectA].mesh_poses[0]);
            //Create the planning scene
            updateObjectCollisionScene();


            std::stringstream ss;

            actioncontroller::PlaceGenerator placeGenerator( GRASP_FILE, objects[objectB], 0.06, 100);
            std::vector<moveit_msgs::PlaceLocation> place_location;

            place_location = placeGenerator.generatePlaceLocations() ;

            ss << "Object to place pose is :\n x: " << objects[ objectA ].mesh_poses[0].position.x <<
               "\n y: " << objects[ objectA ].mesh_poses[0].position.y <<
               "\n z: " << objects[ objectA ].mesh_poses[0].position.z << std::endl;

            ROS_INFO(ss.str().c_str());

            ss.clear();
            ss << "Object to place on pose is :\n x: " << objects[ objectB ].mesh_poses[0].position.x <<
               "\n y: " << objects[ objectB ].mesh_poses[0].position.y <<
               "\n z: " << objects[ objectB ].mesh_poses[0].position.z << std::endl;

            ROS_INFO(ss.str().c_str());

            _move_group.setPlanningTime(45.0);
            _move_group.setSupportSurfaceName(objectA);
            ss.clear();
            ss << "Number of location : " << place_location.size() ;
            ROS_INFO(ss.str().c_str());
            moveit::planning_interface::MoveItErrorCode sucess = _move_group.place(objectA, place_location);

            if(moveit::planning_interface::MoveItErrorCode::SUCCESS == sucess.val){
                return true;
            }else{
                return false;
            }
        }

        void updateObjectCollisionScene() {
            _current_scene.removeCollisionObjects(_current_scene.getKnownObjectNames() );
            std::vector<moveit_msgs::CollisionObject> vec;

            {
                std::lock_guard<std::mutex> lock(mutex);
                for(const auto& element : objects){
                    vec.push_back(element.second);
                    //_move_group.detachObject(element.second.id);
                    //ROS_INFO( element.second.id.c_str() );
                }
            }
            _current_scene.applyCollisionObjects(vec);
        }


    protected:
        //Action server
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<actioncontroller::ActionControllerAction> _as;
        actioncontroller::ActionControllerFeedback _feedback;
        actioncontroller::ActionControllerResult _result;
        std::string _action_name;
        ros::Subscriber _sub_moveit_objects;
        ros::Publisher _planning_scene_diff_publisher;
        actioncontroller::GraspGenerator _graspGen;

        //Moveit
        moveit::planning_interface::MoveGroupInterface _move_group;
        moveit::planning_interface::PlanningSceneInterface _current_scene;

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
                _as(nh_, name, boost::bind(&ActionController::executeCB, this, _1), false),
                _action_name(name),
                _move_group("right_arm"),
                _graspGen(GRASP_FILE)
        {
            _sub_moveit_objects = nh_.subscribe("moveit_objects", 1000, &ActionController::objects_update, this );
            _planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
            _move_group.setPlannerId("TRRTkConfigDefault");
            _as.start();
        }


        ~ActionController(void){
        }

        void executeCB(const actioncontroller::ActionControllerGoalConstPtr &msg){

            std::vector<std::string> command;
            std::string data = (std::string)msg->data;
            std::stringstream ss;
            ss << "Command received : " << data;
            ROS_INFO(ss.str().c_str());
            std::istringstream iss(data);
            while(std::getline(iss, data, ' ')){
                command.push_back(data);
            }
            if(command[0] == "stareAt"){
                _feedback.success = move_head( command );
            }else if(command[0] == "stareAtObject"){
                _feedback.success = move_head( command );
            }else if(command[0] == "pick" ){
                _feedback.success = pick( command );
                //}else if(command[0] == "place" ){
                //   _feedback.success = place( command);
            }else if(command[0] == "placeon" || command[0] == "placeat" ){
                _feedback.success = placeAOnB( command);
            }else if(command[0] == "goto_location" ){
                _feedback.success = goto_location( command );
            }else if(command[0] == "setup" ){
                _feedback.success = setup();
            }else{
                ROS_INFO(std::string("The required order do not exist").c_str());
            }
            _result.success = _feedback.success;
            _as.setSucceeded(_result);
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
