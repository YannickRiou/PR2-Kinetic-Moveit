//
// Created by dtrimoul on 8/20/18.
//

#include "gazebo/gazebo.hh"
#include "ros/ros.h"
#include "moveit_msgs/CollisionObject.h"
#include "gazebo_moveit_objects_synchroniser/GazeboMeshManager.h"
#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_moveit_objects_synchroniser/CollisionObjectArray.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "Gazebo_moveIT_Object_Synchroniser");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<gazebo_moveit_objects_synchroniser::CollisionObjectArray>("moveit_objects", 1000);
    ros::ServiceClient clientWorldProperties = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
    ros::ServiceClient clientObjectProperties = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Rate freq(1);

    std::vector<moveit_msgs::CollisionObject> collidingObjects;
    gazebo_moveit_objects_synchroniser::CollisionObjectArray collidingObjectsMsg;

    gazebo_msgs::GetWorldProperties worldProperties;
    while(ros::ok()){

        if (clientWorldProperties.call(worldProperties))
        {
            for(int i=0; i < worldProperties.response.model_names.size(); ++i){
                moveit_msgs::CollisionObject sceneObject;
                sceneObject.id = worldProperties.response.model_names[i] ;
                GazeboMeshManager::GazeboMeshManager gazeboMeshManager;
                shape_msgs::Mesh m;
                try {
                    m = gazeboMeshManager.getMesh(worldProperties.response.model_names[i]);
                    sceneObject.meshes.resize(1);
                    sceneObject.meshes[0] = m;
                    sceneObject.mesh_poses.resize(1);
                    gazebo_msgs::GetModelState modelState;
                    modelState.request.model_name = worldProperties.response.model_names[i];
                    if(clientObjectProperties.call(modelState)){
                        sceneObject.mesh_poses[0] = modelState.response.pose;
                        //Moving the object to fit the reference frame of odom combined
                        //Have to be fixed by removing gap between odom_combined and map that is the frame of the object
                        sceneObject.mesh_poses[0].position.z = sceneObject.mesh_poses[0].position.z - 0.05;
                        sceneObject.header.stamp = modelState.response.header.stamp;
                        sceneObject.header.frame_id = "/map";
                        sceneObject.header.seq = modelState.response.header.seq;
                    }
                    sceneObject.operation = sceneObject.ADD;
                    collidingObjects.push_back(sceneObject);
                }catch(GazeboMeshManager::IoException exc){
                    ROS_INFO(exc.what());
                }
            }

        }
        collidingObjectsMsg.data.resize(collidingObjects.size());
        for(int i=0; i < collidingObjects.size(); ++i){
            collidingObjectsMsg.data[i] = collidingObjects.at(i);
        }
        //ROS_INFO("if out");
        //gmioc.convertToMoveITcollisionObjects( golm.getObjectList() );

        pub.publish( collidingObjectsMsg );
        //ROS_INFO("List clean");
        ros::spinOnce();
        freq.sleep();
        //ROS_INFO("End While");
        //ROS_INFO("size of mesh vector %i", currentMesh.size());
    }

    return 0;
}



