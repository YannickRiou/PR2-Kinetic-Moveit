//
// Created by dtrimoul on 8/20/18.
//

#include "gazebo/gazebo.hh"
#include "ros/ros.h"
#include "moveit_msgs/CollisionObject.h"
#include "gazebo_moveIT_objects_synchroniser/GazeboObjectListMaker.h"
#include "gazebo_moveIT_objects_synchroniser/GazeboMoveITObjectConverter.h"
#include "gazebo_moveIT_objects_synchroniser/GazeboMeshManager.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "Gazebo_moveIT_Object_Synchroniser");
    ros::NodeHandle n;
    GazeboObjectListMaker golm;
    GazeboMoveITObjectConverter gmioc;
    ros::Publisher pub = n.advertise<moveit_msgs::CollisionObject>("moveit_objects", 1000);
    ros::ServiceClient clientWorldProperties = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
    ros::ServiceClient clientObjectProperties = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_model_state");
    ros::Rate freq(1);


    gazebo_msgs::GetWorldProperties worldProperties;
    while(ros::ok()){
        std::vector<std::shared_ptr<shapes::Mesh>> currentMesh ;
        if (clientWorldProperties.call(worldProperties))
        {
            for(int i=0; i < worldProperties.response.model_names.size(); ++i){
                GazeboMeshManager gazeboMeshManager;
                try {
                    std::shared_ptr<shapes::Mesh> m = gazeboMeshManager.getMesh(worldProperties.response.model_names[i]);
                    currentMesh.push_back(m);
                }catch(std::string exc){
                    ROS_INFO(exc.c_str());
                }
            }
            ROS_INFO("size of mesh vector %i", currentMesh.size());
        }
        ROS_INFO("if out");
        //gmioc.convertToMoveITcollisionObjects( golm.getObjectList() );

        //pub.publish( &gmioc.getCollisionObjects()[0] );
        ROS_INFO("List clean");
        ros::spinOnce();
        freq.sleep();
        ROS_INFO("End While");
        ROS_INFO("size of mesh vector %i", currentMesh.size());
    }

    return 0;
}



