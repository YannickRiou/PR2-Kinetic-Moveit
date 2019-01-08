//
// Created by dtrimoul on 1/8/19.
//

//
// Created by dtrimoul on 8/20/18.
//

#include "ros/ros.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_custom_msgs/CollisionObjectArray.h"



void getObjectTopic(ros::master::V_TopicInfo);

int main(int argc, char **argv){

    ros::init(argc, argv, "Gazebo_moveIT_Object_Synchroniser");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<moveit_custom_msgs::CollisionObjectArray>("moveit_objects", 1000);
    ros::Rate freq(100);

    std::vector<moveit_msgs::CollisionObject> collidingObjects;
    moveit_custom_msgs::CollisionObjectArray collidingObjectsMsg;

    while(ros::ok()){
        //getting object topic from jsk
        ros::master::V_TopicInfo obj_top;
        getObjectTopic( obj_top );

        //create the subscriber for each object

        //check the object list

        //build a moveit object from list

        //publish the list
        pub.publish( collidingObjectsMsg );
        //ROS_INFO("List clean");
        ros::spinOnce();
        freq.sleep();
        //ROS_INFO("End While");
        //ROS_INFO("size of mesh vector %i", currentMesh.size());
    }

    return 0;
}

void getObjectTopic(ros::master::V_TopicInfo obj_topic){
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    for(ros::master::TopicInfo t : topic_list){
        if(t.name.find("object_bb") != std::string::npos){
        obj_topic.push_back(t);
        std::cout << t.name << std::endl;
        }
    }
}