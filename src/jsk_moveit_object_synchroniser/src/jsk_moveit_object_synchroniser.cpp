//
// Created by dtrimoul on 1/8/19.
//

//
// Created by dtrimoul on 8/20/18.
//

#include "ros/ros.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_custom_msgs/CollisionObjectArray.h"
#include "jsk_moveit_object_synchroniser/ObjectTopicHandler.h"



void getObjectTopic(ros::master::V_TopicInfo);

int main(int argc, char **argv){

    ros::init(argc, argv, "jsk_moveit_object_synchroniser");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<moveit_custom_msgs::CollisionObjectArray>("moveit_objects", 1000);
    ros::Rate freq(100);


    std::map<std::string, moveit_msgs::CollisionObject> _objectDatabase;
    ObjectTopicHandler top1(n, "/object_bb/green_cube", _objectDatabase );
    top1.initTopic();
    ObjectTopicHandler top2(n, "/object_bb/red_cube", _objectDatabase );
    top2.initTopic();
    ObjectTopicHandler top3(n, "/object_bb/blue_cube", _objectDatabase );
    top3.initTopic();
    ObjectTopicHandler top4(n, "/object_bb/table", _objectDatabase );
    top4.initTopic();
    //getting object topic from jsk
    //ros::master::V_TopicInfo obj_top;
    //getObjectTopic( obj_top );

    while(ros::ok()){

        moveit_custom_msgs::CollisionObjectArray collidingObjectsMsg;
        for(std::pair<std::string, moveit_msgs::CollisionObject> e : _objectDatabase){
            collidingObjectsMsg.data.push_back(e.second);
        }
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