//
// Created by dtrimoul on 1/15/19.
//

#ifndef JSK_MOVEIT_OBJECT_SYNCHRONISER_OBJECTTOPICHANDLER_H
#define JSK_MOVEIT_OBJECT_SYNCHRONISER_OBJECTTOPICHANDLER_H

#include "ros/ros.h"
#include "moveit_msgs/CollisionObject.h"
#include "jsk_recognition_msgs/BoundingBox.h"
#include <regex>

class ObjectTopicHandler {

public:
    //ObjectTopicHandler();
    ObjectTopicHandler(ros::NodeHandle _nh, std::string _topicName, std::map<std::string, moveit_msgs::CollisionObject> &_moveitObjects);
   // ObjectTopicHandler operator()(ros::NodeHandle nh, std::string topicName, std::map<std::string, moveit_msgs::CollisionObject> &moveitObjects);
    const ros::NodeHandle &get_nh() const;
    const std::string &get_topicName() const;
    const std::map<std::string, moveit_msgs::CollisionObject> &_get_moveitObjects() const;
    const bool initTopic();
    void topicCallback(const jsk_recognition_msgs::BoundingBox::ConstPtr &msg );
    std::string getObjectName();
    shape_msgs::Mesh generateMeshesFromBB(
            jsk_recognition_msgs::BoundingBox_<std::allocator<void>>::_dimensions_type dimensions,
            jsk_recognition_msgs::BoundingBox_<std::allocator<void>>::_pose_type position);
    geometry_msgs::Point generatePoint(float x, float y, float z);

private:
    ros::NodeHandle _nh;
    std::string _topicName;
    std::map<std::string, moveit_msgs::CollisionObject> &_moveitObjects;
    ros::Subscriber _sub;
    std::string _objectName;


    void addVerticesToMesh(const geometry_msgs::Vector3 &dimensions, const geometry_msgs::Pose &position,
                           shape_msgs::Mesh &m);

    void addTrianglesToMesh(shape_msgs::Mesh &m) const;
};


#endif //JSK_MOVEIT_OBJECT_SYNCHRONISER_OBJECTTOPICHANDLER_H
