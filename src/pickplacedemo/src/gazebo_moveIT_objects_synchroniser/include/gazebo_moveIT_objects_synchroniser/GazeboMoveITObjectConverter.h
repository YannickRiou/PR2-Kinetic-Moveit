//
// Created by dtrimoul on 8/20/18.
//

#ifndef GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOMOVEITOBJECTCONVERTER_H
#define GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOMOVEITOBJECTCONVERTER_H


#include <moveit_msgs/CollisionObject.h>
#include "gazebo_msgs/GetWorldProperties.h"

class GazeboMoveITObjectConverter {

private:
    std::vector<moveit_msgs::CollisionObject> collisionObjects;
public:

    void convertToMoveITcollisionObjects(std::vector <gazebo_msgs::GetWorldProperties> vector);

    std::vector<moveit_msgs::CollisionObject> getCollisionObjects();
};


#endif //GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOMOVEITOBJECTCONVERTER_H
