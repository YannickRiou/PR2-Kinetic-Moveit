//
// Created by dtrimoul on 8/20/18.
//

#include <vector>
#include "gazebo_moveIT_objects_synchroniser/GazeboMoveITObjectConverter.h"
#include "gazebo_msgs/GetWorldProperties.h"


void GazeboMoveITObjectConverter::convertToMoveITcollisionObjects(std::vector <gazebo_msgs::GetWorldProperties> vector) {

}


std::vector<moveit_msgs::CollisionObject> GazeboMoveITObjectConverter::getCollisionObjects(){
    return collisionObjects;
}