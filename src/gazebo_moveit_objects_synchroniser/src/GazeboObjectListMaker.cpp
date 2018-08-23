//
// Created by dtrimoul on 8/20/18.
//

#include "gazebo_moveit_objects_synchroniser/GazeboObjectListMaker.h"

GazeboObjectListMaker::GazeboObjectListMaker(){

}

void GazeboObjectListMaker::updateObjectList() {

}

std::vector <gazebo_msgs::GetWorldProperties> GazeboObjectListMaker::getObjectList() {
    return objectList;
}
