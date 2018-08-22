//
// Created by dtrimoul on 8/20/18.
//

#ifndef GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOOBJECTLISTMAKER_H
#define GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOOBJECTLISTMAKER_H


#include <vector>
#include "gazebo_msgs/GetWorldProperties.h"

class GazeboObjectListMaker {
private:

    std::vector<gazebo_msgs::GetWorldProperties> objectList;


public:
    GazeboObjectListMaker();
    void updateObjectList();

    std::vector <gazebo_msgs::GetWorldProperties> getObjectList();
};


#endif //GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOOBJECTLISTMAKER_H
