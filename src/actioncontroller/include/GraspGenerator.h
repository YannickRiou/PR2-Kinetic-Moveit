//
// Created by dtrimoul on 10/16/18.
//

#ifndef ACTIONCONTROLLER_GRASPGENERATOR_H
#define ACTIONCONTROLLER_GRASPGENERATOR_H

#include "moveit_msgs/GripperTranslation.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "ros/ros.h"

namespace actioncontroller {

    class GraspGenerator {

    private :
                std::string _pathToFile;
                trajectory_msgs::JointTrajectory _openGripper;
                trajectory_msgs::JointTrajectory _closeGripper;
                YAML::Node _providedGrasps;
                void setOpenGripper();
                void setClosedGripper();
    public:
            int getProvidedGraspsNumber();

            const trajectory_msgs::JointTrajectory &getOpenGripper() const;
            const trajectory_msgs::JointTrajectory &getCloseGripper() const;
            GraspGenerator(std::string pathToFile);
            moveit_msgs::GripperTranslation generateGraspMove(int graspNumber, std::string moveType);

    };


}
#endif //ACTIONCONTROLLER_GRASPGENERATOR_H
