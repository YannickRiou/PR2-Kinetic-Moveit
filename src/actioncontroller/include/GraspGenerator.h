//
// Created by dtrimoul on 9/27/18.
//

#ifndef ACTIONCONTROLLER_GRASPGENERATOR_H
#define ACTIONCONTROLLER_GRASPGENERATOR_H

#include "string"
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/Grasp.h"
#include "yaml-cpp/yaml.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

namespace actioncontroller {

    class GraspGenerator {
        private:
            std::string _pathToFile;
            YAML::Node _providedGrasps;
            trajectory_msgs::JointTrajectory _openGripper;
            trajectory_msgs::JointTrajectory _closeGripper;
            geometry_msgs::PoseStamped _target;
            std::vector< std::vector<double> > orientations = {{0,0,0},
                                                             {0,0, M_PI / 2},
                                                             {0,M_PI / 2, 0},
                                                             {0,M_PI / 2 , M_PI / 2},
                                                             {M_PI / 2,0,0},
                                                             {M_PI / 2,0,M_PI / 2},
                                                             {M_PI / 2,M_PI / 2,0},
                                                             {M_PI / 2,M_PI / 2,M_PI / 2} };

        public:
            GraspGenerator();
            GraspGenerator(std::string pathToyaml, geometry_msgs::PoseStamped target);
            std::vector<geometry_msgs::PoseStamped> generatePoseOrientation();
            std::vector<moveit_msgs::Grasp> generateGrasp();
            void setOpenGripper();
            void setClosedGripper();
            moveit_msgs::GripperTranslation generateGraspMove(int graspNumber, std::string moveType);
            void displayPoseStampedMsg(geometry_msgs::PoseStamped p);
    };
}

#endif //ACTIONCONTROLLER_GRASPGENERATOR_H
