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

namespace actioncontroller {

    class GraspGenerator {
        private:
            std::string _pathToFile;
            YAML::Node _providedGrasps;
            trajectory_msgs::JointTrajectory _openGripper;
            trajectory_msgs::JointTrajectory _closeGripper;
            geometry_msgs::PoseStamped _pickPosition;
            std::vector<moveit_msgs::Grasp> _grasps;
            geometry_msgs::PoseStamped _target;

        public:
            GraspGenerator();
            GraspGenerator(std::string pathToyaml, geometry_msgs::PoseStamped target);
            const std::string &getPathToFile() const;
            void setPathToFile(const std::string &pathToFile);
            std::vector<moveit_msgs::Grasp> generateGrasp();
            void setOpenGripper();
            void setClosedGripper();
            moveit_msgs::GripperTranslation generateGraspMove(int graspNumber, std::string moveType);
    };
}

#endif //ACTIONCONTROLLER_GRASPGENERATOR_H
