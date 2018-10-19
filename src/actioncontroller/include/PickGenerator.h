//
// Created by dtrimoul on 9/27/18.
//

#ifndef ACTIONCONTROLLER_PICKGENERATOR_H
#define ACTIONCONTROLLER_PICKGENERATOR_H

#include "string"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/Grasp.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "math.h"
#include "time.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ActionControllerTools.h"
#include "GraspGenerator.h"

namespace actioncontroller {

    class PickGenerator {
        private:
            actioncontroller::GraspGenerator _graspGen;
            actioncontroller::ActionControllerTools tools;
            std::string _pathToFile;
            geometry_msgs::PoseStamped _target;
            int _orientations;
            /*std::vector< std::vector<double> > orientations =   {
                                                                    {0,0,0, -0.175, 0, 0.01},
                                                                    {0,0, M_PI / 2, 0, -0.175, +0.01 },
                                                                    {0,M_PI / 2, 0 , 0, 0, 0.175},
                                                                    {0,M_PI / 2 , M_PI / 2, 0, 0, 0.175},
                                                                    {M_PI / 2,0,0, -0.175, 0, 0},
                                                                    {M_PI / 2,0,M_PI / 2, 0, 0, 0.175 },
                                                                    {M_PI / 2,M_PI / 2,0 ,0, 0.175, 0  },
                                                                    {M_PI / 2,M_PI / 2,M_PI / 2, 0, 0, 0.175}
                                                                };
            */

        public:
            PickGenerator(std::string pathToyaml, geometry_msgs::PoseStamped target, int orientations);
            std::vector<geometry_msgs::PoseStamped> generatePoseOrientation();
            std::vector<moveit_msgs::Grasp> generateGrasp();

            //posegeneration
            void cubePoseGenerator(std::vector<geometry_msgs::PoseStamped> &poses, geometry_msgs::PoseStamped target ,double distFingerWrist, double cubeSize, int samples );

            geometry_msgs::PoseStamped generatePose(const Eigen::Affine3d &origin,
                                                    const Eigen::Affine3d &sampleRotation,
                                                    const Eigen::Affine3d &orientationFrameRotation,
                                                    const Eigen::Affine3d &frameTranslation);

    };
}

#endif //ACTIONCONTROLLER_PICKGENERATOR_H
