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
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "yaml-cpp/yaml.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "math.h"
#include "time.h"
#include "Eigen/Core"
#include "Eigen/Geometry"


namespace actioncontroller {

    class GraspGenerator {
        private:
            moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

            std::string _pathToFile;
            YAML::Node _providedGrasps;
            trajectory_msgs::JointTrajectory _openGripper;
            trajectory_msgs::JointTrajectory _closeGripper;
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
            GraspGenerator();
            GraspGenerator(std::string pathToyaml, geometry_msgs::PoseStamped target, int orientations);
            std::vector<geometry_msgs::PoseStamped> generatePoseOrientation();
            std::vector<moveit_msgs::Grasp> generateGrasp();
            void setOpenGripper();
            void setClosedGripper();
            moveit_msgs::GripperTranslation generateGraspMove(int graspNumber, std::string moveType);
            void displayPoseStampedMsg(geometry_msgs::PoseStamped p);

            //posegeneration
            void cubePoseGenerator(std::vector<geometry_msgs::PoseStamped> &poses, geometry_msgs::PoseStamped target ,double distFingerWrist, double cubeSize, int samples );
            void poseMsgToAffine3d(geometry_msgs::PoseStamped &p, Eigen::Affine3d &m);
            void affine3dToPoseMsg(Eigen::Affine3d m, geometry_msgs::PoseStamped &p);
            void displayAffine3d(Eigen::Affine3d m);

            geometry_msgs::PoseStamped generatePose(const std::default_random_engine &randomDouble, const Eigen::Affine3d &origin,
                                                    const Eigen::Affine3d &sampleRotation,
                                                    const Eigen::Affine3d &orientationFrameRotation,
                                                    const Eigen::Affine3d &frameTranslation);

            Eigen::Affine3d affine3dFromAngleAxis(double radianX, double radianY, double radianZ) const;
    };
}

#endif //ACTIONCONTROLLER_GRASPGENERATOR_H
