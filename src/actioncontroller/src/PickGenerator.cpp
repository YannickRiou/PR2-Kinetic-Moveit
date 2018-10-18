//
// Created by dtrimoul on 9/27/18.
//

#include "PickGenerator.h"


namespace actioncontroller {

    PickGenerator::PickGenerator(std::string path, geometry_msgs::PoseStamped target, int orientations) : _graspGen(path)  {
        _target = target;
        _orientations = orientations;
    }

    std::vector<moveit_msgs::Grasp> PickGenerator::generateGrasp(){
        ROS_INFO("creating grasps");
        std::stringstream ss;
        ss << "Grasp Number " <<  _graspGen.getProvidedGraspsNumber();
        ROS_INFO(ss.str().c_str());
        std::vector<moveit_msgs::Grasp> grasps;
        std::vector<geometry_msgs::PoseStamped> targets = generatePoseOrientation();
        for(geometry_msgs::PoseStamped target : targets){
            for (unsigned i = 0; i < _graspGen.getProvidedGraspsNumber() ; ++i) {
                moveit_msgs::Grasp g;
                g.grasp_pose = target;
                g.pre_grasp_approach = _graspGen.generateGraspMove(i, "pre");
                g.post_grasp_retreat = _graspGen.generateGraspMove(i, "post");
                g.pre_grasp_posture = _graspGen.getOpenGripper();
                g.grasp_posture = _graspGen.getCloseGripper();
                grasps.push_back(g);
            }
        }
        return grasps;
    }

    std::vector<geometry_msgs::PoseStamped> PickGenerator::generatePoseOrientation(){

        std::vector<geometry_msgs::PoseStamped> targetOrientations;
        cubePoseGenerator(targetOrientations, _target, 0.175, 0.06, _orientations);
        return targetOrientations;
    }

    void PickGenerator::cubePoseGenerator(std::vector<geometry_msgs::PoseStamped> &poses, geometry_msgs::PoseStamped target ,double distFingerWrist, double cubeSize, int samples ){
        //double endEffectorLength = sqrt( pow(endEffetor.pose.position.x - wrist.pose.position.x, 2 ) + pow(endEffetor.pose.position.y - wrist.pose.position.y, 2 ) + pow(endEffetor.pose.position.z - wrist.pose.position.z , 2 ) );
        //double desiredDistBetweenWristAndTarget = (cubeSize / 2) - fingerLength + endEffectorLength;
        ROS_INFO(std::string("Starting the cube grasp generation").c_str());
        double desiredDistBetweenWristAndTarget = distFingerWrist;
        std::uniform_real_distribution<double> unif(0,2);
        std::default_random_engine randomDouble;

        Eigen::Affine3d origin;
        tools.poseMsgToAffine3d(target, origin);
        ROS_INFO(std::string("origin").c_str());
        tools.displayAffine3d(origin);

        //create the to transformation matrix
        Eigen::Affine3d originToReferenceFrame;
        originToReferenceFrame = origin.inverse();
        ROS_INFO(std::string("originToReferenceFrame").c_str());
        tools.displayAffine3d(originToReferenceFrame);

        // A factoriser
        Eigen::Affine3d x_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(desiredDistBetweenWristAndTarget, 0,0)));
        Eigen::Affine3d y_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(0, desiredDistBetweenWristAndTarget,0)));
        Eigen::Affine3d z_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(desiredDistBetweenWristAndTarget, 0,0)));

        for (int i = 0; i < samples; ++i) {
            //compute the pose for the xz plan
            //create a pose desiredDistBetweenWristAndTarget away from the target center.

            poses.push_back( generatePose(origin,
                                            tools.affine3dFromAngleAxis(0, unif(randomDouble) * M_PI, 0),
                                            tools.affine3dFromAngleAxis( 0, M_PI, 0) ,
                                            x_frameTranslation) );

            /*I have to define plan constraint for approach depending of the grasp orientation / position
            //compute the pose for the yz plan
            poses.push_back( generatePose(origin,
                                          tools.affine3dFromAngleAxis(unif(randomDouble) * M_PI, 0, 0),
                                          tools.affine3dFromAngleAxis(0, 0, -0.5*M_PI) ,
                                          y_frameTranslation) );

            //compute the pose for the xy plan
            poses.push_back( generatePose(origin,
                                          tools.affine3dFromAngleAxis(0, 0, unif(randomDouble) * M_PI),
                                          tools.affine3dFromAngleAxis(0.5 * M_PI, M_PI, 0) ,
                                          z_frameTranslation) );
            */
        }




    }

    geometry_msgs::PoseStamped PickGenerator::generatePose(const Eigen::Affine3d &origin,
                                                           const Eigen::Affine3d &sampleRotation,
                                                           const Eigen::Affine3d &orientationFrameRotation,
                                                           const Eigen::Affine3d &frameTranslation) {
        geometry_msgs::PoseStamped p;

        ROS_INFO(std::string("sampleRotation").c_str());
        tools.displayAffine3d(sampleRotation);

        Eigen::Affine3d new_point = sampleRotation * frameTranslation ;
        ROS_INFO(std::string("new_point").c_str());
        tools.displayAffine3d(new_point);

        //change orientation to face toward the target
        new_point =  new_point * orientationFrameRotation  ;
        ROS_INFO(std::string("Rotated new_point").c_str());
        tools.displayAffine3d(new_point);

        //Transform the createdPose to map frame
        new_point =  origin * new_point  ;
        ROS_INFO(std::string("new_point in map frame").c_str());
        tools.displayAffine3d(new_point);

        //store the pose
        tools.affine3dToPoseMsg(new_point, p);
        tools.displayPoseStampedMsg(p);
        return p;
    }


}