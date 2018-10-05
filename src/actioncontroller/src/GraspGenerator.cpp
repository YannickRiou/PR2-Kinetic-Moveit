//
// Created by dtrimoul on 9/27/18.
//

#include "GraspGenerator.h"


namespace actioncontroller {

    GraspGenerator::GraspGenerator(std::string path, geometry_msgs::PoseStamped target) {
        _pathToFile = path;
        _target = target;
        _providedGrasps = YAML::LoadFile(_pathToFile);
        setOpenGripper();
        setClosedGripper();
    }

    GraspGenerator::GraspGenerator(){

    }

    std::vector<moveit_msgs::Grasp> GraspGenerator::generateGrasp(){
        ROS_INFO("creating grasps");
        std::stringstream ss;
        ss << "Grasp Number " <<  _providedGrasps["grasp"].size();
        ROS_INFO(ss.str().c_str());
        std::vector<moveit_msgs::Grasp> grasps;
        std::vector<geometry_msgs::PoseStamped> targets = generatePoseOrientation();
        for(geometry_msgs::PoseStamped target : targets){
            for (unsigned i = 0; i < _providedGrasps["grasp"].size() ; ++i) {
                moveit_msgs::Grasp g;
                g.grasp_pose = target;
                g.pre_grasp_approach = generateGraspMove(i, "pre");
                g.post_grasp_retreat = generateGraspMove(i, "post");
                g.pre_grasp_posture = _openGripper;
                g.grasp_posture = _closeGripper;
                grasps.push_back(g);
            }
        }

        return grasps;
    }

    moveit_msgs::GripperTranslation GraspGenerator::generateGraspMove(int graspNumber, std::string moveType) {
        ROS_INFO("Building approach and retreat");
        moveit_msgs::GripperTranslation j;
         j.direction.header.frame_id = _providedGrasps["grasp"][graspNumber][moveType]["direction"]["header_frame_id"].as<std::string>();
         if(_providedGrasps["grasp"][graspNumber][moveType]["direction"]["vector_x"] != NULL){
             j.direction.vector.x = _providedGrasps["grasp"][graspNumber][moveType]["direction"]["vector_x"].as<float>() ;
         }
         if(_providedGrasps["grasp"][graspNumber][moveType]["direction"]["vector_y"] != NULL){
             j.direction.vector.y = _providedGrasps["grasp"][graspNumber][moveType]["direction"]["vector_y"].as<float>() ;
         }
         if(_providedGrasps["grasp"][graspNumber][moveType]["direction"]["vector_z"]){
             j.direction.vector.z  = _providedGrasps["grasp"][graspNumber][moveType]["direction"]["vector_z"].as<float>() ;
         }
         j.min_distance = _providedGrasps["grasp"][graspNumber][moveType]["direction"]["min_distance"].as<float>() ;
         j.desired_distance = _providedGrasps["grasp"][graspNumber][moveType]["direction"]["desired_distance"].as<float>() ;
        return j;
    }

    void GraspGenerator::setOpenGripper(){
        ROS_INFO("creating the gripper openning");
        _openGripper.joint_names.resize(_providedGrasps["open"]["joint"].size());
        ROS_INFO("Parsing join");
        for(unsigned i=0; i < _providedGrasps["open"]["joint"].size(); ++i){
            _openGripper.joint_names[i] =  _providedGrasps["open"]["joint"][i].as<std::string>() ;
        }
        ROS_INFO("Parsing join values");
        _openGripper.points.resize(1);
        _openGripper.points[0].positions.resize(_providedGrasps["open"]["joint"].size());
        for(unsigned i=0; i < _providedGrasps["open"]["position"].size(); ++i){
            _openGripper.points[0].positions[i] = _providedGrasps["open"]["position"][i].as<float>();
        }
        ros::Duration d(_providedGrasps["open"]["time_from_start"].as<int>());
        _openGripper.points[0].time_from_start =  d ;
    }

    void GraspGenerator::setClosedGripper(){
        ROS_INFO("creating the gripper closing");
        _closeGripper.joint_names.resize(_providedGrasps["close"]["joint"].size());
        for(unsigned i=0; i < _providedGrasps["close"]["joint"].size() ; ++i){
            _closeGripper.joint_names[i] = _providedGrasps["close"]["joint"][i].as<std::string>()  ;
        }
        _closeGripper.points.resize(1);
        _closeGripper.points[0].positions.resize(_providedGrasps["close"]["joint"].size());
        for(unsigned i=0; i < _providedGrasps["close"]["position"].size(); ++i){
            _closeGripper.points[0].positions[i] = _providedGrasps["close"]["position"][i].as<float>();
        }
        ros::Duration d(_providedGrasps["close"]["time_from_start"].as<int>());
        _closeGripper.points[0].time_from_start = d;
    }

    std::vector<geometry_msgs::PoseStamped> GraspGenerator::generatePoseOrientation(){

        std::vector<geometry_msgs::PoseStamped> targetOrientations;

        for(int i=0; i < orientations.size(); ++i){
            geometry_msgs::PoseStamped p;
            p.header.frame_id = _target.header.frame_id;
            p.pose.position.x = _target.pose.position.x + orientations[i][3];
            p.pose.position.y = _target.pose.position.y + orientations[i][4];
            p.pose.position.z = _target.pose.position.z + orientations[i][5];
            p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( orientations[i][0], orientations[i][1], orientations[i][2]);
            displayPoseStampedMsg(p);
            targetOrientations.push_back(p);
        }
        return targetOrientations;
    }

    void GraspGenerator::displayPoseStampedMsg(geometry_msgs::PoseStamped p){
        std::stringstream ss;
        ss << "header: " << p.header.frame_id
        << "\npose_x : " << p.pose.position.x
        << "\npose_y : " << p.pose.position.y
        << "\npose_z : " << p.pose.position.z
        << "\norientation_x : " << p.pose.orientation.x
        << "\norientation_Y : " << p.pose.orientation.y
        << "\norientation_Z : " << p.pose.orientation.z
        << "\norientation_W : " << p.pose.orientation.w;
        ROS_INFO(ss.str().c_str());
    }

    void GraspGenerator::CubePoseGenerator(std::vector<geometry_msgs::PoseStamped> &poses, geometry_msgs::PoseStamped target  , geometry_msgs::PoseStamped endEffetor, geometry_msgs::PoseStamped wrist, double fingerLength, double cubeSize, int samples ){
        double endEffectorLength = sqrt( pow(endEffetor.pose.position.x - wrist.pose.position.x, 2 ) + pow(endEffetor.pose.position.y - wrist.pose.position.y, 2 ) + pow(endEffetor.pose.position.z - wrist.pose.position.z , 2 ) );
        double desiredDistBetweenWristAndTarget = (cubeSize / 2) - fingerLength + endEffectorLength;

        std::uniform_real_distribution<double> unif(0,2);
        std::default_random_engine randomDouble;

        Eigen::Matrix4d origin;
        PoseMsgToMatrix4d(target, origin);

        //create the to transformation matrix
        Eigen::Affine3d originToReferenceFrame;
        originToReferenceFrame = origin.inverse();

        Eigen::Affine3d x_orientationFrameRotation;
        x_orientationFrameRotation = Eigen::AngleAxisd(1*M_PI, Eigen::Vector3d::UnitX())
                                    * Eigen::AngleAxisd(1*M_PI,  Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

        Eigen::Affine3d y_orientationFrameRotation;
        y_orientationFrameRotation = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX())
                                     * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
                                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

        Eigen::Affine3d z_orientationFrameRotation ;
        z_orientationFrameRotation = Eigen::AngleAxisd(1*M_PI, Eigen::Vector3d::UnitX())
                                                                                * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
                                                                                * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

        Eigen::Affine3d x_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(desiredDistBetweenWristAndTarget, 0,0)));
        Eigen::Affine3d y_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(0, desiredDistBetweenWristAndTarget,0)));
        Eigen::Affine3d z_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(desiredDistBetweenWristAndTarget, 0,0)));

        for (int i = 0; i < samples; ++i) {
            //compute the pose for the xy plan
            //create a pose desiredDistBetweenWristAndTarget away from the target center.
            geometry_msgs::PoseStamped p;


            Eigen::Affine3d x_sampleRotation;
            x_sampleRotation = Eigen::AngleAxisd(unif(randomDouble)*M_PI, Eigen::Vector3d::UnitX())
                               * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

            Eigen::Matrix4d new_point = (x_frameTranslation * x_sampleRotation).matrix();
            //change orientation to face toward the target
            new_point *= x_orientationFrameRotation.matrix() ;
            //Transform the createdPose to map frame
            new_point *= originToReferenceFrame.matrix() ;
            //store the pose
            Matrix4dToPoseMsg(new_point, p);
            poses.push_back(p);
        }
            //compute the pose for the yz plan

            //compute the pose for the xz plan

    }

    void GraspGenerator::PoseMsgToMatrix4d(geometry_msgs::PoseStamped p, Eigen::Matrix4d m){
        Eigen::Quaterniond q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
        Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
        m <<    rot(0), rot(1), rot(2) , p.pose.position.x,
                rot(3), rot(4), rot(5) , p.pose.position.y,
                rot(6), rot(7), rot(8) , p.pose.position.z,
                0,      0,      0,      1;

    }

    void GraspGenerator::Matrix4dToPoseMsg(Eigen::Matrix4d m, geometry_msgs::PoseStamped p){
        p.header.frame_id = "map";
        p.pose.position.x = m(3);
        p.pose.position.y = m(7);
        p.pose.position.z = m(11);
        Eigen::Matrix3d rot;
        rot << m(0) , m(1), m(2), m(4), m(5) , m(6), m(8), m(9), m(10) ;
        Eigen::Quaterniond q(rot);
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();
    }

}