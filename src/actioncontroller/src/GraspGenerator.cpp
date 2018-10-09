//
// Created by dtrimoul on 9/27/18.
//

#include "GraspGenerator.h"


namespace actioncontroller {

    GraspGenerator::GraspGenerator(std::string path, geometry_msgs::PoseStamped target, int orientations) {
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/odom_combined","/moveit_visual_markers"));
        _pathToFile = path;
        _target = target;
        _providedGrasps = YAML::LoadFile(_pathToFile);
        _orientations = orientations;
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
        /*
        for(int i=0; i < orientations.size(); ++i){

            geometry_msgs::PoseStamped p;
            p.header.frame_id = _target.header.frame_id;
            p.pose.position.x = _target.pose.position.x + orientations[i][3];
            p.pose.position.y = _target.pose.position.y + orientations[i][4];
            p.pose.position.z = _target.pose.position.z + orientations[i][5];
            p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( orientations[i][0], orientations[i][1], orientations[i][2]);
            displayPoseStampedMsg(p);
            targetOrientations.push_back(p);


        } */
        cubePoseGenerator(targetOrientations, _target, 0.175, 0.06, _orientations);
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
        visual_tools_.get()->publishCuboid(p.pose, 0.01, 0.01, 0.01, rviz_visual_tools::colors::BLUE );
        visual_tools_.get()->trigger();
        ROS_INFO(ss.str().c_str());
    }

    void GraspGenerator::cubePoseGenerator(std::vector<geometry_msgs::PoseStamped> &poses, geometry_msgs::PoseStamped target ,double distFingerWrist, double cubeSize, int samples ){
        //double endEffectorLength = sqrt( pow(endEffetor.pose.position.x - wrist.pose.position.x, 2 ) + pow(endEffetor.pose.position.y - wrist.pose.position.y, 2 ) + pow(endEffetor.pose.position.z - wrist.pose.position.z , 2 ) );
        //double desiredDistBetweenWristAndTarget = (cubeSize / 2) - fingerLength + endEffectorLength;
        ROS_INFO(std::string("Starting the cube grasp generation").c_str());
        double desiredDistBetweenWristAndTarget = distFingerWrist;
        std::uniform_real_distribution<double> unif(0,2);
        std::default_random_engine randomDouble;

        Eigen::Affine3d origin;
        poseMsgToAffine3d(target, origin);
        ROS_INFO(std::string("origin").c_str());
        displayAffine3d(origin);

        //create the to transformation matrix
        Eigen::Affine3d originToReferenceFrame;
        originToReferenceFrame = origin.inverse();
        ROS_INFO(std::string("originToReferenceFrame").c_str());
        displayAffine3d(originToReferenceFrame);

        Eigen::Affine3d x_orientationFrameRotation;
        x_orientationFrameRotation = Eigen::AngleAxisd(1*M_PI, Eigen::Vector3d::UnitX())
                                    * Eigen::AngleAxisd(1*M_PI,  Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

        ROS_INFO(std::string("x_orientationFrameRotation").c_str());
        displayAffine3d(x_orientationFrameRotation);

        Eigen::Affine3d y_orientationFrameRotation;
        y_orientationFrameRotation = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX())
                                     * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
                                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

        Eigen::Affine3d z_orientationFrameRotation ;
        z_orientationFrameRotation = Eigen::AngleAxisd(1*M_PI, Eigen::Vector3d::UnitX())
                                    * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

        Eigen::Affine3d x_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(desiredDistBetweenWristAndTarget, 0,0)));
        ROS_INFO(std::string("x_frameTranslation").c_str());
        displayAffine3d(x_frameTranslation);

        Eigen::Affine3d y_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(0, desiredDistBetweenWristAndTarget,0)));
        Eigen::Affine3d z_frameTranslation(Eigen::Translation3d(Eigen::Vector3d(desiredDistBetweenWristAndTarget, 0,0)));

        for (int i = 0; i < samples; ++i) {
            //compute the pose for the xy plan
            //create a pose desiredDistBetweenWristAndTarget away from the target center.
            geometry_msgs::PoseStamped p;


            Eigen::Affine3d x_sampleRotation;
            x_sampleRotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                               * Eigen::AngleAxisd(unif(randomDouble)*M_PI,  Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

            ROS_INFO(std::string("x_sampleRotation").c_str());
            displayAffine3d(x_sampleRotation);

            Eigen::Affine3d new_point = x_sampleRotation * x_frameTranslation ;
            ROS_INFO(std::string("new_point").c_str());
            displayAffine3d(new_point);

            //change orientation to face toward the target
            new_point =  new_point * x_orientationFrameRotation  ;
            ROS_INFO(std::string("Rotated new_point").c_str());
            displayAffine3d(new_point);

            //Transform the createdPose to map frame
            new_point =  origin * new_point  ;
            ROS_INFO(std::string("new_point in map frame").c_str());
            displayAffine3d(new_point);

            //store the pose
            affine3dToPoseMsg(new_point, p);
            poses.push_back(p);
            displayPoseStampedMsg(p);
        }
            //compute the pose for the yz plan

            //compute the pose for the xz plan

    }

    void GraspGenerator::poseMsgToAffine3d(geometry_msgs::PoseStamped &p, Eigen::Affine3d &m){
        /* m = Eigen::Affine3d::fromPositionOrientationScale(Eigen::Vector3d(p.pose.position.x, p.pose.position.y, p.pose.position.z),
                Eigen::Quaterniond(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w) ,
                Eigen::Ma );
        */
        auto &o = p.pose.orientation;
        Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
        auto &pose = p.pose.position;
        Eigen::Translation3d t(pose.x, pose.y, pose.z);
        m = t * q;
    }

    void GraspGenerator::affine3dToPoseMsg(Eigen::Affine3d m, geometry_msgs::PoseStamped &p){
        p.header.frame_id = "map";
        Eigen::Vector3d v = m.translation();
        p.pose.position.x = (float)v(0);
        p.pose.position.y = (float)v(1);
        p.pose.position.z = (float)v(2);
        Eigen::Matrix3d rot = m.rotation() ;
        Eigen::Quaterniond q(rot);
        p.pose.orientation.x = (float)q.x();
        p.pose.orientation.y = (float)q.y();
        p.pose.orientation.z = (float)q.z();
        p.pose.orientation.w = (float)q.w();
    }

    void GraspGenerator::displayAffine3d(Eigen::Affine3d affine){
        std::stringstream ss;
        Eigen::Matrix4d m = affine.matrix();
        ss << "\n"  << m(0,0) << "," << m(0,1) << "," << m(0,2) << "," << m(0,3) << ",\n"
                << m(1,0) << "," << m(1,1) << "," << m(1,2) << "," << m(1,3) << ",\n"
                << m(2,0) << "," << m(2,1) << "," << m(2,2) << "," << m(2,3) << ",\n"
                << m(3,0) << "," << m(3,1) << "," << m(3,2) << "," << m(3,3) << ";\n" ;
        ROS_INFO(ss.str().c_str());
    }

}