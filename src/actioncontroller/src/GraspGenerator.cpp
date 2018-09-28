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
            p.pose.position = _target.pose.position;
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

}