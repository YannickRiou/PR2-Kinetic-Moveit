//
// Created by dtrimoul on 10/16/18.
//

#include "GraspGenerator.h"


namespace actioncontroller {

    GraspGenerator::GraspGenerator(std::string pathToFile) {
        _pathToFile = pathToFile;
        _providedGrasps = YAML::LoadFile(_pathToFile);
        setOpenGripper();
        setClosedGripper();
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

    const trajectory_msgs::JointTrajectory &GraspGenerator::getOpenGripper() const {
        return _openGripper;
    }

    const trajectory_msgs::JointTrajectory &GraspGenerator::getCloseGripper() const {
        return _closeGripper;
    }

    int GraspGenerator::getProvidedGraspsNumber() {
        return _providedGrasps.size();
    }

}