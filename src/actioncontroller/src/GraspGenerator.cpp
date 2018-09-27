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

    const std::string &GraspGenerator::getPathToFile() const {
        return _pathToFile;
    }

    void GraspGenerator::setPathToFile(const std::string &pathToFile) {
        _pathToFile = pathToFile;
    }

    std::vector<moveit_msgs::Grasp> GraspGenerator::generateGrasp(){
        std::vector<moveit_msgs::Grasp> grasps;
        for (unsigned i = 0; i < _providedGrasps["grasp"].size() ; ++i) {
            moveit_msgs::Grasp g;
            g.grasp_pose = _target;
            g.pre_grasp_approach = generateGraspMove(i, "pre");
            g.post_grasp_retreat = generateGraspMove(i, "post");
            g.pre_grasp_posture = _openGripper;
            g.grasp_posture = _closeGripper;

        }
        return grasps;
    }

    moveit_msgs::GripperTranslation GraspGenerator::generateGraspMove(int graspNumber, std::string moveType) {
        moveit_msgs::GripperTranslation j;
         j.direction.header.frame_id = _providedGrasps[graspNumber][moveType]["direction"]["header_frame_id"].as<std::string>();
         j.direction.vector.x = _providedGrasps[graspNumber][moveType]["direction"]["vector_x"].as<float>() ;
         j.direction.vector.y = _providedGrasps[graspNumber][moveType]["direction"]["vector_y"].as<float>() ;
         j.direction.vector.z  = _providedGrasps[graspNumber][moveType]["direction"]["vector_z"].as<float>() ;
         j.min_distance = _providedGrasps[graspNumber][moveType]["direction"]["min_distance"].as<float>() ;
         j.desired_distance = _providedGrasps[graspNumber][moveType]["direction"]["desired_distance"].as<float>() ;
        return j;
    }

    void GraspGenerator::setOpenGripper(){
        _openGripper.joint_names.resize(_providedGrasps["open"]["joint"].size());
        for(unsigned i=0; _providedGrasps["open"]["joint"].size(); ++i){
            _openGripper.joint_names[i] =  _providedGrasps["open"]["joint"][i].as<std::string>() ;
        }
        _openGripper.points.resize(_providedGrasps["open"]["joint"].size());
        for(unsigned i=0; _providedGrasps["open"]["position"].size(); ++i){
            _openGripper.points[0].positions[i] = _providedGrasps["open"]["position"][i].as<float>();
        }
        ros::Duration d(_providedGrasps["open"]["time_from_start"].as<int>());
        _openGripper.points[0].time_from_start =  d ;
    }

    void GraspGenerator::setClosedGripper(){
        _closeGripper.joint_names.resize(_providedGrasps["close"]["joint"].size());
        for(unsigned i=0; _providedGrasps["close"]["joint"].size(); ++i){
            _closeGripper.joint_names[i] = _providedGrasps["close"]["joint"][i].as<std::string>()  ;
        }
        _closeGripper.points.resize(_providedGrasps["close"]["joint"].size());
        for(unsigned i=0; _providedGrasps["close"]["position"].size(); ++i){
            _closeGripper.points[0].positions[i] = _providedGrasps["close"]["position"][i].as<float>();
        }
        ros::Duration d(_providedGrasps["close"]["time_from_start"].as<int>());
        _closeGripper.points[0].time_from_start = d;
    }

}