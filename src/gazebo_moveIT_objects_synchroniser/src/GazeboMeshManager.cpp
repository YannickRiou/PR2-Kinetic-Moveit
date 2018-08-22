//
// Created by dtrimoul on 8/20/18.
//

#include "gazebo_moveIT_objects_synchroniser/GazeboMeshManager.h"

GazeboMeshManager::GazeboMeshManager() {
    pathToGazeboModels =  std::string(getenv("HOME")) + "/.gazebo/models/" ;
}

shape_msgs::Mesh GazeboMeshManager::getMesh(std::string model_name) {
    std::string full_path = pathToGazeboModels + model_name + "/meshes/" + model_name + ".dae";
    ROS_INFO("trying to build : %s", model_name.c_str());

    if(boost::filesystem::exists(full_path)){
        shapes::Mesh *m = shapes::createMeshFromResource("file://" + full_path );
        shape_msgs::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        return mesh;
    }else{
        throw "Unable to locate the file for " + model_name + " model";
    }
}
