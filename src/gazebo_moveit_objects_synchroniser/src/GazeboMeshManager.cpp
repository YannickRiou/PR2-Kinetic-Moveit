//
// Created by dtrimoul on 8/20/18.
//

#include "gazebo_moveit_objects_synchroniser/GazeboMeshManager.h"

GazeboMeshManager::GazeboMeshManager() {
    pathToGazeboModels =  std::string(getenv("HOME")) + "/.gazebo/models/" ;
}

shape_msgs::Mesh GazeboMeshManager::getMesh(std::string model_name) {
    std::string clean_model_name = cleanModelName(model_name);
    std::string full_path = pathToGazeboModels + clean_model_name + "/meshes/" + clean_model_name + ".dae";
    ROS_INFO("trying to build : %s", clean_model_name.c_str());

    if(boost::filesystem::exists(full_path)){
        shapes::Mesh *m = shapes::createMeshFromResource("file://" + full_path );
        shape_msgs::Mesh mesh;
        shapes::ShapeMsg mesh_msg = mesh;
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        ROS_INFO("Build suceed");
        return mesh;
    }else{
        throw "Unable to locate the file for " + model_name + " model";
    }
}

std::string GazeboMeshManager::cleanModelName(std::string model_name) {
    std::regex number_regex("_[0-9]*$");
    return std::regex_replace(model_name, number_regex, "$1");
}
