//
// Created by dtrimoul on 8/20/18.
//

#include "gazebo_moveIT_objects_synchroniser/GazeboMeshManager.h"

GazeboMeshManager::GazeboMeshManager() {
    pathToGazeboModels =  std::string(getenv("HOME")) + "/.gazebo/models/" ;
}

std::shared_ptr<shapes::Mesh>  GazeboMeshManager::getMesh(std::string model_name) {
    std::string full_path = pathToGazeboModels + model_name + "/meshes/" + model_name + ".dae";
    ROS_INFO("trying to build : %s", model_name.c_str());

    if(boost::filesystem::exists(full_path)){
        std::shared_ptr<shapes::Mesh> m(shapes::createMeshFromResource("file://" + full_path ));
        return m;
    }else{
        throw "Unable to locate the file for " + model_name + " model";
    }
}
