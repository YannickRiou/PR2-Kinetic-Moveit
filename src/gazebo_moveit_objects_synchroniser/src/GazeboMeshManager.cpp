//
// Created by dtrimoul on 8/20/18.
//

#include "gazebo_moveit_objects_synchroniser/GazeboMeshManager.h"

namespace GazeboMeshManager{

    IoException::IoException(std::string file) {
        file_name = file;
    }

    const char * IoException::what () const noexcept
    {
        std::stringstream ss;
        ss << "file: " << file_name << " not found !" << std::endl;
        return ss.str().c_str();
    }

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
            ROS_INFO("Build succeed");
            return mesh;
        }else{
            throw IoException(full_path);
        }
    }

    std::string GazeboMeshManager::cleanModelName(std::string model_name) {
        std::regex number_regex("_[0-9]*$");
        return std::regex_replace(model_name, number_regex, "$1");
    }


}
