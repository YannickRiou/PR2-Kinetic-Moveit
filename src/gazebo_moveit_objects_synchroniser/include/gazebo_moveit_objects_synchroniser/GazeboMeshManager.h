//
// Created by dtrimoul on 8/20/18.
//

#ifndef GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOMESHMANAGER_H
#define GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOMESHMANAGER_H

#include "geometric_shapes/shape_operations.h"
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include <regex>

namespace GazeboMeshManager{

    class IoException : public std::exception{
        private:
            std::string file_name;

        public:
            explicit IoException(std::string file);
            const char * what () const noexcept;

    };

    class GazeboMeshManager {
    private:
        std::string pathToGazeboModels;
    public:
        GazeboMeshManager();

        shape_msgs::Mesh getMesh(std::string basic_string);

        std::string cleanModelName(std::string basic_string);
    };
}




#endif //GAZEBO_MOVEIT_OBJECTS_SYNCHRONISER_GAZEBOMESHMANAGER_H
