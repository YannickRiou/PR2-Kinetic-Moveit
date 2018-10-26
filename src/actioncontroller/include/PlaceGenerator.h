//
// Created by dtrimoul on 10/11/18.
//

#ifndef ACTIONCONTROLLER_PLACEGENERATOR_H
#define ACTIONCONTROLLER_PLACEGENERATOR_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "moveit_msgs/CollisionObject.h"
#include "ActionControllerTools.h"
#include "moveit_msgs/PlaceLocation.h"
#include "GraspGenerator.h"


namespace actioncontroller{

    class PlaceGenerator {
        private:

            actioncontroller::GraspGenerator _graspGen;
            int _samples;
            double _topObjectHeight;
            actioncontroller::ActionControllerTools tools;
            moveit_msgs::CollisionObject _object;
            std::vector<shape_msgs::Mesh> _objectMeshesInObjectFrame;
            std::vector<geometry_msgs::Point> _topConvexHull;
            geometry_msgs::Point _topVertice;
            std::vector<geometry_msgs::Point> _topVertices;
            std::vector<geometry_msgs::PoseStamped> _possibleLocations;
            void convertMeshPoinToReferenceFrame();

        public:
            std::vector<geometry_msgs::PoseStamped> getPossibleLocations();
            PlaceGenerator( std::string path, moveit_msgs::CollisionObject obj , double topObjectHeight, int samples );
            std::vector<geometry_msgs::Point> getTopVertices();
            void generateTopConvexHull();
            std::vector<geometry_msgs::PoseStamped> samplePossiblePlaceLocation();
            void setTopVertices();
            double crossProductXY(geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C );
            std::vector<moveit_msgs::PlaceLocation> generatePlaceLocations();

    };


    class NoflatSurfaceExeption : public std::exception{
    private:
        std::string _object_name;

    public:
        explicit NoflatSurfaceExeption(std::string object_name);
        const char * what () const noexcept;

    };

    class NoMeshInObjectExeption : public std::exception{
    private:
        std::string _object_name;

    public:
        explicit NoMeshInObjectExeption(std::string object_name);
        const char * what () const noexcept;

    };
}

#endif //ACTIONCONTROLLER_PLACEGENERATOR_H
