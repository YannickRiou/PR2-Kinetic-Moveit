//
// Created by dtrimoul on 10/11/18.
//

#ifndef ACTIONCONTROLLER_PLACEGENERATOR_H
#define ACTIONCONTROLLER_PLACEGENERATOR_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "moveit_msgs/CollisionObject.h"


namespace actioncontroller{

    class PlaceGenerator {
        private:
            moveit_msgs::CollisionObject _object;
            std::vector<geometry_msgs::Point> _topConvexHull;
            geometry_msgs::Point _topVertice;
            std::vector<geometry_msgs::Point> _topVertices;
            std::vector<geometry_msgs::PoseStamped> _possibleLocations;
    public:
            std::vector<geometry_msgs::PoseStamped> get_possibleLocations(int samples);
            PlaceGenerator(moveit_msgs::CollisionObject obj);
            std::vector<geometry_msgs::Point> getTopVertices();
            void generateTopConvexHull();
            std::vector<geometry_msgs::PoseStamped> samplePossiblePlaceLocation(int samples);
            void setTopVertice();
            double crossProductXY(geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C );
    };


    class NoflatSurfaceExeption : public std::exception{
    private:
        std::string _object_name;

    public:
        explicit NoflatSurfaceExeption(std::string object_name);
        const char * what () const noexcept;

    };
}

#endif //ACTIONCONTROLLER_PLACEGENERATOR_H
