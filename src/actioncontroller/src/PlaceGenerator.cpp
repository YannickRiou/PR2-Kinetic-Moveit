//
// Created by dtrimoul on 10/11/18.
//

#include <PlaceGenerator.h>

#include "PlaceGenerator.h"

namespace actioncontroller{

    PlaceGenerator::PlaceGenerator(moveit_msgs::CollisionObject obj) {
        _object = obj;
    }

    std::vector<geometry_msgs::PoseStamped> PlaceGenerator::samplePossiblePlaceLocation(int samples) {
        return std::vector<geometry_msgs::PoseStamped>();
    }

    void PlaceGenerator::setTopVertice() {
        for (int i = 0; i < _object.meshes.size() ; ++i) {
            for (int j = 0; j < _object.meshes[i].vertices.size() ; ++j) {
                if(_object.meshes[i].vertices[j].z > _topVertice.z){
                    _topVertices.clear();
                    _topVertice = _object.meshes[i].vertices[j];
                    _topVertices.push_back(_object.meshes[i].vertices[j]);

                }else if(_object.meshes[i].vertices[j].z == _topVertice.z){
                    _topVertices.push_back(_object.meshes[i].vertices[j]);
                }
            }
        }
        //s'il n'y a pas assez de vertices pour faire une surface
        if(_topVertices.size() < 3){
            throw NoflatSurfaceExeption(std::string(_object.id));
        }
    }

    std::vector<geometry_msgs::Point> PlaceGenerator::getTopVertices() {
        if(_topVertices.size() == 0){
            setTopVertice();
        }
        return _topVertices;
    }

    std::vector<geometry_msgs::PoseStamped> PlaceGenerator::get_possibleLocations(int samples){
        if(_possibleLocations.size() == 0){
            //Il faudra considérer que l'on peut avoir une orientation inversée. On va partir du principe que l'object fournit est dans un référentiel orienté correctement.
            try{
                setTopVertice();
                generateTopConvexHull();
                samplePossiblePlaceLocation(samples);
            }
            catch(NoflatSurfaceExeption e){
                ROS_INFO(e.what());
            }
        }
    }

    double PlaceGenerator::crossProductXY(geometry_msgs::Point O, geometry_msgs::Point A, geometry_msgs::Point B ){
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }


    void PlaceGenerator::generateTopConvexHull() {
        std::sort(_topVertices.begin(), _topVertices.end(), [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
            return (p1.x < p2.x || ( p1.x == p2.x && p1.y < p2.y));
        });

        int k=0;
        _topConvexHull.resize(_topVertices.size() * 2);

        for (int i = 0; i < _topVertices.size() ; ++i) {
            while (k >= 2 && crossProductXY(_topConvexHull[k-2], _topConvexHull[k-1], _topVertices[i]) <= 0) k--;
            _topConvexHull[k++] = _topVertices[i];
        }

        for (int i = _topVertices.size()-1, t = k+1; i > 0; --i) {
            while (k >= t && crossProductXY(_topConvexHull[k-2], _topConvexHull[k-1], _topVertices[i-1]) <= 0) k--;
            _topConvexHull[k++] = _topVertices[i-1];
        }

        _topConvexHull.resize(k-1);
    }




    NoflatSurfaceExeption::NoflatSurfaceExeption(std::string object_name) {
        _object_name = object_name;
    }

    const char * NoflatSurfaceExeption::what () const noexcept
    {
        std::stringstream ss;
        ss << _object_name << "has no flat surface !" << std::endl;
        return ss.str().c_str();
    }

}