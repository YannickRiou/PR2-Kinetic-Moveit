//
// Created by dtrimoul on 10/11/18.
//

#include <PlaceGenerator.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "PlaceGenerator.h"

namespace actioncontroller{

    PlaceGenerator::PlaceGenerator(moveit_msgs::CollisionObject obj) {
        _object = obj;
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/odom_combined","/moveit_visual_markers"));
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
        for (int k = 0; k < _topVertices.size() ; ++k) {
            displayPoint( _topVertices[k] );
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
        ROS_INFO("Convex Hull");
        for (int j = 0; j < _topConvexHull.size() ; ++j) {
            displayPoint(_topConvexHull[j]);
        }
    }

    NoflatSurfaceExeption::NoflatSurfaceExeption(std::string object_name) {
        _object_name = object_name;
    }

    const char * NoflatSurfaceExeption::what () const noexcept{
        std::stringstream ss;
        ss << _object_name << "has no flat surface !" << std::endl;
        return ss.str().c_str();
    }

    void PlaceGenerator::displayPoint(const geometry_msgs::Point &p){
        geometry_msgs::PoseStamped ps;
        ps.pose.position = p;
        ps.header.frame_id = "/odom_combined";
        ps.pose.orientation.x = 0;
        ps.pose.orientation.y = 0;
        ps.pose.orientation.z = 0;
        ps.pose.orientation.w = 1;
        visual_tools_.get()->publishCuboid(ps.pose, 0.01, 0.01, 0.01, rviz_visual_tools::colors::LIME_GREEN );
        visual_tools_.get()->trigger();
        std::stringstream ss;
        ss << "Point:\n x:" << p.x << "\n y:" << p.y << "\n z:" << p.z << std::endl ;
        ROS_INFO(ss.str().c_str());
    }

}