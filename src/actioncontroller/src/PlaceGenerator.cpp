//
// Created by dtrimoul on 10/11/18.
//

#include <PlaceGenerator.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "PlaceGenerator.h"

namespace actioncontroller{

    PlaceGenerator::PlaceGenerator(std::string path, moveit_msgs::CollisionObject obj, double topObjectHeight, int samples) : _graspGen(path) {
        _samples = samples;
        _object = obj;
        _topObjectHeight = topObjectHeight;
    }

    std::vector<geometry_msgs::PoseStamped> PlaceGenerator::samplePossiblePlaceLocation() {
        std::vector<geometry_msgs::PoseStamped> poses;

        double maxX, minX, maxY, minY = 0;
        for (int i = 0; i < _topVertices.size() ; ++i) {
            if(_topVertices[i].x > maxX){
                maxX = _topVertices[i].x;
            }else{
                if(_topVertices[i].x < minX){
                    minX = _topVertices[i].x;
                }
            }
            if(_topVertices[i].y > maxY){
                maxY = _topVertices[i].y;
            }else{
                if(_topVertices[i].y < minY){
                    minY = _topVertices[i].y;
                }
            }
        }

        std::uniform_real_distribution<double> unif(0, 10000);
        std::default_random_engine randomDouble;

        for (int i = 0; i < _samples ; ++i) {
            ROS_INFO(std::string("Generating point for placing").c_str());
            geometry_msgs::PoseStamped p;
            p.header.frame_id = _object.header.frame_id;
            p.pose.orientation.x = 0;
            p.pose.orientation.y = 0;
            p.pose.orientation.z = 0;
            p.pose.orientation.w = 1;
            p.pose.position.x = minX +( std::fmod( unif(randomDouble) , maxX ) );
            p.pose.position.y = minY +( std::fmod( unif(randomDouble) , maxY ) );
            p.pose.position.z =  _topVertice.z + ( _topObjectHeight / 2 );
            tools.displayPoseStampedMsg(p);
        }

        return poses;
    }

    void PlaceGenerator::setTopVertices() {

        if(!_object.meshes.empty()){
            std::stringstream ss;
            ss << "number of meshes: " << _object.meshes.size() ;
            ROS_INFO(ss.str().c_str());
            _topVertice = _object.meshes[0].vertices[0];
            tools.displayPoint(_topVertice);
            for (int i = 0; i < _object.meshes.size() ; ++i) {
                ss.clear();
                ss << "number of vertice in mesh " << i << " : " << _object.meshes[i].vertices.size();
                ROS_INFO(ss.str().c_str());
                for (int j = 0; j < _object.meshes[i].vertices.size() ; ++j) {
                    if(_object.meshes[i].vertices[j].z > _topVertice.z){
                        ROS_INFO(std::string("new Top vertex").c_str());
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
                tools.displayPoint( _topVertices[k] );
            }
        }else{
            throw NoMeshInObjectExeption(std::string(_object.id));
        }

    }

    std::vector<geometry_msgs::Point> PlaceGenerator::getTopVertices() {
        if(_topVertices.size() == 0){
            setTopVertices();
        }
        return _topVertices;
    }

    std::vector<geometry_msgs::PoseStamped> PlaceGenerator::getPossibleLocations(){
        if(_possibleLocations.empty()){
            //Il faudra considérer que l'on peut avoir une orientation inversée. On va partir du principe que l'object fournit est dans un référentiel orienté correctement.
            try{
                ROS_INFO(std::string("getting top vertices").c_str());
                setTopVertices();
                ROS_INFO(std::string("getting convex Hull").c_str());
                generateTopConvexHull();
                ROS_INFO(std::string("getting top vertices").c_str());
                samplePossiblePlaceLocation();
            }
            catch(std::exception e){
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
            tools.displayPoint(_topConvexHull[j]);
        }
    }

    std::vector<moveit_msgs::PlaceLocation> PlaceGenerator::generatePlaces() {
        std::vector<moveit_msgs::PlaceLocation> locations;
        ROS_INFO("creating place");
        std::stringstream ss;
        ss << "Grasp Number " <<  _graspGen.getProvidedGraspsNumber();
        ROS_INFO(ss.str().c_str());
        std::vector<geometry_msgs::PoseStamped> targets = getPossibleLocations();
        for(geometry_msgs::PoseStamped target : targets){
            for (unsigned i = 0; i < _graspGen.getProvidedGraspsNumber() ; ++i) {
                moveit_msgs::PlaceLocation pl;
                pl.place_pose = target;
                pl.pre_place_approach = _graspGen.generateGraspMove(i, "pre");
                pl.post_place_retreat = _graspGen.generateGraspMove(i, "post");
                pl.post_place_posture = _graspGen.getOpenGripper();
                locations.push_back(pl);
            }
        }
        return locations;
    }

    NoflatSurfaceExeption::NoflatSurfaceExeption(std::string object_name) {
        _object_name = object_name;
    }

    const char * NoflatSurfaceExeption::what () const noexcept{
        std::stringstream ss;
        ss << _object_name << "has no flat surface !" << std::endl;
        return ss.str().c_str();
    }

    NoMeshInObjectExeption::NoMeshInObjectExeption(const std::string object_name)  {
        _object_name = object_name;
    }

    const char * NoMeshInObjectExeption::what () const noexcept{
        std::stringstream ss;
        ss << _object_name << "has an empty mesh !" << std::endl;
        return ss.str().c_str();
    }
}