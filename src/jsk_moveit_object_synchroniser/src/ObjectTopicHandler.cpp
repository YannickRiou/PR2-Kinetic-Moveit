//
// Created by dtrimoul on 1/15/19.
//

#include <jsk_moveit_object_synchroniser/ObjectTopicHandler.h>

//ObjectTopicHandler::ObjectTopicHandler() {}
const bool ObjectTopicHandler::initTopic(){
    _sub = _nh.subscribe(_topicName, 1000, &ObjectTopicHandler::topicCallback, this );
    return _sub.getNumPublishers();
}
const ros::NodeHandle &ObjectTopicHandler::get_nh() const {
    return _nh;
}
const std::string &ObjectTopicHandler::get_topicName() const {
    return _topicName;
}
const std::map<std::string, moveit_msgs::CollisionObject> &ObjectTopicHandler::_get_moveitObjects() const {
    return _moveitObjects;
}

/// Parametrized constructor
/// \param _nh the node handler
/// \param _topicName the name of the jsk bounding box topic
/// \param _moveitObjects the map of the moveit objects that is going to be published
ObjectTopicHandler::ObjectTopicHandler(ros::NodeHandle _nh, std::string _topicName,std::map<std::string, moveit_msgs::CollisionObject> &_moveitObjects) :
        _nh(_nh) ,
        _topicName(_topicName),
        _moveitObjects(_moveitObjects){}

/*
ObjectTopicHandler ObjectTopicHandler::operator()(ros::NodeHandle nh, std::string topicName,
                                                  std::map<std::string, moveit_msgs::CollisionObject> &moveitObjects) {
    return ObjectTopicHandler(nh, topicName, moveitObjects);
}
*/
 /// the callback for the object topic
/// Build a new moveit Collision object and add it to the object collision map
/// \param msg jsk Boundingbox message
void ObjectTopicHandler::topicCallback(const jsk_recognition_msgs::BoundingBox::ConstPtr &msg) {

    std::cout << "msg received on " << _topicName << std::endl;
    std::cout << "_moveitObjects size " << _moveitObjects.size() << std::endl;

    moveit_msgs::CollisionObject object;
    object.id = getObjectName();
    object.meshes.resize(1);
    object.meshes[0] = generateMeshesFromBB(msg.get()->dimensions, msg.get()->pose);
    object.mesh_poses.resize(1);
    object.mesh_poses[0] = msg.get()->pose;
    object.header = msg.get()->header;
    _moveitObjects.emplace(getObjectName(), object);
}

/// remove the namespace from the topic name
/// \return the last element of the topic name that is supposed to be the object name
std::string ObjectTopicHandler::getObjectName() {
    if(_objectName != ""){
        return _objectName;
    }else{
        std::regex removeNamespace("^\\/.*\\/");
        _objectName = std::regex_replace(_topicName, removeNamespace, "");
        return _objectName;
    }
}

/// Build a mesh from the bouding boxe
/// \param dimensions the size of the boxes in x,y,z
/// \param position the pose of the center of the bb
/// \return a mesh
shape_msgs::Mesh ObjectTopicHandler::generateMeshesFromBB(
        geometry_msgs::Vector3 dimensions,
        geometry_msgs::Pose position) {
    shape_msgs::Mesh m;
    addVerticesToMesh(dimensions, position, m);
    addTrianglesToMesh(m);
    return m ;
}

void ObjectTopicHandler::addTrianglesToMesh(shape_msgs::Mesh &m) const {
    m.triangles.resize(12);
    m.triangles[0].vertex_indices = {0,1,3} ;
    m.triangles[1].vertex_indices = {1,2,3} ;
    m.triangles[2].vertex_indices = {2,5,1} ;
    m.triangles[3].vertex_indices = {5,6,2} ;
    m.triangles[4].vertex_indices = {4,5,0} ;
    m.triangles[5].vertex_indices = {5,1,0} ;
    m.triangles[6].vertex_indices = {4,0,7} ;
    m.triangles[7].vertex_indices = {0,3,7} ;
    m.triangles[8].vertex_indices = {3,2,7} ;
    m.triangles[9].vertex_indices = {2,6,7} ;
    m.triangles[10].vertex_indices = {5,4,6} ;
    m.triangles[11].vertex_indices = {4,6,7} ;
}

void
ObjectTopicHandler::addVerticesToMesh(const geometry_msgs::Vector3 &dimensions, const geometry_msgs::Pose &position,
                                      shape_msgs::Mesh &m) {
    m.vertices.resize(8);
    std::cout << dimensions.x << std::endl;
    std::cout << position.position.x << std::endl;
    std::cout << dimensions.y << std::endl;
    std::cout << position.position.y << std::endl;
    std::cout << dimensions.z << std::endl;
    std::cout << position.position.z << std::endl;
    
    m.vertices[0] = generatePoint(
            position.position.x - (dimensions.x / 200),
            position.position.y + (dimensions.y / 200),
            position.position.z + (dimensions.z / 200));
    m.vertices[1] = generatePoint(
            position.position.x - (dimensions.x / 200),
            position.position.y - (dimensions.y / 200),
            position.position.z + (dimensions.z / 200));
    m.vertices[2] = generatePoint(
            position.position.x - (dimensions.x / 200),
            position.position.y - (dimensions.y / 200),
            position.position.z - (dimensions.z / 200));
    m.vertices[3] = generatePoint(
            position.position.x - (dimensions.x / 200),
            position.position.y + (dimensions.y / 200),
            position.position.z - (dimensions.z / 200));
    m.vertices[4] = generatePoint(
            position.position.x + (dimensions.x / 200),
            position.position.y + (dimensions.y / 200),
            position.position.z + (dimensions.z / 200));
    m.vertices[5] = generatePoint(
            position.position.x + (dimensions.x / 200),
            position.position.y - (dimensions.y / 200),
            position.position.z + (dimensions.z / 200));
    m.vertices[6] = generatePoint(
            position.position.x + (dimensions.x / 200),
            position.position.y - (dimensions.y / 200),
            position.position.z - (dimensions.z / 200));
    m.vertices[7] = generatePoint(
            position.position.x + (dimensions.x / 200),
            position.position.y + (dimensions.y / 200),
            position.position.z - (dimensions.z / 200));
}

/// Homemade copy constructor for geometry_msgs point
/// \param x
/// \param y
/// \param z
/// \return
geometry_msgs::Point ObjectTopicHandler::generatePoint(float x, float y, float z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}


