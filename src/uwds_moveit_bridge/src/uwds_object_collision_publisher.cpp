#include "uwds_moveit_bridge/uwds_object_collision_publisher.h"

namespace uwds_moveit_bridge
{
  void UwdsObjectCollisionPublisher::onInit()
  {

    pub_ = getNodeHandle().advertise<moveit_custom_msgs::CollisionObjectArray>("uwds_moveit_objects", 1000);
    uwds::ReconfigurableClient::onInit();
    // Not after due to default configuration that happend in the init ;)


  }

  void UwdsObjectCollisionPublisher::onChanges(const std::string& world,
                         const std_msgs::Header& header,
                         const Invalidations& invalidations)
  {
        // Here goes the code that is executed on every changes

    moveit_custom_msgs::CollisionObjectArray objects;
    for (const auto& node : worlds()[world].scene().nodes())
    {

      if(node->type == uwds::MESH)
      {
          //build a new moveit object
          moveit_msgs::CollisionObject object;
          //Modifier le header pour que la frame soit celle de l'object world/name
          std::string name = boost::algorithm::to_lower_copy(node->name);

          object.id = world + "/" + name;
          ROS_INFO(std::string("Building " + world + "/" + name).c_str());

          std::vector<std::string> mesh_ids;
          for(auto property : node->properties)
          {
              if(property.name=="meshes")
              {
                  boost::split(mesh_ids, property.data, boost::is_any_of(","), boost::token_compress_on);
                  break;
              }
          }
          for(auto mesh_id : mesh_ids) {
                  shape_msgs::Mesh mesh;
                  mesh.vertices = meshes()[mesh_id].vertices;
                  //copy des triangles
                  for (const auto &triangle : meshes()[mesh_id].triangles) {
                      shape_msgs::MeshTriangle tri;
                      tri.vertex_indices = triangle.vertex_indices;
                      mesh.triangles.push_back(tri);
                  }

                  object.meshes.push_back(mesh);
                  object.mesh_poses.push_back( node->position.pose );

              object.operation = object.ADD;
              objects.data.push_back(object);
          }
          object.operation = object.ADD;
          objects.data.push_back(object);
          std::stringstream ss;
          ss << "objects size : "  << objects.data.size();
          ROS_INFO(ss.str().c_str());
      }
    }
    //publish the object array
    pub_.publish(objects);
  }

  void UwdsObjectCollisionPublisher::onReconfigure(const std::vector<std::string>& new_input_worlds)
  {
    // Here goes the code that is executed on each reconfiguration
    // reset the qsr traces in the server ?
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_moveit_bridge::UwdsObjectCollisionPublisher, nodelet::Nodelet)
