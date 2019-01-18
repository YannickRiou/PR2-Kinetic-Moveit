//
// Created by dtrimoul on 11/29/18.
//

#include "uwds_moveit_scene_publisher/UwdsMoveitCollisionObjectPublisher.h"



namespace uwds_moveit_scene_publisher
{
    void UwdsMoveitCollisionObjectPublisher::onInit()
    {
        // ros param init comes here (thresholds etc.)
        pub_ = nh_->advertise<moveit_custom_msgs::CollisionObjectArray>("moveit_objects", 1000);
        uwds::ReconfigurableClient::onInit();
        // Not after due to default configuration that happend in the init ;)

    }

    void UwdsMoveitCollisionObjectPublisher::onChanges(const std::string& world,
                                   const std_msgs::Header& header,
                                   const Invalidations& invalidations)
    {
        std::cout << "test" << std::endl;
        // Here goes the code that is executed on every changes
        moveit_custom_msgs::CollisionObjectArray objects;
        for (const auto& subject_id : invalidations.node_ids_updated)
        {
            uwds_msgs::Node subject = this->worlds()[world].scene().nodes()[subject_id];
            if(subject.type == uwds::MESH)
            {
                //build a new moveit object
                moveit_msgs::CollisionObject object;
                //Modifier le header pour que la frame soit celle de l'object world/name
                std::stringstream frame_id;
                frame_id << world << "/" << subject.name;
                object.header.frame_id = frame_id.str();
                object.id = subject.id;
                for (const auto mesh_id : getNodeMeshes(subject)) {
                    shape_msgs::Mesh mesh;
                    mesh.vertices = meshes()[mesh_id].vertices;
                    //copy des triangles
                    for(auto triangle : mesh.triangles){
                        shape_msgs::MeshTriangle tri;
                        tri.vertex_indices = triangle.vertex_indices;
                    }
                    object.meshes.push_back(mesh);
                    object.mesh_poses.push_back(geometry_msgs::Pose());
                }
                object.operation = object.ADD;
                objects.data.push_back(object);
            }
        }
        //publish the object array
        pub_.publish(objects);

        std::cout << "coucou maman" << std::endl;
    }

    void UwdsMoveitCollisionObjectPublisher::onReconfigure(const std::vector<std::string>& new_input_worlds)
    {
        // Here goes the code that is executed on each reconfiguration

    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_moveit_scene_publisher::UwdsMoveitCollisionObjectPublisher, nodelet::Nodelet)
