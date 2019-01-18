//
// Created by dtrimoul on 11/29/18.
//

#ifndef UWDS_MOVEIT_SCENE_PUBLISHER_UWDS_MOVEIT_OBJECT_CONVERTER_H
#define UWDS_MOVEIT_SCENE_PUBLISHER_UWDS_MOVEIT_OBJECT_CONVERTER_H

#include <uwds/uwds.h>
#include <uwds/reconfigurable_client.h>
#include "moveit_custom_msgs/CollisionObjectArray.h"

using namespace uwds_msgs;
using namespace uwds;

namespace uwds_moveit_scene_publisher {

    class UwdsMoveitCollisionObjectPublisher : public uwds::ReconfigurableClient  {
    private:
        ros::Publisher pub_;
    public:
        /**@brief
         * The default constructor.
         */
        UwdsMoveitCollisionObjectPublisher(): uwds::ReconfigurableClient(uwds::READER) {}

        /**@brief
         * The default destructor.
         */
        ~UwdsMoveitCollisionObjectPublisher() = default;

        /** @brief
         * Initialize method. Subclass should call this method
         * in its onInit method.
         */
        virtual void onInit();

    protected:
        /** @brief
         * This method is called when there is a change in a world.
         *
         * @param world The world that have been updated
         * @param nodes_id_updated The node IDs that have been updated
         * @param situations_id_updated The situation IDs that have been updated
         */
        virtual void onChanges(const std::string& world,
                               const std_msgs::Header& header,
                               const Invalidations& invalidations);

        virtual void onSubscribeChanges(const std::string world) {}

        virtual void onUnsubscribeChanges(const std::string world) {}

        virtual void onReconfigure(const std::vector<std::string>& worlds);


    };
}



#endif //UWDS_MOVEIT_SCENE_PUBLISHER_UWDS_MOVEIT_OBJECT_CONVERTER_H
