#ifndef UWDS_OBJECT_COLLISION_PUBLISHER_HPP
#define UWDS_OBJECT_COLLISION_PUBLISHER_HPP

#include <uwds/uwds.h>
#include <uwds/reconfigurable_client.h>
#include "moveit_custom_msgs/CollisionObjectArray.h"

using namespace uwds_msgs;
using namespace uwds;

namespace uwds_moveit_bridge
{
  class UwdsObjectCollisionPublisher : public uwds::ReconfigurableClient
  {
  private:
      ros::Publisher pub_;
  public:
    /**@brief
     * The default constructor.
     */
    UwdsObjectCollisionPublisher(): uwds::ReconfigurableClient(uwds::FILTER) {}

    /**@brief
     * The default destructor.
     */
    ~UwdsObjectCollisionPublisher() = default;

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
     * @param header The header
     * @param invalidations The invalidations received
     */
    virtual void onChanges(const std::string& world,
                           const std_msgs::Header& header,
                           const Invalidations& invalidations);

    virtual void onSubscribeChanges(const std::string world) {}

    virtual void onUnsubscribeChanges(const std::string world) {}

    virtual void onReconfigure(const std::vector<std::string>& worlds);

  };
}

#endif
