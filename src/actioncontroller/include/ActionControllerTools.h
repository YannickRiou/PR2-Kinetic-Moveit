#ifndef ACTION_CONTROLLER_TOOLS
#define ACTION_CONTROLLER_TOOLS

#include <string>
#include <actioncontroller/ActionControllerAction.h>
#include <actionlib/client/simple_action_client.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "moveit_visual_tools/moveit_visual_tools.h"

namespace actioncontroller{

    class ActionControllerTools{

    private:
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    public:
        ActionControllerTools();

        void posePopulator(geometry_msgs::Pose &p, float x, float y, float z, float ox, float oy, float oz, float ow);
/*
        void createActionControllerMessage(std::string group_id, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &msg);

        void createActionControllerMessage(std::string group_id, std::string object, actioncontroller::ActionControllerGoal &msg);

        void createActionControllerMessage(std::string group_id, std::string objectA, std::string objectB, actioncontroller::ActionControllerGoal &msg);

        void createActionControllerMessage(std::string group_id, std::string object, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &msg);
*/
        void callingActionController(std::string part, actioncontroller::ActionControllerGoal &msg);

        void displayAffine3d(Eigen::Affine3d m);

        void displayPoseStampedMsg(geometry_msgs::PoseStamped p);

        void displayPoint(const geometry_msgs::Point &p);

        void poseMsgToAffine3d(geometry_msgs::PoseStamped &p, Eigen::Affine3d &m);

        void affine3dToPoseMsg(Eigen::Affine3d m, geometry_msgs::PoseStamped &p);

        Eigen::Affine3d affine3dFromAngleAxis(double radianX, double radianY, double radianZ) const;
    };
}

#endif
