#include "ActionControllerTools.h"

namespace actioncontroller{

            void ActionControllerTools::createActionControllerMessage(std::string group_id, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &goal){
                goal.goal.action = group_id;
                goal.goal.pose.position.x = x;
                goal.goal.pose.position.y = y;
                goal.goal.pose.position.z = z;
                goal.goal.pose.orientation.x = ox;
                goal.goal.pose.orientation.y = oy;
                goal.goal.pose.orientation.z = oz;
                goal.goal.pose.orientation.w = w;
            }

            void ActionControllerTools::createActionControllerMessage(std::string group_id, std::string object, actioncontroller::ActionControllerGoal &goal){
                goal.goal.action = group_id;
                goal.goal.object = object;
            }

            void ActionControllerTools::createActionControllerMessage(std::string group_id, std::string object, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &goal){
                goal.goal.action = group_id;
                goal.goal.pose.position.x = x;
                goal.goal.pose.position.y = y;
                goal.goal.pose.position.z = z;
                goal.goal.pose.orientation.x = ox;
                goal.goal.pose.orientation.y = oy;
                goal.goal.pose.orientation.z = oz;
                goal.goal.pose.orientation.w = w;
                goal.goal.object = object;
            }

            void ActionControllerTools::callingActionController(std::string part, actioncontroller::ActionControllerGoal &goal){
                actionlib::SimpleActionClient<actioncontroller::ActionControllerAction> ac_("action_controller", true);
                while(!ac_.waitForServer(ros::Duration(5.0))){
                        ROS_INFO("Waiting for the action server to come up");
                    }

                ROS_INFO("Sending goal to ActionController");
                ac_.sendGoal(goal);
                ROS_INFO("wating for result");

                ac_.waitForResult();

                bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
                if(success)
                    ROS_INFO("Move %s success", part.c_str());
                else
                    ROS_INFO("Failed to move %s", part.c_str());
            }
}