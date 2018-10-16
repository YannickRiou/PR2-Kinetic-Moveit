
#include <ActionControllerTools.h>

#include "ActionControllerTools.h"



namespace actioncontroller{

            //refactor code to add a parameter to set the reference frame for visualisation
            ActionControllerTools::ActionControllerTools(){
                visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/odom_combined","/moveit_visual_markers"));
            }

            void ActionControllerTools::createActionControllerMessage(std::string group_id, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &msg){
                msg.goal.action = group_id;
                msg.goal.pose.position.x = x;
                msg.goal.pose.position.y = y;
                msg.goal.pose.position.z = z;
                msg.goal.pose.orientation.x = ox;
                msg.goal.pose.orientation.y = oy;
                msg.goal.pose.orientation.z = oz;
                msg.goal.pose.orientation.w = w;
            }

            void ActionControllerTools::createActionControllerMessage(std::string group_id, std::string object, actioncontroller::ActionControllerGoal &msg){
                msg.goal.action = group_id;
                msg.goal.objectA = object;
            }

            void
            ActionControllerTools::createActionControllerMessage(std::string group_id, std::string objectA, std::string objectB,
                                                                 actioncontroller::ActionControllerGoal &msg) {
                createActionControllerMessage(group_id, objectA, msg);
                msg.goal.objectB = objectB;
            }

            void ActionControllerTools::createActionControllerMessage(std::string group_id, std::string object, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &msg){
                msg.goal.action = group_id;
                msg.goal.pose.position.x = x;
                msg.goal.pose.position.y = y;
                msg.goal.pose.position.z = z;
                msg.goal.pose.orientation.x = ox;
                msg.goal.pose.orientation.y = oy;
                msg.goal.pose.orientation.z = oz;
                msg.goal.pose.orientation.w = w;
                msg.goal.objectA = object;
            }

            void ActionControllerTools::callingActionController(std::string part, actioncontroller::ActionControllerGoal &msg){
                actionlib::SimpleActionClient<actioncontroller::ActionControllerAction> ac_("action_controller", true);
                while(!ac_.waitForServer(ros::Duration(5.0))){
                        ROS_INFO("Waiting for the action server to come up");
                    }

                ROS_INFO("Sending goal to ActionController");
                ac_.sendGoal(msg);
                ROS_INFO("wating for result");

                ac_.waitForResult();

                bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
                if(success)
                    ROS_INFO("Move %s success", part.c_str());
                else
                    ROS_INFO("Failed to move %s", part.c_str());
            }

            void ActionControllerTools::displayPoseStampedMsg(geometry_msgs::PoseStamped p){
                std::stringstream ss;
                ss << "header: " << p.header.frame_id
                   << "\npose_x : " << p.pose.position.x
                   << "\npose_y : " << p.pose.position.y
                   << "\npose_z : " << p.pose.position.z
                   << "\norientation_x : " << p.pose.orientation.x
                   << "\norientation_Y : " << p.pose.orientation.y
                   << "\norientation_Z : " << p.pose.orientation.z
                   << "\norientation_W : " << p.pose.orientation.w;
                visual_tools_.get()->publishCuboid(p.pose, 0.01, 0.01, 0.01, rviz_visual_tools::BLUE );
                visual_tools_.get()->trigger();
                ROS_INFO(ss.str().c_str());
            }

            void ActionControllerTools::displayAffine3d(Eigen::Affine3d affine){
                std::stringstream ss;
                Eigen::Matrix4d m = affine.matrix();
                ss << "\n"  << m(0,0) << "," << m(0,1) << "," << m(0,2) << "," << m(0,3) << ",\n"
                   << m(1,0) << "," << m(1,1) << "," << m(1,2) << "," << m(1,3) << ",\n"
                   << m(2,0) << "," << m(2,1) << "," << m(2,2) << "," << m(2,3) << ",\n"
                   << m(3,0) << "," << m(3,1) << "," << m(3,2) << "," << m(3,3) << ";\n" ;
                ROS_INFO(ss.str().c_str());
            }

            void ActionControllerTools::displayPoint(const geometry_msgs::Point &p){
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