
#include <ActionControllerTools.h>


namespace actioncontroller{

    //refactor code to add a parameter to set the reference frame for visualisation
    ActionControllerTools::ActionControllerTools(){
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/odom_combined","/moveit_visual_markers"));
    }
/*
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
*/
    void ActionControllerTools::callingActionController(std::string part, actioncontroller::ActionControllerGoal &msg){
        actionlib::SimpleActionClient<actioncontroller::ActionControllerAction> ac_("action_controller", true);
        while(!ac_.waitForServer(ros::Duration(5.0))){
            ROS_DEBUG(std::string("Waiting for the action server to come up").c_str());
        }

        ROS_DEBUG(std::string("Sending goal to ActionController").c_str());
        ac_.sendGoal(msg);
        ROS_DEBUG(std::string("wating for result").c_str());

        ac_.waitForResult();
        std::stringstream ss;

        bool success = (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
        if(success)
            ss << "Move " << success << " " << part.c_str();
        else
            ss << "Failed to move " << part.c_str();

        ROS_DEBUG(ss.str().c_str());
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
        ROS_DEBUG(ss.str().c_str());
    }

    void ActionControllerTools::displayAffine3d(Eigen::Affine3d affine){
        std::stringstream ss;
        Eigen::Matrix4d m = affine.matrix();
        ss << "\n"  << m(0,0) << "," << m(0,1) << "," << m(0,2) << "," << m(0,3) << ",\n"
           << m(1,0) << "," << m(1,1) << "," << m(1,2) << "," << m(1,3) << ",\n"
           << m(2,0) << "," << m(2,1) << "," << m(2,2) << "," << m(2,3) << ",\n"
           << m(3,0) << "," << m(3,1) << "," << m(3,2) << "," << m(3,3) << ";\n" ;
        ROS_DEBUG(ss.str().c_str());
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
        ROS_DEBUG(ss.str().c_str());
    }

    void ActionControllerTools::poseMsgToAffine3d(geometry_msgs::PoseStamped &p, Eigen::Affine3d &m){
        /* m = Eigen::Affine3d::fromPositionOrientationScale(Eigen::Vector3d(p.pose.position.x, p.pose.position.y, p.pose.position.z),
                Eigen::Quaterniond(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w) ,
                Eigen::Ma );
        */
        auto &o = p.pose.orientation;
        Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
        auto &pose = p.pose.position;
        Eigen::Translation3d t(pose.x, pose.y, pose.z);
        m = t * q;
    }

    void ActionControllerTools::affine3dToPoseMsg(Eigen::Affine3d m, geometry_msgs::PoseStamped &p){
        p.header.frame_id = "odom_combined";
        Eigen::Vector3d v = m.translation();
        p.pose.position.x = (float)v(0);
        p.pose.position.y = (float)v(1);
        p.pose.position.z = (float)v(2);
        Eigen::Matrix3d rot = m.rotation() ;
        Eigen::Quaterniond q(rot);
        p.pose.orientation.x = (float)q.x();
        p.pose.orientation.y = (float)q.y();
        p.pose.orientation.z = (float)q.z();
        p.pose.orientation.w = (float)q.w();
    }

    Eigen::Affine3d ActionControllerTools::affine3dFromAngleAxis(double radianX, double radianY, double radianZ) const {
        Eigen::Affine3d orientationFrameRotation;
        orientationFrameRotation = Eigen::AngleAxisd(radianX, Eigen::Vector3d::UnitX())
                                   * Eigen::AngleAxisd(radianY, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(radianZ, Eigen::Vector3d::UnitZ());
        return orientationFrameRotation;
    }

    void ActionControllerTools::posePopulator(geometry_msgs::Pose &p, float x, float y, float z, float ox, float oy, float oz, float ow) {
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        p.orientation.x = ox;
        p.orientation.y = oy;
        p.orientation.z = oz;
        p.orientation.w = ow;
    }


}