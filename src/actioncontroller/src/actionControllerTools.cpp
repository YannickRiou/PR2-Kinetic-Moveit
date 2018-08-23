

class ActionControllerTools{
	
	private:

	public:
		void createActionControllerMessage(std::string group_id, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &goal){	
			goal.goal.move_group_id = group_id;
			goal.goal.pose.pose.position.x = x;
			goal.goal.pose.pose.position.y = y;
			goal.goal.pose.pose.position.z = z;
			goal.goal.pose.pose.orientation.x = ox;
			goal.goal.pose.pose.orientation.y = oy;
			goal.goal.pose.pose.orientation.z = oz;
			goal.goal.pose.pose.orientation.w = w;
		}

		void callingActionController(std::string part, actioncontroller::ActionControllerGoal &goal){
			actionlib::SimpleActionClient<actioncontroller::ActionControllerAction> ac_("ActionController", true);
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