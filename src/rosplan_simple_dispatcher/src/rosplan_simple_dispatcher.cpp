//
// Created by dtrimoul on 11/9/18.
//


#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actioncontroller/ActionControllerAction.h>
#include "std_msgs/String.h"
#include <regex>
#include <mutex>
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"

std::vector<std::string> _plan;
bool isChanged = false;
std::mutex mutex;


std::vector<std::string> generatePlanFromMsg( std::string msg){
    std::vector<std::string> plan;
    std::istringstream ss(msg);
    std::smatch sm;
    std::string line;
    while(std::getline(ss, line, '\n')){
        std::regex action("\\((.*)\\)");
        std::regex_search(line,sm, action);
        plan.push_back( sm[1] ) ;
    }
    for (int i = 0; i < plan.size() ; ++i) {
        std::cout << plan[i] << std::endl;
    }
    return plan;
}

void new_plan_callback(const std_msgs::String::ConstPtr& msg){
    isChanged = true;
    std::lock_guard<std::mutex> lock(mutex);
    _plan = generatePlanFromMsg(std::string(msg->data));
}

int main(int argc, char **argv){

    ros::init(argc, argv, "rosplan_simple_dispatcher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/rosplan_planner_interface/planner_output", 1000, new_plan_callback);
    actionlib::SimpleActionClient<actioncontroller::ActionControllerAction> ac("action_controller", true);
    ros::Rate loop_rate(10);



    //ac.waitForServer();
    int planStep;
    while(ros::ok()){
        if(isChanged){
            planStep = 0;
            //cancel the current action in the
            std::cout << "cancelled" << std::endl;
            //kill the previous plan thread
            std::cout << "new Plan" << std::endl;

            //load new domain.
            ros::ServiceClient client = n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>( "/rosplan_knowledge_base/domain/operator_details");
            rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;


            isChanged = false;

        }

        if(planStep < _plan.size()){
            actioncontroller::ActionControllerGoal msg;
            msg.data = _plan[planStep];
            std::stringstream ss;
            ss << "calling action controller to execute : " << _plan[planStep] ;



            ROS_INFO(ss.str().c_str());
            ac.sendGoal(msg);
            ac.waitForResult();

            bool success = (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

            if(success){
                ROS_INFO("Success");
                planStep++;
                //mise à jours des prédicats et des faits.
            }else
                ROS_INFO("FAIL");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}