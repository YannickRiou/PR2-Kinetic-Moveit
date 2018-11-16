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
#include "rosplan_simple_dispatcher/RPAction.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"

rosplan_dispatch_msgs::CompletePlan _plan;
bool isChanged = false;
std::mutex mutex;
std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates;
rosplan_knowledge_msgs::DomainFormula params;
rosplan_knowledge_msgs::DomainOperator op;
ros::Publisher pddl_action_parameters_pub;


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

void new_plan_callback(const rosplan_dispatch_msgs::CompletePlan plan){
    isChanged = true;
    std::lock_guard<std::mutex> lock(mutex);
    _plan = plan;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "rosplan_simple_dispatcher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/rosplan_parsing_interface/complete_plan", 1000, new_plan_callback);
    ros::Rate loop_rate(10);

    int planStep;
    while(ros::ok()){

        if(isChanged){
            planStep = 0;
            //cancel the current action in the
            std::cout << "cancelled" << std::endl;
            //kill the previous plan thread
            std::cout << "new Plan" << std::endl;
            isChanged = false;

        }


        if(planStep < _plan.plan.size()){
            RPAction rpa;
            rpa.runActionInterface(n, _plan.plan[planStep].name);
            rpa.concreteCallback( _plan.plan[planStep] );
            rpa.dispatchCallback( _plan.plan[planStep] );

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}