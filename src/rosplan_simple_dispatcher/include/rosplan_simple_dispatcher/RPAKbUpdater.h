//
// Created by dtrimoul on 11/20/18.
//

#ifndef ROSPLAN_SIMPLE_DISPATCHER_RPAKBUPDATER_H
#define ROSPLAN_SIMPLE_DISPATCHER_RPAKBUPDATER_H

#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"

class RPAKbUpdater {

private:
    ros::ServiceClient _kb_operator_detail_server;
    ros::ServiceClient _kb_pred_update_server;
    ros::ServiceClient _kb_operator_server;
    ros::NodeHandle _nh;
    // name , <action , pre/post condition>
    std::map<std::string, rosplan_knowledge_msgs::GetDomainOperatorDetailsService> _stored_operators;
    rosplan_knowledge_msgs::GetDomainOperatorDetailsService _currentMsg;
    std::map< std::string, std::string > _parameters;
    rosplan_knowledge_msgs::KnowledgeUpdateServiceArray _atStart;
    rosplan_knowledge_msgs::KnowledgeUpdateServiceArray _atEnd;

public:
    RPAKbUpdater(ros::NodeHandle nh);

    void storeOperatorDetails(std::string action);

    void createConcretePredicates();

    void updateAtStart();

    void updateAtEnd();

    std::vector<std::string> getActionAndParam(std::string &action);

    std::map<std::string, std::string> mapParameters(std::vector<std::string> vector,
                                                     rosplan_knowledge_msgs::DomainOperator_<std::allocator<void>>::_formula_type formula_);
};


#endif //ROSPLAN_SIMPLE_DISPATCHER_RPAKBUPDATER_H
