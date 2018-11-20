//
// Created by dtrimoul on 11/20/18.
//

#include "rosplan_simple_dispatcher/RPAKbUpdater.h"

void RPAKbUpdater::storeOperatorDetails(std::string action) {

    std::vector<std::string> actionDecomposition;
    actionDecomposition = getActionAndParam(action);
    rosplan_knowledge_msgs::GetDomainOperatorDetailsService predMsg;
    predMsg.request.name = actionDecomposition[0];
    if(_kb_operator_detail_server.call(predMsg) ){
        _stored_operators.emplace( actionDecomposition[0],  predMsg );
    }

   _parameters = mapParameters(actionDecomposition , predMsg.response.op.formula);
    _currentMsg = predMsg;


}

void RPAKbUpdater::updateAtEnd() {
    _kb_pred_update_server.call(_atEnd);
}

void RPAKbUpdater::updateAtStart() {
    _kb_pred_update_server.call(_atStart);
}

void RPAKbUpdater::createConcretePredicates() {
    _atStart.request.knowledge.clear();
    _atEnd.request.knowledge.clear();

    for (int i = 0; i < _currentMsg.response.op.at_start_add_effects.size() ; ++i) {
        rosplan_knowledge_msgs::KnowledgeItem curPred;
        curPred.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        curPred.attribute_name = _currentMsg.response.op.at_start_add_effects[i].name;
        diagnostic_msgs::KeyValue pair;
        for (int j = 0; j < _currentMsg.response.op.at_start_add_effects[i].typed_parameters.size() ; ++j) {
            pair.key = _currentMsg.response.op.at_start_add_effects[i].typed_parameters[j].key;
            pair.value = _parameters[_currentMsg.response.op.at_start_add_effects[i].typed_parameters[j].key] ;
            curPred.values.push_back(pair);
        }
        _atStart.request.knowledge.push_back(curPred);
        _atStart.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
    }

    for (int i = 0; i < _currentMsg.response.op.at_start_del_effects.size() ; ++i) {
        rosplan_knowledge_msgs::KnowledgeItem curPred;
        curPred.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        curPred.attribute_name = _currentMsg.response.op.at_start_del_effects[i].name;
        diagnostic_msgs::KeyValue pair;
        for (int j = 0; j < _currentMsg.response.op.at_start_del_effects[i].typed_parameters.size() ; ++j) {
            pair.key = _currentMsg.response.op.at_start_del_effects[i].typed_parameters[j].key;
            pair.value = _parameters[_currentMsg.response.op.at_start_del_effects[i].typed_parameters[j].key] ;
            curPred.values.push_back(pair);
        }
        _atStart.request.knowledge.push_back(curPred);
        _atStart.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
    }

    for (int i = 0; i < _currentMsg.response.op.at_end_add_effects.size() ; ++i) {
        rosplan_knowledge_msgs::KnowledgeItem curPred;
        curPred.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        curPred.attribute_name = _currentMsg.response.op.at_end_add_effects[i].name;
        diagnostic_msgs::KeyValue pair;
        for (int j = 0; j < _currentMsg.response.op.at_end_add_effects[i].typed_parameters.size() ; ++j) {
            pair.key = _currentMsg.response.op.at_end_add_effects[i].typed_parameters[j].key;
            pair.value = _parameters[_currentMsg.response.op.at_end_add_effects[i].typed_parameters[j].key] ;
            curPred.values.push_back(pair);
        }
        _atEnd.request.knowledge.push_back(curPred);
        _atEnd.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
    }

    for (int i = 0; i < _currentMsg.response.op.at_end_del_effects.size() ; ++i) {
        rosplan_knowledge_msgs::KnowledgeItem curPred;
        curPred.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        curPred.attribute_name = _currentMsg.response.op.at_end_del_effects[i].name;
        diagnostic_msgs::KeyValue pair;
        for (int j = 0; j < _currentMsg.response.op.at_end_del_effects[i].typed_parameters.size() ; ++j) {
            pair.key = _currentMsg.response.op.at_end_del_effects[i].typed_parameters[j].key;
            pair.value = _parameters[_currentMsg.response.op.at_end_del_effects[i].typed_parameters[j].key] ;
            curPred.values.push_back(pair);
        }
        _atEnd.request.knowledge.push_back(curPred);
        _atEnd.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
    }
}

RPAKbUpdater::RPAKbUpdater(ros::NodeHandle nh) {
    _nh = nh;
    _kb_operator_server = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/rosplan_knowledge_base/domain/operators");
    _kb_operator_detail_server = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/rosplan_knowledge_base/domain/operator_details");
    _kb_pred_update_server = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
}

std::vector<std::string> RPAKbUpdater::getActionAndParam(std::string &action) {
    std::vector<std::string> decomposedAction;
    std::istringstream iss(action);
    while(std::getline(iss, action, ' ')){
        decomposedAction.push_back(action);
    }
    return decomposedAction;
}

std::map<std::string, std::string> RPAKbUpdater::mapParameters(std::vector<std::string> params,
                                                               rosplan_knowledge_msgs::DomainFormula action) {
    std::map<std::string, std::string> mappedParams;
    for (int i = 1; i < params.size(); ++i) {
        mappedParams.emplace(action.typed_parameters[i-1].key, params[i]);
    }
    return mappedParams;
}
