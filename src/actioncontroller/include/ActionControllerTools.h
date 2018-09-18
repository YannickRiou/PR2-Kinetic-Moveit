#ifndef ACTION_CONTROLLER_TOOLS
#define ACTION_CONTROLLER_TOOLS

#include <string>
#include <actioncontroller/ActionControllerAction.h>
#include <actionlib/client/simple_action_client.h>


namespace actioncontroller{

    class ActionControllerTools{

    private:

    public:
        void createActionControllerMessage(std::string group_id, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &goal);

        void createActionControllerMessage(std::string group_id, std::string object, actioncontroller::ActionControllerGoal &msg);

        void createActionControllerMessage(std::string group_id, std::string object, float x, float y, float z, float w, float ox, float oy, float oz, actioncontroller::ActionControllerGoal &goal);

        void callingActionController(std::string part, actioncontroller::ActionControllerGoal &goal);
    };
}

#endif
