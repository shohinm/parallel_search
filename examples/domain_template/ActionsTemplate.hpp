#ifndef ACTIONS_TEMPLATE_HPP
#define ACTIONS_TEMPLATE_HPP

#include <common/Action.hpp>

namespace epase
{

class ActionTemplate : public Action
{

public:
    ActionTemplate(const std::string& type, ParamsType params);
    ActionSuccessor Apply(StateVarsType state_vars, int thread_id); 
    bool CheckPreconditions(StateVarsType state); 
};

}

#endif
