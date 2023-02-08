#ifndef INSAT_NAV2D_ACTION_HPP
#define INSAT_NAV2D_ACTION_HPP

#include <common/Action.hpp>
#include "RobotNav2dActions.hpp"

namespace ps
{

    class InsatNav2dAction : public RobotNav2dAction
    {

    public:
        InsatNav2dAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, bool is_expensive = true)
                : RobotNav2dAction(type, params, map, is_expensive){};
        bool CheckPreconditions(StateVarsType state);
        ActionSuccessor GetSuccessor(StateVarsType state_vars, int thread_id);
        ActionSuccessor GetSuccessorLazy(StateVarsType state_vars, int thread_id);
        ActionSuccessor Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id=0);

    protected:

    };


}

#endif
