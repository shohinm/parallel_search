#ifndef INSAT_ACTION_HPP
#define INSAT_ACTION_HPP

#include <common/Action.hpp>

namespace ps
{

    class InsatAction : public Action
    {

    public:
        // Action(){};
        InsatAction(const std::string& type, ParamsType params = ParamsType(), bool is_expensive = true)
                : Action(type, params, is_expensive){};
        virtual ~InsatAction(){};
        virtual bool CheckPreconditions(StateVarsType state)=0;
        virtual ActionSuccessor GetSuccessor(StateVarsType state_vars, int thread_id=0)=0;
        virtual ActionSuccessor GetSuccessorLazy(StateVarsType state_vars, int thread_id=0){};
        virtual ActionSuccessor Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id=0){};

    protected:

    };

}

#endif
