#ifndef ACTION_HPP
#define ACTION_HPP

#include <common/State.hpp>

namespace ps
{

class Action
{

public:
    // Action(){};
    Action(const std::string& type, ParamsType params = ParamsType()):type_(type), params_(params){};
    virtual ~Action(){};
    virtual bool CheckPreconditions(StateVarsType state)=0; 
    virtual ActionSuccessor GetSuccessor(StateVarsType state_vars, int thread_id=0)=0; 
    virtual ActionSuccessor GetSuccessorLazy(StateVarsType state_vars, int thread_id=0){}; 
    virtual ActionSuccessor Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id=0){}; 
    std::string GetType() const {return type_;};
    bool operator==(const Action& other_action) const
    {
        return type_ == other_action.type_;
    }

protected:
    std::string type_;
    ParamsType params_;

};

}

#endif
