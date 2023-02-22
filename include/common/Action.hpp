#ifndef ACTION_HPP
#define ACTION_HPP

#include <common/State.hpp>

namespace ps
{

class Action
{

public:
    // Action(){};
    Action(const std::string& type, ParamsType params = ParamsType(), bool is_expensive = true)
    : type_(type), params_(params), is_expensive_(is_expensive){};
    virtual ~Action(){};
    virtual bool CheckPreconditions(const StateVarsType& state)=0; 
    virtual ActionSuccessor GetSuccessor(const StateVarsType& state_vars, int thread_id=0)=0; 
    virtual ActionSuccessor GetSuccessorLazy(const StateVarsType& state_vars, int thread_id=0){}; 
    virtual ActionSuccessor Evaluate(const StateVarsType& parent_state_vars, const StateVarsType& child_state_vars, int thread_id=0){}; 
    std::string GetType() const {return type_;};
    bool IsExpensive() const {return is_expensive_;};
    bool operator==(const Action& other_action) const
    {
        return type_ == other_action.type_;
    }

protected:
    std::string type_;
    ParamsType params_;
    bool is_expensive_;

};

}

#endif
