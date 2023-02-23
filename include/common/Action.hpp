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
    virtual bool CheckPreconditions(StateVarsType state)=0; 
    virtual bool IsFeasible(const StateVarsType& state, int thread_id=0)
    {throw std::runtime_error("IsFeasible not implemented!");};
    virtual StateVarsType SampleFeasibleState(int thread_id=0)
    {throw std::runtime_error("SampleFeasibleState not implemented!");};
    virtual double GetCostToSuccessor(const StateVarsType& current_state, const StateVarsType& successor_state, int thread_id)
    {throw std::runtime_error("GetCostToSuccessor not implemented!");};
    virtual ActionSuccessor GetSuccessor(StateVarsType state_vars, int thread_id=0)=0; 
    virtual ActionSuccessor GetSuccessorLazy(StateVarsType state_vars, int thread_id=0)
    {throw std::runtime_error("GetSuccessorLazy not implemented!");};
    virtual ActionSuccessor Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id=0){};

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
