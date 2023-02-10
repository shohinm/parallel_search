#ifndef ACTION_HPP
#define ACTION_HPP

#include <common/State.hpp>
#include <common/insat/InsatState.hpp>

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
    virtual ActionSuccessor GetSuccessor(StateVarsType state_vars, int thread_id=0)=0; 
    virtual ActionSuccessor GetSuccessorLazy(StateVarsType state_vars, int thread_id=0){}; 
    virtual ActionSuccessor Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id=0){};
    std::string GetType() const {return type_;};
    bool IsExpensive() const {return is_expensive_;};
    bool operator==(const Action& other_action) const
    {
        return type_ == other_action.type_;
    }

    // INSAT
    virtual TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0){};
    virtual TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0){};
    virtual double getCost(const TrajType& traj, int thread_id=0){};
    virtual bool isFeasible(TrajType& traj){};

protected:
    std::string type_;
    ParamsType params_;
    bool is_expensive_;

};

}

#endif
