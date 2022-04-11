#ifndef ACTION_HPP
#define ACTION_HPP

#include <State.hpp>

namespace epase
{

class Action
{

public:
    // Action(){};
    Action(const std::string& type, ParamsType params = ParamsType()):type_(type), params_(params){};
    virtual ActionSuccessor Apply(StateVarsType state){}; 
    virtual bool CheckPreconditions(StateVarsType state){}; 
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
