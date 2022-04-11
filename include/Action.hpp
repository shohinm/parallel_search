#ifndef ACTION_HPP
#define ACTION_HPP

#include <State.hpp>

namespace epase
{

class Action
{

public:
	Action(){};
	Action(const std::string& type):type_(type){};
    virtual ActionSuccessor Apply(StateVarsType state){}; 
    virtual bool CheckPreconditions(StateVarsType state){}; 

    bool operator==(const Action& other_action) const
    {
        return type_ == other_action.type_;
    }
    std::string type_;

};

}

#endif
