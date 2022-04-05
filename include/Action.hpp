#ifndef ACTION_HPP
#define ACTION_HPP

#include <Action.hpp>


class Action
{

public:
    virtual State Apply(State& state) = 0; 

}




#endif
