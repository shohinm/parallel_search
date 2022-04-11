#include <iostream>
#include <State.hpp>

using namespace std;
using namespace epase;

size_t State::id_counter_ = 0;

// State::State():
// g_val_(numeric_limits<double>::infinity()),
// f_val_(numeric_limits<double>::infinity()),
// is_visited_(false)
// {
//     state_id_ = id_counter_++;
// }

State::State(const StateVarsType& vars):
vars_(vars),
g_val_(numeric_limits<double>::infinity()),
f_val_(numeric_limits<double>::infinity()),
is_visited_(false),
being_expanded_(false),
num_successors_(0),
num_expanded_successors_(0)
{
    state_id_ = id_counter_++;
}

void State::Print(string str)
{
    cout << str
    << "State: " << GetStateID() 
    << " | Vars: [";
    for (auto& v : vars_)
    	cout << v << ", "; 
    cout << "] | g: " << GetGValue() 
    << " | h: " << GetHValue() 
    << " | f: "<< GetFValue() 
    << " | is_closed: " << IsVisited()
    << " | num_successors: " << num_successors_ 
    << " | num_expanded_successors: " << num_expanded_successors_ 
    << endl;
}

bool IsLesserState::operator()(const State& lhs, const State& rhs)
{
    if (lhs.GetFValue() == rhs.GetFValue()) // tie breaking
        return lhs.GetStateID() < rhs.GetStateID();
    else
		return lhs.GetFValue() < rhs.GetFValue();
}

