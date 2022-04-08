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
g_val_(numeric_limits<double>::infinity()),
f_val_(numeric_limits<double>::infinity()),
is_visited_(false),
vars_(vars)
{
    state_id_ = id_counter_++;
}


bool IsLesserState::operator()(const State& lhs, const State& rhs)
{
    if (lhs.GetFValue() == rhs.GetFValue()) // tie breaking
        return lhs.GetStateID() < rhs.GetStateID();
    else
		return lhs.GetFValue() < rhs.GetFValue();
}

