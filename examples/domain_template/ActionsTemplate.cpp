#include "ActionsTemplate.hpp"

using namespace epase;

ActionTemplate::ActionTemplate(const std::string& type, ParamsType params): 
Action(type, params)
{

}

ActionSuccessor ActionTemplate::Apply(StateVarsType state_vars, int thread_id)
{

}

bool ActionTemplate::CheckPreconditions(StateVarsType state)
{

}
