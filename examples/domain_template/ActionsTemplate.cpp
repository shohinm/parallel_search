#include "ActionsTemplate.hpp"

using namespace ps;

ActionTemplate::ActionTemplate(const std::string& type, ParamsType params): 
Action(type, params)
{

}

ActionSuccessor ActionTemplate::GetSuccessor(StateVarsType state_vars, int thread_id)
{

}

bool ActionTemplate::CheckPreconditions(StateVarsType state)
{

}
