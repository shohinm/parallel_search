#include <iostream>
#include <cmath>
#include "Planner.hpp"

using namespace std;
using namespace epase;

Planner::Planner(ParamsType planner_params)
{

}

Planner::~Planner()
{
	cleanUp();
    start_state_ptr_ = NULL;
    goal_state_ptr_ = NULL;
    num_evaluated_edges_ = 0;
    num_state_expansions_ = 0;   
    total_time_ = 0;
    solution_cost_ = 0;

}

void Planner::SetActions(vector<shared_ptr<Action>> actions_ptrs)
{
    actions_ptrs_ = actions_ptrs;
}

void Planner::SetStartState(const StateVarsType& state_vars)
{
	start_state_ptr_ = constructState(state_vars);
}

void Planner::SetGoalChecker(function<double(const StatePtrType)> callback)
{

}

void Planner::SetStateMapKeyGenerator(function<size_t(const StateVarsType&)> callback)
{
	state_key_generator_ = callback;
}

void Planner::SetEdgeKeyGenerator(function<size_t(const EdgePtrType&)> callback)
{
	edge_key_generator_ = callback;
}

void Planner::SetHeuristicGenerator(function<double(const StatePtrType)> callback)
{
	unary_heuristic_generator_ = callback;
}

void Planner::SetStateToStateHeuristicGenerator(function<double(const StatePtrType, const StatePtrType)> callback)
{
	binary_heuristic_generator_ = callback;
}

bool Planner::PrintStats(int exp_idx)
{
    cout << "Total planning time: " << total_time_ << endl;
    cout << "Solution cost: " << solution_cost_ << endl;
    cout << "Number of state expansiosn: " << num_state_expansions_ << endl;
    cout << "Number of edges evaluaited: " << num_evaluated_edges_ << endl;
}

void Planner::initialize()
{
	cleanUp();
    plan_.clear();

}

void Planner::resetStates()
{
    for (auto it = state_map_.begin(); it != state_map_.end(); ++it)
    {
        it->second->ResetGValue();
        it->second->ResetFValue();
        // it->second->ResetVValue();
        it->second->ResetIncomingEdgePtr();        
        it->second->UnsetVisited();     
        it->second->UnsetBeingExpanded();      
        it->second->num_successors_ = 0;   
        it->second->num_expanded_successors_ = 0;   
    }
}

size_t Planner::getEdgeKey(const EdgePtrType& edge_ptr)
{
    if (edge_ptr->action_ptr_->GetType() != "dummy")
        return edge_key_generator_(edge_ptr);
    else // proxy edge for epase
        return state_key_generator_(edge_ptr->parent_state_ptr_->GetStateVars());
}

StatePtrType Planner::constructState(const StateVarsType& state)
{
    size_t key = state_key_generator_(state);
    StatePtrMapType::iterator it = state_map_.find(key);
    StatePtrType state_ptr;
    
    // Check if state exists in the search state map
    if (it == state_map_.end())
    {
        state_ptr = new State(state);
        state_map_.insert(pair<size_t, StatePtrType>(key, state_ptr));
    }
    else 
    {
        state_ptr = it->second;
    }
   
    return state_ptr;
}

double Planner::computeHeuristic(const StatePtrType state_ptr)
{
    return roundOff(unary_heuristic_generator_(state_ptr));
}

double Planner::computeHeuristic(const StatePtrType state_ptr_1, const StatePtrType state_ptr_2)
{
    return roundOff(binary_heuristic_generator_(state_ptr_1, state_ptr_2));
}

bool Planner::isGoalState(StatePtrType state)
{

}

void Planner::constructPlan(StatePtrType state)
{
    while(state->GetIncomingEdgePtr())
    {
        plan_.insert(plan_.begin(), PlanElement(state->GetStateVars(), state->GetIncomingEdgePtr()->action_ptr_, state->GetIncomingEdgePtr()->GetCost()));        
        solution_cost_ += state->GetIncomingEdgePtr()->GetCost();
        state = state->GetIncomingEdgePtr()->parent_state_ptr_;     
    }
}

double Planner::roundOff(double value, int prec)
{
    double pow_10 = pow(10.0, prec);
    return round(value * pow_10) / pow_10;
}

void Planner::cleanUp()
{
    for (auto& state_it : state_map_)
    {        
        if (state_it.second)
        {
            delete state_it.second;
            state_it.second = NULL;
        }
    }
	state_map_.clear();

    for (auto& edge_it : edge_map_)
    {
        if (edge_it.second)
        {
            delete edge_it.second;
            edge_it.second = NULL;
        }
    }
    edge_map_.clear();
    
    State::ResetStateIDCounter();
    Edge::ResetStateIDCounter();
}





