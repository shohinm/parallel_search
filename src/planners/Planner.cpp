#include <iostream>
#include <cmath>
#include <planners/Planner.hpp>

using namespace std;
using namespace ps;

Planner::Planner(ParamsType planner_params):
planner_params_(planner_params)
{
    heuristic_w_ = planner_params_["heuristic_weight"];
}

Planner::~Planner()
{
	cleanUp();
}

void Planner::SetActions(vector<shared_ptr<Action>> actions_ptrs)
{
    actions_ptrs_ = actions_ptrs;
}

void Planner::SetStartState(const StateVarsType& state_vars)
{
	start_state_ptr_ = constructState(state_vars);
}

void Planner::SetGoalChecker(function<bool(const StatePtrType&)> callback)
{
    goal_checker_ = callback;
}

void Planner::SetStateMapKeyGenerator(function<size_t(const StateVarsType&)> callback)
{
	state_key_generator_ = callback;
}

void Planner::SetEdgeKeyGenerator(function<size_t(const EdgePtrType&)> callback)
{
	edge_key_generator_ = callback;
}

void Planner::SetHeuristicGenerator(function<double(const StatePtrType&)> callback)
{
	unary_heuristic_generator_ = callback;
}

void Planner::SetStateToStateHeuristicGenerator(function<double(const StatePtrType&, const StatePtrType&)> callback)
{
	binary_heuristic_generator_ = callback;
}

std::vector<PlanElement> Planner::GetPlan() const
{
    return plan_;
}

PlannerStats Planner::GetStats() const
{
    return planner_stats_;
}

void Planner::initialize()
{
    plan_.clear();

    // Initialize planner stats
    planner_stats_ = PlannerStats();

    // Initialize start state
    start_state_ptr_->SetGValue(0);
    start_state_ptr_->SetHValue(computeHeuristic(start_state_ptr_));
    
    // Reset goal state
    goal_state_ptr_ = NULL;

    // Reset state
    planner_stats_ = PlannerStats();

    // Reset h_min
    h_val_min_ = DINF;

}


void Planner::resetStates()
{
    for (auto it = state_map_.begin(); it != state_map_.end(); ++it)
    {
        it->second->ResetGValue();
        it->second->ResetFValue();
        it->second->ResetVValue();
        it->second->ResetIncomingEdgePtr();        
        it->second->UnsetVisited();     
        it->second->UnsetBeingExpanded();      
        it->second->num_successors_ = 0;   
        it->second->num_expanded_successors_ = 0;   
    }
}

void Planner::resetClosed()
{
    // cout << "*********************** REsetting *********************" << endl;
    for (auto it = state_map_.begin(); it != state_map_.end(); ++it)
    {
        // it->second->Print();
        it->second->UnsetVisited();     
        it->second->UnsetBeingExpanded();      
        // it->second->num_successors_ = 0;
        // it->second->num_expanded_successors_ = 0;   
    }
    // cout << "***********************  *********************" << endl;
}

size_t Planner::getEdgeKey(const EdgePtrType& edge_ptr)
{
    if (edge_ptr->action_ptr_ == NULL) // proxy edge
        return state_key_generator_(edge_ptr->parent_state_ptr_->GetStateVars());
    else 
        return edge_key_generator_(edge_ptr);
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

double Planner::computeHeuristic(const StatePtrType& state_ptr)
{
    return roundOff(unary_heuristic_generator_(state_ptr));
}

double Planner::computeHeuristic(const StatePtrType& state_ptr_1, const StatePtrType& state_ptr_2)
{
    return roundOff(binary_heuristic_generator_(state_ptr_1, state_ptr_2));
}

bool Planner::isGoalState(const StatePtrType& state_ptr)
{
    return goal_checker_(state_ptr);
}

void Planner::constructPlan(StatePtrType& state_ptr)
{
    double cost = 0;
    plan_.clear();
    while(state_ptr->GetIncomingEdgePtr())
    {
        if (state_ptr->GetIncomingEdgePtr()) // For start state_ptr, there is no incoming edge
            plan_.insert(plan_.begin(), PlanElement(state_ptr->GetStateVars(), state_ptr->GetIncomingEdgePtr()->action_ptr_, state_ptr->GetIncomingEdgePtr()->GetCost()));        
        else
            plan_.insert(plan_.begin(), PlanElement(state_ptr->GetStateVars(), NULL, 0));        
        state_ptr->GetIncomingEdgePtr()->Print();
        cost += state_ptr->GetIncomingEdgePtr()->GetCost();
        state_ptr = state_ptr->GetIncomingEdgePtr()->parent_state_ptr_;     
    }

    planner_stats_.path_length_ = plan_.size();
    planner_stats_.path_cost_ = cost;
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

void Planner::exit()
{
    cleanUp();
}
