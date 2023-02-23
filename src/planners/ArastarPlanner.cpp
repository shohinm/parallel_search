#include <iostream>
#include <planners/ArastarPlanner.hpp>

using namespace std;
using namespace ps;

ArastarPlanner::ArastarPlanner(ParamsType planner_params):
WastarPlanner(planner_params)
{    
    time_budget_ = planner_params_["time_budget"];
}

ArastarPlanner::~ArastarPlanner()
{
    
}

bool ArastarPlanner::Plan()
{
    initialize();
    auto t_start = chrono::steady_clock::now();
    double heuristic_w = heuristic_w_;
    double time_budget = time_budget_;
    while (heuristic_w_>=1 && time_budget_>0)
    {
        resetClosed();
        improvePath();

        // Early termination if there's no solution
        if (goal_state_ptr_ == NULL)
        {
            break;
        }
        heuristic_w_ -= 1;
        // Append inconsistent list to open
        for(auto it_state = state_incon_list_.begin(); it_state != state_incon_list_.end(); it_state++)
        {
            state_open_list_.push(*it_state);
        }
        state_incon_list_.clear();

        StateQueueMinType state_open_list;
        while (!state_open_list_.empty())
        {
            auto state = state_open_list_.min();
            state_open_list_.pop();

            state->SetFValue(state->GetGValue() + heuristic_w_*state->GetHValue());

        }
    }

}

void ArastarPlanner::improvePath()
{
    auto t_start = chrono::steady_clock::now();
    while (!state_open_list_.empty())
    {
        auto state_ptr = state_open_list_.min();
        state_open_list_.pop();

        // Return solution if goal state is expanded
        if (isGoalState(state_ptr))
        {
            auto t_end = chrono::steady_clock::now();
            double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();            
            goal_state_ptr_ = state_ptr;
            
            // Reconstruct and return path
            constructPlan(state_ptr);   
            planner_stats_.total_time_ = 1e-9*t_elapsed;
            exit();
            return true;
        }

        expandState(state_ptr);        
        
    }

    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
    planner_stats_.total_time_ = 1e-9*t_elapsed;
    return false;
}

void ArastarPlanner::initialize()
{
    WastarPlanner::initialize();
    state_incon_list_.clear();
}

void ArastarPlanner::expandState(StatePtrType state_ptr)
{
    
    if (VERBOSE) state_ptr->Print("Expanding");

    planner_stats_.num_jobs_per_thread_[0] +=1;
    planner_stats_.num_state_expansions_++;

    state_ptr->SetVisited();
   
    for (auto& action_ptr: actions_ptrs_)
    {
        if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
        {
            // Evaluate the edge
            auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars());
            planner_stats_.num_evaluated_edges_++; // Only the edges controllers that satisfied pre-conditions and args are in the open list
            //********************
            
            updateState(state_ptr, action_ptr, action_successor);
        }
    }
}

void ArastarPlanner::updateState(StatePtrType& state_ptr, ActionPtrType& action_ptr, ActionSuccessor& action_successor)
{
    if (action_successor.success_)
    {
        auto successor_state_ptr = constructState(action_successor.successor_state_vars_costs_.back().first);
        double cost = action_successor.successor_state_vars_costs_.back().second;                

        if (!successor_state_ptr->IsVisited())
        {
            double new_g_val = state_ptr->GetGValue() + cost;
            
            if (successor_state_ptr->GetGValue() > new_g_val)
            {

                double h_val = successor_state_ptr->GetHValue();
                if (h_val == -1)
                {
                    h_val = computeHeuristic(successor_state_ptr);
                    successor_state_ptr->SetHValue(h_val);        
                }

                if (h_val != DINF)
                {
                    h_val_min_ = h_val < h_val_min_ ? h_val : h_val_min_;
                    successor_state_ptr->SetGValue(new_g_val);
                    successor_state_ptr->SetFValue(new_g_val + heuristic_w_*h_val);
                    
                    auto edge_ptr = new Edge(state_ptr, successor_state_ptr, action_ptr);
                    edge_ptr->SetCost(cost);
                    edge_map_.insert(make_pair(getEdgeKey(edge_ptr), edge_ptr));
                    
                    successor_state_ptr->SetIncomingEdgePtr(edge_ptr);
                    
                    if (state_open_list_.contains(successor_state_ptr))
                    {
                        state_open_list_.decrease(successor_state_ptr);
                    }
                    else
                    {
                        state_open_list_.push(successor_state_ptr);
                    }

                }

            }       
        }
    } 
}

void ArastarPlanner::exit()
{
    state_incon_list_.clear();
    WastarPlanner::exit();
}
