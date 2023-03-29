#include <iostream>
#include <planners/WastarPlanner.hpp>

using namespace std;
using namespace ps;

WastarPlanner::WastarPlanner(ParamsType planner_params):
Planner(planner_params)
{    

}

WastarPlanner::~WastarPlanner()
{
    
}

bool WastarPlanner::Plan()
{
    initialize();
    startTimer();   
    while (!state_open_list_.empty() && !checkTimeout())
    {
        auto state_ptr = state_open_list_.min();
        state_open_list_.pop();

        // Return solution if goal state is expanded
        if (isGoalState(state_ptr))
        {
            auto t_end = chrono::steady_clock::now();
            double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();            
            goal_state_ptr_ = state_ptr;
            
            // Reconstruct and return path
            constructPlan(state_ptr);   
            planner_stats_.total_time = 1e-9*t_elapsed;
            exit();
            return true;
        }

        expandState(state_ptr);        
        
    }

    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
    planner_stats_.total_time = 1e-9*t_elapsed;
    return false;
}

void WastarPlanner::initialize()
{
    Planner::initialize();
    planner_stats_.num_jobs_per_thread.resize(1, 0);

    // Initialize open list
    start_state_ptr_->SetFValue(start_state_ptr_->GetGValue() + heuristic_w_*start_state_ptr_->GetHValue());
    state_open_list_.push(start_state_ptr_);

    planner_stats_.num_threads_spawned = 1;
}

void WastarPlanner::expandState(StatePtrType state_ptr)
{
    
    if (VERBOSE) state_ptr->Print("Expanding");

    planner_stats_.num_jobs_per_thread[0] +=1;
    planner_stats_.num_state_expansions_++;

    state_ptr->SetVisited();
   
    for (auto& action_ptr: actions_ptrs_)
    {
        if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
        {
            // Evaluate the edge
            auto t_start = chrono::steady_clock::now();
            auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars());
            auto t_end = chrono::steady_clock::now();
            planner_stats_.action_eval_times[action_ptr->GetType()].emplace_back(1e-9*chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count());
            planner_stats_.num_evaluated_edges++; // Only the edges controllers that satisfied pre-conditions and args are in the open list
            //********************
            
            updateState(state_ptr, action_ptr, action_successor);
        }
    }
}

void WastarPlanner::updateState(StatePtrType& state_ptr, ActionPtrType& action_ptr, ActionSuccessor& action_successor)
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

void WastarPlanner::exit()
{
    // Clear open list
    while (!state_open_list_.empty())
    {
        state_open_list_.pop();
    }
    
    Planner::exit();
}
