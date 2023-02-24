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
            state_open_list.push(state);
        }
        state_open_list_ = move(state_open_list);

    }

    // Reset heuristic weight & time budget
    heuristic_w_ = heuristic_w;
    time_budget_ = time_budget;
    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
    planner_stats_.total_time_ = 1e-9*t_elapsed;
    if (goal_state_ptr_ != NULL) {
        exit();
        return true;
    }
    exit();
    return false;
}

void ArastarPlanner::initialize()
{
    WastarPlanner::initialize();
    state_incon_list_.clear();
}

void ArastarPlanner::improvePath()
{
    auto t_start = chrono::steady_clock::now();
    double t_spent = 0;
    while (t_spent <= time_budget_)
    {
        // Terminate condition check
        if (goal_state_ptr_ != NULL)
        {
            if (!state_open_list_.empty())
            {
                if (goal_state_ptr_->GetFValue() < state_open_list_.min()->GetFValue())
                {
                    // Construct path and return
                    auto goal_state_ptr = goal_state_ptr_;
                    plan_.clear();
                    constructPlan(goal_state_ptr);
                    auto t_end = chrono::steady_clock::now();
                    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
                    time_budget_ -= 1e-9*t_elapsed;
                    return;
                }
            }
            else
            {
                // Construct path and return
                auto goal_state_ptr = goal_state_ptr_;
                plan_.clear();
                constructPlan(goal_state_ptr);
                auto t_end = chrono::steady_clock::now();
                double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
                time_budget_ -= 1e-9*t_elapsed;
                return;
            }
        }

        if (state_open_list_.empty())
        {
            auto t_end = chrono::steady_clock::now();
            double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
            time_budget_ -= 1e-9*t_elapsed;
            return;
        }

        auto state_ptr = state_open_list_.min();
        state_open_list_.pop();

        // Update goal state ptr
        if (isGoalState(state_ptr))
        {
            if (goal_state_ptr_ == NULL)
                goal_state_ptr_ = state_ptr;
            else if(state_ptr->GetGValue() < goal_state_ptr_->GetGValue())
                goal_state_ptr_ = state_ptr;
        }

        expandState(state_ptr);        

        // Time budget check
        auto t_curr = chrono::steady_clock::now();
        double t_spent = 1e-9*chrono::duration_cast<chrono::nanoseconds>(t_curr-t_start).count();
    }

    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
    time_budget_ -= 1e-9*t_elapsed;
    return;
}

void ArastarPlanner::expandState(StatePtrType state_ptr)
{
    
    if (VERBOSE) state_ptr->Print("Expanding");

    planner_stats_.num_jobs_per_thread_[0] +=1;
    planner_stats_.num_state_expansions_++;

    state_ptr->SetVisited();
    state_ptr->SetVValue(state_ptr->GetGValue());
   
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
                
                if (!successor_state_ptr->IsVisited())
                {
                    if (state_open_list_.contains(successor_state_ptr))
                    {
                        state_open_list_.decrease(successor_state_ptr);
                    }
                    else
                    {
                        state_open_list_.push(successor_state_ptr);
                    }
                }
                else
                {
                    if (find(state_incon_list_.begin(), state_incon_list_.end(), successor_state_ptr) == state_incon_list_.end())
                    {
                        state_incon_list_.push_back(successor_state_ptr);
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
