#include <iostream>
#include <planners/ArastarPlanner.hpp>
#include <cmath>
#include <fstream>

using namespace std;
using namespace ps;

ArastarPlanner::ArastarPlanner(ParamsType planner_params):
WastarPlanner(planner_params)
{    
    delta_w_ = planner_params_["heuristic_reduction"];
}

ArastarPlanner::~ArastarPlanner()
{

}

bool ArastarPlanner::Plan()
{
    initialize();

    // Experiment
    std::vector<double> data_list;

    startTimer();
    double heuristic_w = heuristic_w_;
    while ((heuristic_w_>1 || fabs(heuristic_w_ - 1)<0.001f) && !checkTimeout())
    {
        resetClosed();
        improvePath();

        // Early termination if there's no solution
        if (goal_state_ptr_ == NULL || heuristic_w_ == 1)
        {
            break;
        }

        // Update heuristic weight
        heuristic_w_ -= delta_w_;

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

    // Reset heuristic weight
    heuristic_w_ = heuristic_w;
    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
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
    while (!checkTimeout())
    {
        // Terminate condition check
        if (goal_state_ptr_ != NULL)
        {
            if (!state_open_list_.empty())
            {
                if (goal_state_ptr_->GetFValue() + 1e-5 < state_open_list_.min()->GetFValue())
                {
                    // Construct path and return
                    auto goal_state_ptr = goal_state_ptr_;
                    plan_.clear();
                    constructPlan(goal_state_ptr);
                    return;
                }
            }
            else
            {
                // Construct path and return
                auto goal_state_ptr = goal_state_ptr_;
                plan_.clear();
                constructPlan(goal_state_ptr);
                return;
            }
        }

        if (state_open_list_.empty())
        {
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
    }

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
            // Check if the edge is already in the edge map
            auto edge_temp = Edge(state_ptr, action_ptr);
            auto edge_key = getEdgeKey(&edge_temp);
            auto it_edge = edge_map_.find(edge_key); 
            EdgePtrType edge_ptr_next;

            if (it_edge == edge_map_.end())
            {
                edge_ptr_next = new Edge(state_ptr, action_ptr);
                edge_map_.insert(make_pair(edge_key, edge_ptr_next));
            }
            else
            {
                edge_ptr_next = it_edge->second;
            }
            // Update state
            updateState(state_ptr, action_ptr, edge_ptr_next);
        }
    }
}

void ArastarPlanner::updateState(StatePtrType& state_ptr, ActionPtrType& action_ptr, EdgePtrType& edge_ptr)
{
    if (!edge_ptr->is_eval_)
    {
        auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars());
        edge_ptr->is_eval_ = true;
        planner_stats_.num_evaluated_edges_++;
        if (action_successor.success_) 
        {
            auto successor_state_ptr = constructState(action_successor.successor_state_vars_costs_.back().first);
            double cost = action_successor.successor_state_vars_costs_.back().second;                
            double new_g_val = state_ptr->GetGValue() + cost;

            // Set successor and cost in expanded edge
            edge_ptr->child_state_ptr_ = successor_state_ptr;
            edge_ptr->SetCost(cost);
            updateEdge(edge_ptr, successor_state_ptr, new_g_val);
        }
        else
        {
            edge_ptr->is_invalid_ = true;
        }
    }
    else if (!edge_ptr->is_invalid_)
    {
        double new_g_val = edge_ptr->parent_state_ptr_->GetGValue() + edge_ptr->GetCost();
        auto successor_state_ptr = edge_ptr->child_state_ptr_;
        updateEdge(edge_ptr, successor_state_ptr, new_g_val);
    }  
}

void ArastarPlanner::updateEdge(EdgePtrType edge_ptr, StatePtrType successor_state_ptr, double new_g_val)
{
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

void ArastarPlanner::exit()
{
    state_incon_list_.clear();
    WastarPlanner::exit();
}
