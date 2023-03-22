#include <iostream>
#include <planners/ArastarPlanner.hpp>
#include <cmath>
#include <fstream>

#define EXPERIMENT 1

using namespace std;
using namespace ps;

ArastarPlanner::ArastarPlanner(ParamsType planner_params):
WastarPlanner(planner_params)
{    

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
        // cout << "Heuristic weight: " << heuristic_w_ << endl;
        // cout << "Goal state: " << goal_state_ptr_->GetFValue() << endl;

        if (EXPERIMENT)
        {
            auto t_end = chrono::steady_clock::now();
            double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
            data_list.push_back(heuristic_w_);
            data_list.push_back(1e-9*t_elapsed);
            data_list.push_back(goal_state_ptr_->GetFValue());
            data_list.push_back(planner_stats_.num_state_expansions_);
            data_list.push_back(planner_stats_.num_evaluated_edges_);
        }
        // Early termination if there's no solution
        if (goal_state_ptr_ == NULL || heuristic_w_ == 1)
        {
            break;
        }

        if (ADAPTIVE)
        {
            // Get max e-value of current open list
            double max_e_value = std::numeric_limits<double>::min();

            // debug
            // size_t state_key;
            // StatePtrMapType::iterator it_state;

            // Iterate through open list
            for(auto it_state = state_open_list_.begin(); it_state != state_open_list_.end(); it_state++)
            {
                auto state = *it_state;
                if (state->GetHValue() != 0 && state->GetGValue() < goal_state_ptr_->GetGValue())
                {
                    double e_value = (goal_state_ptr_->GetGValue() - state->GetGValue())/state->GetHValue();
                    if (e_value > max_e_value)
                    {
                        max_e_value = e_value;

                        // debug
                        // state_key = state_key_generator_(state->GetStateVars());
                    }
                }
            }

            // state with max e-value
            // it_state = state_map_.find(state_key);
            // it_state->second->Print("state max e-value");
            // cout << it_state->second->expansion_priority_ << endl;
            // cout << best_cost_ << endl;
            // if (best_cost_ + 1e-4 < it_state->second->expansion_priority_)
            // {
            //     cout << "Rounding error????" << endl;
            // }
            if (max_e_value != std::numeric_limits<double>::min())
            {
                heuristic_w_ = max_e_value;
                if (heuristic_w_ < 1)
                {
                    heuristic_w_ = 1;
                }
                // min_state->parent_state_ptr_->Print("Min state open");
                // cout << "Updated Heuristic weight: " << heuristic_w_ << endl;
            }
            else
            {
                break;
            }
        }
        else
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

    if (EXPERIMENT)
    {
        string filename = "experiment_1_arastar_" + to_string(heuristic_w) + "_" + to_string(delta_w_)+ ".txt";
        std::ofstream newFile(filename, std::ios::app);
        for (auto data : data_list)
        {
            newFile << data << ",";
        }
        newFile << endl;
    }

    // Reset heuristic weight & time budget
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
    delta_w_ = 0.5;
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
                if (goal_state_ptr_->GetFValue() + 1e-2 < state_open_list_.min()->GetFValue())
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

            // // Evaluate the edge
            // if(!edge_ptr_next->is_eval_)
            // {
            //     auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars());
            //     edge_ptr_next->is_eval_ = true;
            //     planner_stats_.num_evaluated_edges_++; // Only the edges controllers that satisfied pre-conditions and args are in the open list
            //     //********************
            // }
            // else
            // {
            //     // update existing successor state
            //     auto action_successor = edge_ptr_next->child_state_ptr_;
            // }
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
        else
        {
            edge_ptr->is_invalid_ = true;
        }
    }
    else if (!edge_ptr->is_invalid_)
    {
        double new_g_val = edge_ptr->parent_state_ptr_->GetGValue() + edge_ptr->GetCost();
        auto successor_state_ptr = edge_ptr->child_state_ptr_;

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
}

void ArastarPlanner::exit()
{
    state_incon_list_.clear();
    WastarPlanner::exit();
}
