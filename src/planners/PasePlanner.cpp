#include <iostream>
#include <algorithm>
#include <planners/PasePlanner.hpp>

using namespace std;
using namespace ps;

PasePlanner::PasePlanner(ParamsType planner_params):
Planner(planner_params)
{    
    num_threads_  = planner_params["num_threads"];
    vector<LockType> lock_vec(num_threads_-1);
    lock_vec_.swap(lock_vec);
}

PasePlanner::~PasePlanner()
{
    
}

bool PasePlanner::Plan()
{
    initialize();
    
    planner_stats_.num_threads_spawned = 1;
    startTimer();   
    if (num_threads_ == 1)
    {
        paseThread(0);
    }
    else
    {
        for (int thread_id = 0; thread_id < num_threads_-1; ++thread_id)
        {
            if (VERBOSE) cout << "Spawining state expansion thread " << thread_id << endl;
            state_expansion_futures_.emplace_back(async(launch::async, &PasePlanner::paseThread, this, thread_id));
        }        
        
        planner_stats_.num_threads_spawned += state_expansion_futures_.size();
    }

    // Spin till termination, should be replaced by conditional variable
    while(!terminate_&& !checkTimeout()){}

    terminate_ = true;
    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
    planner_stats_.total_time = 1e-9*t_elapsed;
    exit();

    return plan_found_;    
}

void PasePlanner::initialize()
{
    Planner::initialize();
    planner_stats_.num_jobs_per_thread.resize(num_threads_, 0);

    terminate_ = false;
    recheck_flag_ = true;
    plan_found_ = false;
    
    state_expansion_futures_.clear();
    being_expanded_states_.clear();

    // Initialize open list
    start_state_ptr_->SetFValue(start_state_ptr_->GetGValue() + heuristic_w_*start_state_ptr_->GetHValue());
    state_open_list_.push(start_state_ptr_);

}

void PasePlanner::paseThread(int thread_id)
{
    lock_.lock();
    vector<StatePtrType> popped_state_ptrs;
    
    while(!terminate_)
    {

        StatePtrType state_ptr = NULL;

        while (!state_ptr && !terminate_)
        {
            
            // No solution check        
            if (state_open_list_.empty() && being_expanded_states_.empty())
            {
                terminate_ = true;
                plan_found_ = false;
                lock_.unlock();
                return;
            }

            while(!state_ptr && !state_open_list_.empty())
            {
                state_ptr = state_open_list_.min();
                state_open_list_.pop();
                popped_state_ptrs.emplace_back(state_ptr);


                if (state_ptr->IsBeingExpanded())
                    continue;

                for (auto& popped_state_ptr : popped_state_ptrs)
                {
                    auto h_diff = computeHeuristic(popped_state_ptr, state_ptr);

                    if (state_ptr->GetGValue() > popped_state_ptr->GetGValue() + heuristic_w_*h_diff)
                    {
                        state_ptr = NULL;
                        break;
                    }
                }
     
                if (state_ptr)
                {
                    for (auto& being_expanded_state : being_expanded_states_)
                    {
                        auto h_diff = computeHeuristic(being_expanded_state, state_ptr);
                        if (state_ptr->GetGValue() > being_expanded_state->GetGValue() + heuristic_w_*h_diff)
                        {
                            state_ptr = NULL;
                            break;
                        }
                    }                    
                }


            }
    

            // Re add the popped states except curr state which will be expanded now
            for (auto& popped_state_ptr : popped_state_ptrs)
            {
                if (popped_state_ptr != state_ptr)
                {
                    state_open_list_.push(popped_state_ptr);
                }
            }
            popped_state_ptrs.clear();

            if (!state_ptr)
            {
                lock_.unlock();
                // Wait for recheck_flag_ to be set true;
                while(!recheck_flag_ && !terminate_){}
                lock_.lock();    
                continue;
            }

            recheck_flag_ = false;
            
            // Return solution if goal state is expanded
            if (isGoalState(state_ptr) && (!terminate_))
            {
                // Reconstruct path and return
                goal_state_ptr_ = state_ptr;
                constructPlan(goal_state_ptr_);   
                plan_found_ = true;
                terminate_ = true;
                recheck_flag_ = true;
                lock_.unlock();
                return;
            }
            
        }

        // Needed since the while loop may exit when plan is found and state_ptr is NULL
        if (!state_ptr)
        {
            lock_.unlock();
            return;
        }

        state_ptr->SetBeingExpanded();
        being_expanded_states_.emplace_back(state_ptr);
        state_ptr->SetVisited();

        expandState(state_ptr, thread_id);    
    }
    
    lock_.unlock();

}

void PasePlanner::expandState(StatePtrType state_ptr, int thread_id)
{
    
    if (VERBOSE) state_ptr->Print("Expanding");
    
    planner_stats_.num_jobs_per_thread[thread_id] +=1;
    planner_stats_.num_state_expansions_++;
   
    for (auto& action_ptr: actions_ptrs_)
    {
        if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
        {

            lock_.unlock();
            // Evaluate the edge
            auto t_start = chrono::steady_clock::now();
            auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars(), thread_id);
            auto t_end = chrono::steady_clock::now();
            //********************
            lock_.lock();
            planner_stats_.action_eval_times[action_ptr->GetType()].emplace_back(1e-9*chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count());
            planner_stats_.num_evaluated_edges++; // Only the edges controllers that satisfied pre-conditions and args are in the open list

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
    }

    state_ptr->UnsetBeingExpanded();
    auto it_state_be = find(being_expanded_states_.begin(), being_expanded_states_.end(), state_ptr);
    if (it_state_be != being_expanded_states_.end())
    {
        being_expanded_states_.erase(it_state_be);
    }

    recheck_flag_ = true;
}

void PasePlanner::exit()
{
    bool all_expansion_threads_terminated = false;
    while (!all_expansion_threads_terminated)
    {
        all_expansion_threads_terminated = true;
        for (auto& fut : state_expansion_futures_)
        {
            if (!isFutureReady(fut))
            {
                all_expansion_threads_terminated = false;
                break;
            }
        }
    }
    state_expansion_futures_.clear();

    // Clear open list
    while (!state_open_list_.empty())
    {
        state_open_list_.pop();
    }
    
    Planner::exit();
}
