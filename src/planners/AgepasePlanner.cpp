#include <iostream>
#include <algorithm>
#include <cmath>
#include <planners/AgepasePlanner.hpp>
 
#include <fstream>
#define DEBUG 0
#define EXPERIMENT 1

using namespace std;
using namespace ps;
    
AgepasePlanner::AgepasePlanner(ParamsType planner_params):
GepasePlanner(planner_params)
{    
    naive_  = planner_params["naive"];
    adaptive_ = planner_params["adaptive"];
}

AgepasePlanner::~AgepasePlanner()
{
    
}

bool AgepasePlanner::Plan()
{
    initialize();

    // Experiment
    std::vector<double> data_list;
    smpl::intrusive_heap<State, IsGreaterState> state_close_list;

    startTimer();
    double heuristic_w = heuristic_w_;
    while ((heuristic_w_>1 || fabs(heuristic_w_ - 1)<0.001f) && !checkTimeout())
    {
        terminate_ = false;

        resetClosed();
        state_close_list.clear();
        // cout << "Start state: " << start_state_ptr_->GetFValue() << endl;
        being_expanded_states_.clear();
        improvePath();
        // cout << "Start state: " << start_state_ptr_->GetFValue() << endl;
        if (DEBUG)
        {
            cout << "Heuristic weight: " << heuristic_w_ << endl;
            cout << "Goal state: " << goal_state_ptr_->GetFValue() << endl;
            cout << "Best Cost: " << best_cost_ << endl;
            cout << "Open list size: " << edge_open_list_.size() << endl;
            cout << "min edge open list:" << edge_open_list_.min()->expansion_priority_ << endl;
            cout << "Incon list size: " << edge_incon_list_.size() << endl;
        }
        if (goal_state_ptr_->GetFValue() < best_cost_)
        {
            best_cost_ = goal_state_ptr_->GetFValue();
            best_plan_.clear();
            best_plan_ = plan_;
        }
        if (EXPERIMENT)
        {
            auto t_end = chrono::steady_clock::now();
            double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
            data_list.push_back(heuristic_w_);
            data_list.push_back(1e-9*t_elapsed);
            data_list.push_back(best_cost_);
        }
        // cout << "Min edge open: " << edge_open_list_.min()->expansion_priority_ << endl;
        // cout << "Min state BE: " << being_expanded_states_.min()->GetFValue() << endl;

        // Early termination if there's no solution
        if (goal_state_ptr_ == NULL || heuristic_w_ == 1)
        {
            break;
        }

        // if (!naive_)
        // {
        //     // append inconsistent list's edges into Eopen
        //     for(auto it_edge = edge_incon_list_.begin(); it_edge != edge_incon_list_.end(); it_edge++)
        //     {
        //         edge_open_list_.push(*it_edge);
        //     }
        // }
        
        // Update heuristic weight
        if (adaptive_)
        {
            // Get max e-value of current open list
            double max_e_value = std::numeric_limits<double>::min();

            // debug
            // size_t edge_key;
            // EdgePtrMapType::iterator it_edge;

            // Iterate through open list
            for(auto it_edge = edge_open_list_.begin(); it_edge != edge_open_list_.end(); it_edge++)
            {
                auto edge = *it_edge;
                if (edge->parent_state_ptr_->GetHValue() != 0 && edge->parent_state_ptr_->GetGValue() < best_cost_)
                {
                    double e_value = (best_cost_ - edge->parent_state_ptr_->GetGValue())/edge->parent_state_ptr_->GetHValue();
                    if (e_value > max_e_value)
                    {
                        max_e_value = e_value;

                        // debug
                        // edge_key = getEdgeKey(edge);
                    }
                }
            }

            // edge with max e-value
            // it_edge = edge_map_.find(edge_key);
            // it_edge->second->Print("Edge max e-value");
            // cout << it_edge->second->expansion_priority_ << endl;
            // cout << best_cost_ << endl;
            // if (best_cost_ + 1e-4 < it_edge->second->expansion_priority_)
            // {
            //     cout << "Rounding error????" << endl;
            // }
            // if (!edge_incon_list_.empty())
            // {}
            // else if (max_e_value != std::numeric_limits<double>::min())
            if (max_e_value != std::numeric_limits<double>::min())
            {
                heuristic_w_ = max_e_value;
                if (heuristic_w_ < 1)
                {
                    heuristic_w_ = 1;
                }
                // min_edge->parent_state_ptr_->Print("Min edge open");
                // cout << "Updated Heuristic weight: " << heuristic_w_ << endl;
            }
            else
            {
                break;
            }
        }
        else
        {
            heuristic_w_ -= delta_w_;
        }
        
        if (naive_)
        {
            edge_incon_list_.clear();
            while (!edge_open_list_.empty())
            {
                auto edge = edge_open_list_.min();
                edge_open_list_.pop();
            }

            resetStates();

            // Initialize start state
            start_state_ptr_->SetGValue(0);
            start_state_ptr_->SetHValue(computeHeuristic(start_state_ptr_));
            auto edge_ptr = Edge(start_state_ptr_, dummy_action_ptr_);
            auto edge_key = getEdgeKey(&edge_ptr);
            auto it_edge = edge_map_.find(edge_key);
            it_edge->second->expansion_priority_ = heuristic_w_*it_edge->second->parent_state_ptr_->GetHValue();
            start_state_ptr_->SetFValue(it_edge->second->expansion_priority_);

            edge_open_list_.push(it_edge->second);
        }
        else
        {
            EdgeQueueMinType edge_open_list;

            // append inconsistent list's edges into Eopen
            for(auto it_edge = edge_incon_list_.begin(); it_edge != edge_incon_list_.end(); it_edge++)
            {
                edge_open_list_.push(*it_edge);
            }

            edge_incon_list_.clear();
            
            while (!edge_open_list_.empty())
            {
                auto edge = edge_open_list_.min();
                edge_open_list_.pop();

                edge->parent_state_ptr_->SetFValue(edge->parent_state_ptr_->GetGValue() + heuristic_w_*edge->parent_state_ptr_->GetHValue());
                edge->expansion_priority_ = edge->parent_state_ptr_->GetFValue();
                
                if (edge->action_ptr_ != dummy_action_ptr_)
                {
                    auto edge_temp = Edge(edge->parent_state_ptr_, dummy_action_ptr_);
                    auto edge_key = getEdgeKey(&edge_temp);
                    edge = edge_map_.find(edge_key)->second;
                }

                // Prune if g(s) + h(s) > best_cost
                // if ((edge->parent_state_ptr_->GetGValue() + edge->parent_state_ptr_->GetHValue()) > best_cost_)
                // {
                //     continue;
                // }

                if (edge_open_list.contains(edge))
                {
                    edge_open_list.decrease(edge);
                }
                else
                {
                    edge_open_list.push(edge);
                }
            }
            edge_open_list_ = move(edge_open_list);
        }

        // Print min in open list
        if (DEBUG)
        {
            cout << "Min edge open: " << edge_open_list_.min()->expansion_priority_ << endl;
            cout << heuristic_w_ << endl;
        }
        // edge_open_list_.min()->Print("MIN EDGE OPEN");
    
    }
    
    if (EXPERIMENT)
    {
        string filename = "experiment_" + to_string(num_threads_) + "_aepase_" + to_string(heuristic_w) + "_" + to_string(delta_w_) + "_" + to_string(naive_) + "_" + to_string(adaptive_) + ".txt";
        std::ofstream newFile(filename, std::ios::app);
        for (auto data : data_list)
        {
            newFile << data << ",";
        }
        newFile << endl;
    }

    terminate_ = true;
    plan_ = best_plan_;
    // Reset heuristic weight & time budget
    heuristic_w_ = heuristic_w;
    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
    planner_stats_.total_time_ = 1e-9*t_elapsed;
    if (!plan_.empty()) {
        exit();
        return true;
    }
    exit();
    return false;
}

void AgepasePlanner::initialize()
{
    GepasePlanner::initialize();
    edge_incon_list_.clear();
    delta_w_ = 0.5;
    // delta_w_ = 1;
    best_cost_ = numeric_limits<double>::max();
}

void AgepasePlanner::improvePath() 
{
    vector<EdgePtrType> popped_edges;

    lock_.lock();

    while(!checkTimeout())
    {
        EdgePtrType curr_edge_ptr = NULL;

        while (!curr_edge_ptr && !terminate_)
        {
            // if (goal_state_ptr_ != NULL && !naive_)
            if (goal_state_ptr_ != NULL)
            {
                if (!edge_open_list_.empty())
                {
                    // Terminate condition: no state in open/be has f-value < goal's g-value
                    if (goal_state_ptr_->GetFValue() + 1e-1 < edge_open_list_.min()->expansion_priority_)
                    // if (goal_state_ptr_->GetFValue() < edge_open_list_.min()->expansion_priority_)
                    {
                        // Construct path
                        auto goal_state_ptr = goal_state_ptr_;
                        plan_.clear();
                        constructPlan(goal_state_ptr);
                        lock_.unlock();
                        exitMultiThread();
                        return;
                    }
                }
                else if (!being_expanded_states_.empty())
                {
                    // Terminate condition: no state in open/be has f-value < goal's g-value
                    if (goal_state_ptr_->GetFValue() + 1e-1 < being_expanded_states_.min()->GetFValue())
                    // if (goal_state_ptr_->GetFValue() < being_expanded_states_.min()->GetFValue())
                    {
                        // Construct path
                        auto goal_state_ptr = goal_state_ptr_;
                        plan_.clear();
                        constructPlan(goal_state_ptr);
                        lock_.unlock();
                        exitMultiThread();
                        return;
                    }
                }
                else // Edge case: when open is empty but goal_state just got set
                {
                    // Construct path
                    auto goal_state_ptr = goal_state_ptr_;
                    plan_.clear();
                    constructPlan(goal_state_ptr);
                    lock_.unlock();
                    exitMultiThread();
                    return;
                }
            }

            if (edge_open_list_.empty() && being_expanded_states_.empty())
            {
                // cout << "No solution found!\n";
                // cin.get();
                lock_.unlock();
                exitMultiThread();
                return;
            }

            while(!curr_edge_ptr && !edge_open_list_.empty())
            {
                curr_edge_ptr = edge_open_list_.min();
                edge_open_list_.pop();
                popped_edges.emplace_back(curr_edge_ptr);

                // If the parent state is being expanded, then all the outgoing edges are safe to expand
                if (curr_edge_ptr->parent_state_ptr_->IsBeingExpanded())
                    break;

                // Independence check of curr_edge with edges in BE
                for (auto& being_expanded_state : being_expanded_states_)
                {
                    if (being_expanded_state != curr_edge_ptr->parent_state_ptr_)
                    {
                        auto h_diff = computeHeuristic(being_expanded_state, curr_edge_ptr->parent_state_ptr_);
                        if (curr_edge_ptr->parent_state_ptr_->GetGValue() > being_expanded_state->GetGValue() + heuristic_w_*h_diff)
                        {
                            curr_edge_ptr = NULL;
                            break;
                        }
                    }
                }
     
                if (curr_edge_ptr)
                {
                    // Independence check of curr_edge with edges in OPEN that are in front of curr_edge
                    for (auto& popped_edge_ptr : popped_edges)
                    {
                        if (popped_edge_ptr->parent_state_ptr_ != curr_edge_ptr->parent_state_ptr_)
                        {
                            auto h_diff = computeHeuristic(popped_edge_ptr->parent_state_ptr_, curr_edge_ptr->parent_state_ptr_);
                            if (curr_edge_ptr->parent_state_ptr_->GetGValue() > popped_edge_ptr->parent_state_ptr_->GetGValue() + heuristic_w_*h_diff)
                            {
                                curr_edge_ptr = NULL;
                                break;
                            }                        
                        }
                    }
                }
            }
    

            // Re add the popped states except curr state which will be expanded now
            for (auto& popped_edge_ptr : popped_edges)
            {
                if (popped_edge_ptr != curr_edge_ptr)
                {
                    edge_open_list_.push(popped_edge_ptr);
                }
            }
            popped_edges.clear();

            if (!curr_edge_ptr)
            {
                lock_.unlock();
                // Wait for recheck_flag_ to be set true;
                unique_lock<mutex> locker(lock_);
                cv_.wait(locker, [this](){return (recheck_flag_ == true);});
                recheck_flag_ = false;
                locker.unlock();
                lock_.lock();
                continue;
            }

            
            // Update goal state ptr
            if (isGoalState(curr_edge_ptr->parent_state_ptr_) && (!terminate_))
            {
                if (goal_state_ptr_ == NULL)
                    goal_state_ptr_ = curr_edge_ptr->parent_state_ptr_;
                else if(curr_edge_ptr->parent_state_ptr_->GetFValue() < goal_state_ptr_->GetFValue())
                    goal_state_ptr_ = curr_edge_ptr->parent_state_ptr_;

                // if (naive_)
                // {
                //     // Construct path
                //     auto goal_state_ptr = goal_state_ptr_;
                //     plan_.clear();
                //     constructPlan(goal_state_ptr);
                //     lock_.unlock();
                //     exitMultiThread();
                //     return;
                // }
            }

        }

        // Insert the state in BE and mark it closed if the edge being expanded is dummy edge
        if (curr_edge_ptr->action_ptr_ == dummy_action_ptr_)
        {
            planner_stats_.num_state_expansions_++;  
            curr_edge_ptr->parent_state_ptr_->SetVisited();
            curr_edge_ptr->parent_state_ptr_->SetBeingExpanded();
            being_expanded_states_.push(curr_edge_ptr->parent_state_ptr_);
        }

        lock_.unlock();

        int thread_id = 0;
        bool edge_expansion_assigned = false;

        if (num_threads_ == 1)
        {
            expand(curr_edge_ptr, 0);
        }
        else
        {
            while (!edge_expansion_assigned)
            {
                unique_lock<mutex> locker(lock_vec_[thread_id]);
                bool status = edge_expansion_status_[thread_id];
                locker.unlock();

                if (!status)
                {
                    int num_threads_current = edge_expansion_futures_.size();
                    if (thread_id >= num_threads_current)
                    {
                        if (VERBOSE) cout << "Spawning edge expansion thread " << thread_id << endl;
                        edge_expansion_futures_.emplace_back(async(launch::async, &AgepasePlanner::expandEdgeLoop, this, thread_id));
                    }
                    locker.lock();
                    edge_expansion_vec_[thread_id] = curr_edge_ptr;
                    edge_expansion_status_[thread_id] = 1;
                    edge_expansion_assigned = true;       
                    locker.unlock();
                    cv_vec_[thread_id].notify_one();
                }
                else
                {
                    thread_id = thread_id == num_threads_-2 ? 0 : thread_id+1;
                }

            }
        }

        lock_.lock();

    }

    lock_.unlock();
    exitMultiThread();
    return;
}

void AgepasePlanner::expand(EdgePtrType edge_ptr, int thread_id)
{
    auto t_start = chrono::steady_clock::now();
    lock_.lock();
    auto t_lock_e = chrono::steady_clock::now();
    planner_stats_.lock_time_ += 1e-9*chrono::duration_cast<chrono::nanoseconds>(t_lock_e-t_start).count();

    planner_stats_.num_jobs_per_thread_[thread_id] +=1;
    
    // Proxy edge, add the real edges to Eopen
    if (edge_ptr->action_ptr_ == dummy_action_ptr_)
    {       
        auto state_ptr = edge_ptr->parent_state_ptr_;
        // state_ptr->num_successors_ = 0;
        // state_ptr->num_expanded_successors_ = 0;   
        state_ptr->SetVValue(state_ptr->GetGValue());

        for (auto& action_ptr: actions_ptrs_)
        {
            if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
            {
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

                edge_ptr_next->expansion_priority_ = edge_ptr->expansion_priority_;
                state_ptr->num_successors_+=1;

                if (action_ptr->IsExpensive())
                {
                    if (VERBOSE) cout << "Pushing successor with g_val: " << state_ptr->GetGValue() << " | h_val: " << state_ptr->GetHValue() << endl;
                    if (edge_open_list_.contains(edge_ptr_next))
                    {
                        edge_open_list_.decrease(edge_ptr_next);
                    }
                    else
                    {
                        edge_open_list_.push(edge_ptr_next);
                    }
                    notifyMainThread();
                }
                else
                {
                    expandEdge(edge_ptr_next, thread_id);
                }
            }

        }
       
    }
    else // Real edge, evaluate and add proxy edges for child 
    {        
        expandEdge(edge_ptr, thread_id);
    }


    if (edge_ptr->parent_state_ptr_->num_expanded_successors_ == edge_ptr->parent_state_ptr_->num_successors_)
    {
        edge_ptr->parent_state_ptr_->UnsetBeingExpanded();
        if (being_expanded_states_.contains(edge_ptr->parent_state_ptr_))
        {   
            being_expanded_states_.erase(edge_ptr->parent_state_ptr_);
            notifyMainThread();
        }
    }

    if (edge_ptr->parent_state_ptr_->num_expanded_successors_ > edge_ptr->parent_state_ptr_->num_successors_)
    {
        edge_ptr->parent_state_ptr_->Print();
        throw runtime_error("Number of expanded edges cannot be greater than number of successors");
    }
    else
    {
        if (VERBOSE) edge_ptr->Print("Expansion completed ");
    }

    auto t_end_expansion = chrono::steady_clock::now();
    planner_stats_.cumulative_expansions_time_ += 1e-9*chrono::duration_cast<chrono::nanoseconds>(t_end_expansion-t_start).count();

    lock_.unlock();
}

void AgepasePlanner::expandEdge(EdgePtrType edge_ptr, int thread_id)
{
    if (!edge_ptr->is_eval_)
    {
        auto action_ptr = edge_ptr->action_ptr_;

        lock_.unlock();
        // Evaluate the edge
        auto t_start = chrono::steady_clock::now();
        auto action_successor = action_ptr->GetSuccessor(edge_ptr->parent_state_ptr_->GetStateVars(), thread_id);
        auto t_end = chrono::steady_clock::now();
        
        auto t_lock_s = chrono::steady_clock::now();
        lock_.lock();
        auto t_lock_e = chrono::steady_clock::now();
        planner_stats_.lock_time_ += 1e-9*chrono::duration_cast<chrono::nanoseconds>(t_lock_e-t_lock_s).count();

        edge_ptr->is_eval_ = true;
        planner_stats_.num_evaluated_edges_++; // Only the edges controllers that satisfied pre-conditions and args are in the open list

        if (action_successor.success_)
        {
            auto successor_state_ptr = constructState(action_successor.successor_state_vars_costs_.back().first);
            double cost = action_successor.successor_state_vars_costs_.back().second;                

            // Set successor and cost in expanded edge
            edge_ptr->child_state_ptr_ = successor_state_ptr;
            edge_ptr->SetCost(cost);

            double new_g_val = edge_ptr->parent_state_ptr_->GetGValue() + cost;

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
                    
                    // Insert poxy edge
                    auto edge_temp = Edge(successor_state_ptr, dummy_action_ptr_);
                    auto edge_key = getEdgeKey(&edge_temp);
                    auto it_edge = edge_map_.find(edge_key); 
                    EdgePtrType proxy_edge_ptr;

                    if (it_edge == edge_map_.end())
                    {
                        proxy_edge_ptr = new Edge(successor_state_ptr, dummy_action_ptr_);
                        edge_map_.insert(make_pair(edge_key, proxy_edge_ptr));
                    }
                    else
                    {
                        proxy_edge_ptr = it_edge->second;
                    }

                    proxy_edge_ptr->expansion_priority_ = new_g_val + heuristic_w_*h_val;
                    
                    if (!successor_state_ptr->IsVisited())
                    {
                        if (edge_open_list_.contains(proxy_edge_ptr))
                        {
                            edge_open_list_.decrease(proxy_edge_ptr);
                        }
                        else
                        {
                            edge_open_list_.push(proxy_edge_ptr);
                        }
                    }
                    else
                    {
                        if (find(edge_incon_list_.begin(), edge_incon_list_.end(), proxy_edge_ptr) == edge_incon_list_.end())
                        {
                            edge_incon_list_.emplace_back(proxy_edge_ptr);
                        }
                    }

                    notifyMainThread();
                }       
            }
        }
        else
        {
            edge_ptr->is_invalid_ = true;
            if (VERBOSE) edge_ptr->Print("No successors for");
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
                
                // Insert poxy edge
                auto edge_temp = Edge(successor_state_ptr, dummy_action_ptr_);
                auto edge_key = getEdgeKey(&edge_temp);
                auto it_edge = edge_map_.find(edge_key); 
                EdgePtrType proxy_edge_ptr;

                if (it_edge == edge_map_.end())
                {
                    proxy_edge_ptr = new Edge(successor_state_ptr, dummy_action_ptr_);
                    edge_map_.insert(make_pair(edge_key, proxy_edge_ptr));
                }
                else
                {
                    proxy_edge_ptr = it_edge->second;
                }

                proxy_edge_ptr->expansion_priority_ = new_g_val + heuristic_w_*h_val;
                
                if (!successor_state_ptr->IsVisited())
                {
                    if (edge_open_list_.contains(proxy_edge_ptr))
                    {
                        edge_open_list_.decrease(proxy_edge_ptr);
                    }
                    else
                    {
                        edge_open_list_.push(proxy_edge_ptr);
                    }
                }
                else
                {
                    if (find(edge_incon_list_.begin(), edge_incon_list_.end(), proxy_edge_ptr) == edge_incon_list_.end())
                    {
                        edge_incon_list_.emplace_back(proxy_edge_ptr);
                    }
                }

                notifyMainThread();
            }       
        }
    }

    edge_ptr->parent_state_ptr_->num_expanded_successors_ += 1;

}

void AgepasePlanner::exit()
{
    terminate_ = true;

    for (int thread_id = 0; thread_id < num_threads_-1; ++thread_id)
    {
        unique_lock<mutex> locker(lock_vec_[thread_id]);
        edge_expansion_status_[thread_id] = 1;
        locker.unlock();
        cv_vec_[thread_id].notify_one();
    }

    planner_stats_.num_threads_spawned_ = edge_expansion_futures_.size()+1;
    bool all_expansion_threads_terminated = false;
    while (!all_expansion_threads_terminated)
    {
        all_expansion_threads_terminated = true;
        int i = 0;
        for (auto& fut : edge_expansion_futures_)
        {
            if (!isFutureReady(fut))
            {
                all_expansion_threads_terminated = false;
                break;
            }
            i++;
        }
    }
    edge_expansion_futures_.clear();

    edge_incon_list_.clear();
    
    while (!edge_open_list_.empty())
    {
        edge_open_list_.pop();
    }
    being_expanded_states_.clear();

    Planner::exit();
}

void AgepasePlanner::exitMultiThread()
{
    bool all_expansion_threads_terminated = false;
    while  (!all_expansion_threads_terminated)
    {
        all_expansion_threads_terminated = true;
        for (int thread_id = 0; thread_id < num_threads_-1; ++thread_id)
        {
            unique_lock<mutex> locker(lock_vec_[thread_id]);
            auto status = edge_expansion_status_[thread_id];
            if (status)
            {
                all_expansion_threads_terminated = false;
                break;
            }
        
        }
    }
}