#include <iostream>
#include <algorithm>
#include <planners/EpasePlanner.hpp>

using namespace std;
using namespace ps;

EpasePlanner::EpasePlanner(ParamsType planner_params):
GepasePlanner(planner_params),
Planner(planner_params)
{    

}

EpasePlanner::~EpasePlanner()
{
    
}

bool EpasePlanner::Plan()
{
    
    initialize();    
    
    vector<EdgePtrType> popped_edges;

    lock_.lock();
    startTimer();
    while(!terminate_ && !checkTimeout())
    {
        EdgePtrType curr_edge_ptr = NULL;

        while (!curr_edge_ptr && !terminate_)
        {
            if (edge_open_list_.empty() && being_expanded_states_.empty())
            {
                terminate_ = true;
                auto t_end = chrono::steady_clock::now();
                double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
                planner_stats_.total_time_ = 1e-9*t_elapsed;
                cout << "Goal Not Reached" << endl;   
                lock_.unlock();
                exit();
                return false;
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

            
            // Return solution if goal state is expanded
            if (isGoalState(curr_edge_ptr->parent_state_ptr_) && (!terminate_))
            {

                // cout << "--------------------------------------------------------" << endl;            
                // cout << "Goal Reached!" << endl;
                // cout << "--------------------------------------------------------" << endl;            
                
                // Construct path
                goal_state_ptr_ = curr_edge_ptr->parent_state_ptr_;
                constructPlan(goal_state_ptr_);   
                terminate_ = true;
                recheck_flag_ = true;

                auto t_end = chrono::steady_clock::now();
                double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
                planner_stats_.total_time_ = 1e-9*t_elapsed;

                lock_.unlock();
                exit();

                return true;
            }
            
        }

        // Insert the state in BE and mark it closed if the edge being expanded is dummy edge
        if (curr_edge_ptr->action_ptr_ == dummy_action_ptr_)
        {
            planner_stats_.num_state_expansions_++;  
            curr_edge_ptr->parent_state_ptr_->SetVisited();
            curr_edge_ptr->parent_state_ptr_->SetBeingExpanded();
            being_expanded_states_.emplace_back(curr_edge_ptr->parent_state_ptr_);
        }

        lock_.unlock();

        int thread_id = 0;
        bool edge_expansion_assigned = false;

        if (num_threads_ == 1)
        {
            expandEdge(curr_edge_ptr, 0);
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
                        edge_expansion_futures_.emplace_back(async(launch::async, &EpasePlanner::expandEdgeLoop, this, thread_id));
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

    terminate_ = true;
    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
    planner_stats_.total_time_ = 1e-9*t_elapsed;
    // cout << "Goal Not Reached, Number of states expanded: " << planner_stats_.num_state_expansions_ << endl;   
    lock_.unlock();
    exit();
    return false;
}

void EpasePlanner::expandEdgeLoop(int thread_id)
{
    while (!terminate_)
    {
        unique_lock<mutex> locker(lock_vec_[thread_id]);
        cv_vec_[thread_id].wait(locker, [this, thread_id](){return (edge_expansion_status_[thread_id] == 1);});
        locker.unlock();

        if (terminate_)
            break;

        expandEdge(edge_expansion_vec_[thread_id], thread_id);

        locker.lock();
        edge_expansion_vec_[thread_id] = NULL;
        edge_expansion_status_[thread_id] = 0;
        locker.unlock();

    }    
}

void EpasePlanner::expandEdge(EdgePtrType edge_ptr, int thread_id)
{
    auto t_start = chrono::steady_clock::now();
    lock_.lock();
    auto t_lock_e = chrono::steady_clock::now();
    planner_stats_.lock_time_ += 1e-9*chrono::duration_cast<chrono::nanoseconds>(t_lock_e-t_start).count();

    if (VERBOSE) edge_ptr->Print("Expanding");

    planner_stats_.num_jobs_per_thread_[thread_id] +=1;

    auto state_ptr = edge_ptr->parent_state_ptr_;
    
    // Proxy edge, add the real edges to Eopen
    if (edge_ptr->action_ptr_ == dummy_action_ptr_)
    {       
        for (auto& action_ptr: actions_ptrs_)
        {
            if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
            {
                auto edge_ptr_real = new Edge(state_ptr, action_ptr);
                edge_map_.insert(make_pair(getEdgeKey(edge_ptr_real), edge_ptr_real));

                // edge_ptr_real->exp_priority_ = state_ptr->GetGValue() + heuristic_w_*state_ptr->GetHValue();
                edge_ptr_real->expansion_priority_ = edge_ptr->expansion_priority_;

                if (VERBOSE) cout << "Pushing successor with g_val: " << state_ptr->GetGValue() << " | h_val: " << state_ptr->GetHValue() << endl;
               
                state_ptr->num_successors_+=1;
                edge_open_list_.push(edge_ptr_real);
                notifyMainThread();
            }
        }
    }
    else // Real edge, evaluate and add proxy edges for child 
    {        
        
        auto action_ptr = edge_ptr->action_ptr_;

        lock_.unlock();
        // Evaluate the edge
        auto t_start = chrono::steady_clock::now();
        auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars(), thread_id);
        auto t_end = chrono::steady_clock::now();
        //********************
        
        auto t_lock_s = chrono::steady_clock::now();
        lock_.lock();
        auto t_lock_e = chrono::steady_clock::now();

        planner_stats_.action_eval_times_[action_ptr->GetType()].emplace_back(1e-9*chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count());
        planner_stats_.lock_time_ += 1e-9*chrono::duration_cast<chrono::nanoseconds>(t_lock_e-t_lock_s).count();

        planner_stats_.num_evaluated_edges_++; // Only the edges controllers that satisfied pre-conditions and args are in the open list

        if (action_successor.success_)
        {
            auto successor_state_ptr = constructState(action_successor.successor_state_vars_costs_.back().first);
            double cost = action_successor.successor_state_vars_costs_.back().second;                

            // Set successor and cost in expanded edge
            edge_ptr->child_state_ptr_ = successor_state_ptr;
            edge_ptr->SetCost(cost);

            if (!successor_state_ptr->IsVisited())
            {
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
                        
                        if (edge_open_list_.contains(proxy_edge_ptr))
                        {
                            edge_open_list_.decrease(proxy_edge_ptr);
                        }
                        else
                        {
                            edge_open_list_.push(proxy_edge_ptr);
                        }

                        notifyMainThread();
                    }

                }       
            }
        }
        else
        {
            if (VERBOSE) edge_ptr->Print("No successors for");
        }

        edge_ptr->parent_state_ptr_->num_expanded_successors_ += 1;

        if (edge_ptr->parent_state_ptr_->num_expanded_successors_ == edge_ptr->parent_state_ptr_->num_successors_)
        {
            edge_ptr->parent_state_ptr_->UnsetBeingExpanded();
            auto it_state_be = find(being_expanded_states_.begin(), being_expanded_states_.end(), edge_ptr->parent_state_ptr_);
            if (it_state_be != being_expanded_states_.end())
            {
                being_expanded_states_.erase(it_state_be);
                notifyMainThread();
            }
        }

        if (edge_ptr->parent_state_ptr_->num_expanded_successors_ > edge_ptr->parent_state_ptr_->num_successors_)
        {
            edge_ptr->parent_state_ptr_->Print();
            throw runtime_error("Number of expanded edges cannot be greater than number of successors");
        }

    }

    auto t_end = chrono::steady_clock::now();
    planner_stats_.cumulative_expansions_time_ += 1e-9*chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();

    lock_.unlock();
}


void EpasePlanner::exit()
{
    being_expanded_states_.clear();
    GepasePlanner::exit();
}