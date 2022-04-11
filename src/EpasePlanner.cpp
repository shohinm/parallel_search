#include <iostream>
#include "EpasePlanner.hpp"

#define VERBOSE 0

using namespace std;
using namespace epase;

EpasePlanner::EpasePlanner(ParamsType planner_params):
Planner(planner_params)
{    
    num_threads_  = planner_params["num_threads"];
    vector<LockType> lock_vec(num_threads_-1);
    lock_vec_.swap(lock_vec);
}

EpasePlanner::~EpasePlanner()
{
    
}

bool EpasePlanner::Plan(int exp_idx)
{
    
    initialize();    
    auto t_start = chrono::system_clock::now();
    
    vector<Edge*> popped_edges;

    lock_.lock();

    while(!terminate_)
    {
        Edge* curr_edge_ptr = NULL;

        while (!curr_edge_ptr && !terminate_)
        {
            if (edge_open_list_.empty() && being_expanded_states_.empty())
            {
                terminate_ = true;
                auto t_end = chrono::system_clock::now();
                double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
                total_time_ = 1e-9*t_elapsed;
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

                // state_to_expand_found = true;

                if (curr_edge_ptr->parent_state_ptr_->IsBeingExpanded())
                    continue;

                // Independence check of curr_edge with edges in OPEN that are in front of curr_edge
                for (auto& popped_edge_ptr : popped_edges)
                {
                    auto h_diff = computeHeuristic(popped_edge_ptr->parent_state_ptr_, curr_edge_ptr->parent_state_ptr_);
                    // if (curr_edge_ptr->GetGValue() > popped_edge_ptr->GetGValue() + heuristic_w_*(popped_edge_ptr->GetHValue() -  curr_edge_ptr->GetHValue()))
                    if (curr_edge_ptr->parent_state_ptr_->GetGValue() > popped_edge_ptr->parent_state_ptr_->GetGValue() + heuristic_w_*h_diff)
                    {
                        // state_to_expand_found = false;
                        curr_edge_ptr = NULL;
                        break;
                    }
                }
     
                if (curr_edge_ptr)
                {
                    // Independence check of curr_edge with edges in BE
                    for (auto& being_expanded_state : being_expanded_states_)
                    {
                        auto h_diff = computeHeuristic(being_expanded_state, curr_edge_ptr->parent_state_ptr_);
                        if (curr_edge_ptr->parent_state_ptr_->GetGValue() > being_expanded_state->GetGValue() + heuristic_w_*h_diff)
                        {
                            // state_to_expand_found = false;
                            curr_edge_ptr = NULL;
                            break;
                        }
                    }
                }
            }
    

            // Re add the popped states except curr state which will be expanded now
            for (auto& popped_edge_ptr : popped_edges)
            {
                if (popped_edge_ptr != curr_edge_ptr)
                    edge_open_list_.push(popped_edge_ptr);
            }
            popped_edges.clear();

            if (!curr_edge_ptr)
            {
                lock_.unlock();
                // Wait for recheck_flag_ to be set true;
                while(!recheck_flag_ && !terminate_){}
                lock_.lock();    
                continue;
            }

            // if (!terminate_)
            recheck_flag_ = false;
            
            // Return solution if goal state is expanded
            // if (isGoalState(curr_edge_ptr) && !terminate_)
            if (isGoalState(curr_edge_ptr->parent_state_ptr_) && (!terminate_))
            {
                auto t_end = chrono::system_clock::now();
                double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
                total_time_ = 1e-9*t_elapsed;

                // Reconstruct and return path
                cout << "--------------------------------------------------------" << endl;            
                cout << "Goal Reached!" << endl;
                // cout << "Number of edge expansions threads spawned: " << edge_expansion_futures_.size() << endl;
                cout << "--------------------------------------------------------" << endl;            
                goal_state_ptr_ = curr_edge_ptr->parent_state_ptr_;
                constructPlan(curr_edge_ptr->parent_state_ptr_);   
                terminate_ = true;
                recheck_flag_ = true;
                lock_.unlock();
                exit();
                cout << "--------------------------------------------------------" << endl;            

                return true;
            }
            
        }

        // Insert the state in BE and mark it closed if the edge being expanded is dummy edge
        if (curr_edge_ptr->action_ptr_ == dummy_action_ptr_)
        {
            num_state_expansions_++;  
            curr_edge_ptr->parent_state_ptr_->SetVisited();
            curr_edge_ptr->parent_state_ptr_->SetBeingExpanded();
            being_expanded_states_.emplace_back(curr_edge_ptr->parent_state_ptr_);
        }

        lock_.unlock();

        // cout << "____________________" << endl;
        // cout << "Open list size: " << edge_open_list_.size() << endl; 
        // cout << "BE size: " << being_expanded_states_.size() << endl;
        // cout << "____________________" << endl;

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
                lock_vec_[thread_id].lock();
                bool status = edge_expansion_status_[thread_id];
                lock_vec_[thread_id].unlock();

                if (!status)
                {
                    int num_threads_current = edge_expansion_futures_.size();
                    if (thread_id >= num_threads_current)
                    {
                        // cout << "Spawning thread " << thread_id << endl;
                        edge_expansion_futures_.emplace_back(async(launch::async, &EpasePlanner::expandEdgeLoop, this, thread_id));
                        // cout << "New threads size: " << edge_expansion_futures_.size() << endl;
                    }
                    lock_vec_[thread_id].lock();
                    edge_expansion_vec_[thread_id] = curr_edge_ptr;
                    edge_expansion_status_[thread_id] = 1;
                    edge_expansion_assigned = true;       
                    lock_vec_[thread_id].unlock();
                }
                else
                    thread_id = thread_id == num_threads_-2 ? 0 : thread_id+1;

            }
        }

        lock_.lock();


    }

    terminate_ = true;
    auto t_end = chrono::system_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
    total_time_ = 1e-9*t_elapsed;
    cout << "Goal Not Reached, Number of states expanded: " << num_state_expansions_ << endl;   
    lock_.unlock();
    exit();
    return false;



}

void EpasePlanner::initialize()
{
    Planner::initialize();
    terminate_ = false;
    recheck_flag_ = true;

    edge_expansion_vec_.clear();
    edge_expansion_vec_.resize(num_threads_-1, NULL);
    
    edge_expansion_status_.clear();
    edge_expansion_status_.resize(num_threads_-1, 0);

    edge_expansion_futures_.clear();
 
    edge_open_list_ = EdgeQueueMinType();
    being_expanded_states_.clear();

    // Insert proxy edge with start state
    // dummy_action_ptr_ = make_shared<Action>("dummy");
    dummy_action_ptr_ = NULL;
    auto edge_ptr = new Edge(start_state_ptr_, dummy_action_ptr_);
    edge_ptr->expansion_priority_ = heuristic_w_*computeHeuristic(start_state_ptr_);

    edge_map_.insert(make_pair(getEdgeKey(edge_ptr), edge_ptr));
    edge_open_list_.push(edge_ptr);   


}

void EpasePlanner::expandEdgeLoop(int thread_id)
{
    while (!terminate_)
    {

        lock_vec_[thread_id].lock();
        bool status = edge_expansion_status_[thread_id];
        lock_vec_[thread_id].unlock();

        while ((!status) && (!terminate_))
        {
            lock_vec_[thread_id].lock();
            status = edge_expansion_status_[thread_id];
            lock_vec_[thread_id].unlock();
            // cout << "Expansion thread " << thread_id << " waiting! " << edge_expansion_status_[thread_id] << endl;
        }

        if (terminate_)
            break;


        expandEdge(edge_expansion_vec_[thread_id], thread_id);

        lock_vec_[thread_id].lock();
        edge_expansion_vec_[thread_id] = NULL;
        edge_expansion_status_[thread_id] = 0;
        lock_vec_[thread_id].unlock();

    }    
}

void EpasePlanner::expandEdge(Edge* edge_ptr, int thread_id)
{
    lock_.lock();

    if (VERBOSE)
        edge_ptr->Print("Expanding");

   
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

                if (VERBOSE)
                    cout << "Pushing successor with g_val: " << state_ptr->GetGValue() << " | h_val: " << state_ptr->GetHValue() << endl;
               
                state_ptr->num_successors_+=1;
                edge_open_list_.push(edge_ptr_real);
            }
        }
       
        // num_proxy_expansions_++;
        recheck_flag_ = true;
    }
    else // Real edge, evaluate and add proxy edges for child 
    {        
        
        auto action_ptr = edge_ptr->action_ptr_;

        lock_.unlock();
        // Evaluate the edge
        auto t_start = chrono::system_clock::now();
        auto action_successor = action_ptr->Apply(state_ptr->GetStateVars(), thread_id);
        auto t_end = chrono::system_clock::now();
        num_evaluated_edges_++; // Only the edges controllers that satisfied pre-conditions and args are in the open list
        //********************
        lock_.lock();

        if (action_successor.success_)
        {
            auto successor_state_ptr = constructState(action_successor.successor_state_vars_costs_.back().first);

            if (!successor_state_ptr->IsVisited())
            {
                double cost = action_successor.successor_state_vars_costs_.back().second;                
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
                        successor_state_ptr->SetGValue(new_g_val);
                        successor_state_ptr->SetIncomingEdgePtr(edge_ptr);
                        
                        // Insert poxy edge
                        auto edge_temp = Edge(successor_state_ptr, dummy_action_ptr_);
                        auto edge_key = getEdgeKey(&edge_temp);
                        auto it_edge = edge_map_.find(edge_key); 
                        Edge* proxy_edge_ptr;

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

                    }

                }       
            }
        }
        else
        {
            if (VERBOSE)
            {
                cout << "Edge returned no successors ";
                edge_ptr->Print();
            }


        }

        edge_ptr->parent_state_ptr_->num_expanded_successors_ += 1;

        if (edge_ptr->parent_state_ptr_->num_expanded_successors_ == edge_ptr->parent_state_ptr_->num_successors_)
        {
            edge_ptr->parent_state_ptr_->UnsetBeingExpanded();
            auto it_state_be = find(being_expanded_states_.begin(), being_expanded_states_.end(), edge_ptr->parent_state_ptr_);
            if (it_state_be != being_expanded_states_.end())
                being_expanded_states_.erase(it_state_be);
        }

        if (edge_ptr->parent_state_ptr_->num_expanded_successors_ > edge_ptr->parent_state_ptr_->num_successors_)
        {
            edge_ptr->parent_state_ptr_->Print();
            throw runtime_error("Number of expanded edges cannot be greater than number of successors");
        }


        recheck_flag_ = true;

    } // if (!edge_ptr->gac_.controller_)
    

    lock_.unlock();

}

void EpasePlanner::exit()
{
    bool all_expansion_threads_terminated = false;
    while (!all_expansion_threads_terminated)
    {
        all_expansion_threads_terminated = true;
        for (auto& fut : edge_expansion_futures_)
        {
            if (!isFutureReady(fut))
            {
                all_expansion_threads_terminated = false;
                break;
            }
        }
    }
    edge_expansion_futures_.clear();

    Planner::exit();
}
