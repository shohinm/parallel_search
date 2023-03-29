#include <iostream>
#include <algorithm>
#include <planners/MplpPlanner.hpp>

using namespace std;
using namespace ps;

MplpPlanner::MplpPlanner(ParamsType planner_params):
Planner(planner_params)
{    
    num_threads_  = planner_params["num_threads"];
    vector<LockType> lock_vec(num_threads_);
    lock_vec_.swap(lock_vec);
}

MplpPlanner::~MplpPlanner()
{
    
}

bool MplpPlanner::Plan()
{    
    initialize();    
    startTimer();   

    delegate_edges_process_ = shared_ptr<thread>(new thread(&MplpPlanner::delegateEdges, this));
    monitor_paths_process_ = shared_ptr<thread>(new thread(&MplpPlanner::monitorPaths, this));    
    planner_stats_.num_threads_spawned = 3;

    int plan_idx = 0;
    bool path_exists = true;
    while (!plan_found_ and path_exists)
    {

        lock_2_.lock();
        plan_evaluation_outcomes_.resize(plan_idx+1);
        plan_evaluation_outcomes_[plan_idx] = -1;
        lock_2_.unlock();

        path_exists = replanMPLP();

        auto t_end = chrono::steady_clock::now();
        double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
        plan_idx++;

    }

    if (VERBOSE)
    {
        if (plan_found_)
        {
            cout << "Success! Valid plan found! " << endl;
        }
        else
        {
            cout << "Failure! Valid plan not found! " << endl;
        }
    }
    
    terminate_ = true;

    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start_).count();
    planner_stats_.total_time = 1e-9*t_elapsed;

    exit();

    return plan_found_;
}

void MplpPlanner::initialize()
{
    Planner::initialize();
    planner_stats_.num_jobs_per_thread.resize(num_threads_, 0);

    terminate_ = false;
    plan_found_ = false;

    edge_evaluation_vec_.clear();
    edge_evaluation_vec_.resize(num_threads_-1, NULL);
    
    edge_evaluation_status_.clear();
    edge_evaluation_status_.resize(num_threads_-1, 0);

    edge_evaluation_futures_.clear();

    lazy_plans_.clear();
    plan_evaluation_outcomes_.clear();
    successful_plan_idx_ = -1;
    cost_bound_ = -1;
}

void MplpPlanner::initializeReplanning()
{
    resetStates();

    while (!state_open_list_.empty())
    {
        state_open_list_.pop();
    }
    
    start_state_ptr_->SetGValue(0);
    state_open_list_.push(start_state_ptr_);
}

bool MplpPlanner::replanMPLP()
{
    if (VERBOSE) cout << "------------ Replanning MPLP ------------" << endl;

    auto t_start_replanning = chrono::steady_clock::now();
    
    initializeReplanning(); 
    int num_expansions = 0;

    while (!state_open_list_.empty() && !checkTimeout())
    {

        if (plan_found_)
        {
            // auto t_end_replanning = chrono::steady_clock::now();
            // double t_elapsed_replanning = chrono::duration_cast<chrono::nanoseconds>(t_end_replanning-t_start_replanning).count();
            // replanning_times_.emplace_back(1e-9*t_elapsed_replanning);
            return true;
        }

        // if ((PRINT_EXPANSIONS)&&(num_expansions%100 == 0))
        //     cout << "Expansion: " << num_expansions << endl;
    
        auto state_ptr = state_open_list_.min();
        state_open_list_.pop();

        // Return solution if goal state is expanded
        if (isGoalState(state_ptr))
        {
            if (VERBOSE) cout << "Goal Reached, Number of states expanded: " << num_expansions << endl;
    
            goal_state_ptr_ = state_ptr;
            
            lock_2_.lock();
            cost_bound_ = max(cost_bound_, goal_state_ptr_->GetGValue());
            lock_2_.unlock();

            // auto t_end_replanning = chrono::steady_clock::now();
            // double t_elapsed_replanning = chrono::duration_cast<chrono::nanoseconds>(t_end_replanning-t_start_replanning).count();
            // replanning_times_.emplace_back(1e-9*t_elapsed_replanning);

            // Reconstruct and return path
            constructPlan(goal_state_ptr_);   
            return true;
        }

        expandState(state_ptr);
        state_ptr->SetVisited();
    
        if (VERBOSE)
        {
            cout << "Open list size: " << state_open_list_.size() << endl; 
            cout << "EQ size: " << edges_open_.size() << endl;         
            cout << "Graph size: " << state_map_.size() << endl;
            cout << "State with min h_val in Open list: " << h_val_min_ << endl;            
        }

        num_expansions++;
        // auto t_end = chrono::steady_clock::now();
        // double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
        // if ((timeout_>0) && (1e-9*t_elapsed > timeout_))
        //     return false;
    }

    // cout << "Goal not reached Number of states expanded: " << num_expansions << endl;
    // auto t_end_replanning = chrono::steady_clock::now();
    // double t_elapsed_replanning = chrono::duration_cast<chrono::nanoseconds>(t_end_replanning-t_start_replanning).count();
    // replanning_times_.emplace_back(1e-9*t_elapsed_replanning);
    return false;

}

void MplpPlanner::expandState(StatePtrType state_ptr)
{

    if (VERBOSE) state_ptr->Print("Expanding");

    bool state_expanded_before = false;

    for (auto& action_ptr: actions_ptrs_)
    {

        EdgePtrType edge_ptr = NULL;
        edge_ptr = new Edge(state_ptr, action_ptr);
        auto edge_key = getEdgeKey(edge_ptr);
        delete edge_ptr;
        edge_ptr = NULL;        
        // Don't need a lock since no other thread is adding to edge_map_ except this. Which means
        // that when this line is being executed, no thread is modifying (writing) edge_map_.
        auto it_edge = edge_map_.find(edge_key); 
       
        bool edge_generated = true;

        if (it_edge == edge_map_.end())
        {
            if (VERBOSE) cout << "Expand: Edge not generated " << endl;
            edge_generated = false;
        }
        else if (it_edge->second->is_closed_)
        {
            // Edge in Eclosed
            if (VERBOSE) cout << "Expand: Edge in closed " << endl;
            edge_ptr = it_edge->second;
        }
        else if (edges_open_.contains(it_edge->second))
        {
            // Edge in Eopen
            if (VERBOSE) cout << "Expand: Edge in open " << endl;
            edge_ptr = it_edge->second;
        }
        else if (it_edge->second->is_eval_)
        {
            // Edge in Eeval
            if (VERBOSE) cout << "Expand: Edge in eval " << endl;
            edge_ptr = it_edge->second;
        }
        else if (it_edge->second->is_invalid_)
        {
            if (VERBOSE) cout << "Invalid edge" << endl;
            continue;
        }            

        if (!edge_generated && action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
        {

            // Evaluate the edge
            auto t_start = chrono::steady_clock::now();
            auto action_successor = action_ptr->GetSuccessorLazy(state_ptr->GetStateVars());
            auto t_end = chrono::steady_clock::now();
            //********************

            // Only the actions that satisfied pre-conditions and args are in the open list

            if (action_successor.success_)
            {
                auto successor_state_ptr = constructState(action_successor.successor_state_vars_costs_.back().first);                            
                edge_ptr = new Edge(state_ptr, successor_state_ptr, action_ptr);
                edge_ptr->SetCost(action_successor.successor_state_vars_costs_.back().second);
                assignEdgePriority(edge_ptr);

                lock_.lock();
                edge_map_.insert(make_pair(getEdgeKey(edge_ptr), edge_ptr));                            
                edges_open_.push(edge_ptr);
                lock_.unlock();
            
            }
        }

        if (edge_ptr && edge_ptr->child_state_ptr_ && (!edge_ptr->child_state_ptr_->IsVisited()))
        {
            auto successor_state_ptr = edge_ptr->child_state_ptr_;
            double new_g_val = state_ptr->GetGValue() + edge_ptr->GetCost();
            
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
        else
        {
            // Insert into Einvalid if no valid successor is generated
            edge_ptr = new Edge(state_ptr, action_ptr);
            edge_ptr->SetCost(DINF);
            edge_ptr->is_invalid_ = true;

            // Insert invalid edge into edge_map_. edge_map_ insert has to be under lock because edge_map_.find is happening
            // in updateEdgeCbk
            lock_.lock();
            edge_map_.insert(make_pair(getEdgeKey(edge_ptr), edge_ptr));
            lock_.unlock();
        }    

        if (edge_generated)
            state_expanded_before = true;

    }
    
    // if (state_expanded_before)
    // {
    //     num_reexpansions += 1;
    //     t_reexpansion_ += 1e-9*t_elapsed_expand;
    // }
    // else
    // {
    //     num_firstexpansions += 1;
    //     t_firstexpansion_ += 1e-9*t_elapsed_expand;
    // }

}

void MplpPlanner::delegateEdges()
{
    while (!terminate_)
    {
        if (edges_open_.size() == 0)
        {
            continue;            
        }

        lock_.lock();
        auto edge_ptr = edges_open_.min();
        edges_open_.pop();
        lock_.unlock();

        edge_ptr->is_eval_= true;

        // Find a free thread, spawn if necessary
        int thread_id = 3; // T0: lazy search, T1: monitorPaths, T2: delegateEdges
        bool edge_evaluation_assigned = false;
       
        while(!edge_evaluation_assigned)
        {
            lock_vec_[thread_id].lock();
            // cout << "Checking thread " << thread_id << endl;
            bool status = edge_evaluation_status_[thread_id];
            lock_vec_[thread_id].unlock();

            if (!status)
            {

                if (thread_id-3 >= edge_evaluation_futures_.size())
                {
                    if (VERBOSE) cout << "Spawning edge evaluation thread " << thread_id << endl;
                    edge_evaluation_futures_.emplace_back(async(launch::async, &MplpPlanner::evaluateEdgeLoop, this, thread_id));
                    planner_stats_.num_threads_spawned+=1;
                }

                if (VERBOSE) edge_ptr->Print("Delegating");

                lock_vec_[thread_id].lock();
                edge_evaluation_vec_[thread_id] = edge_ptr;
                edge_evaluation_status_[thread_id] = 1;
                edge_evaluation_assigned = true;       
                lock_vec_[thread_id].unlock();

            }
            else
            {
                thread_id = thread_id == num_threads_-1 ? 3 : thread_id+1;
            }
        }
    }
}

void MplpPlanner::evaluateEdge(EdgePtrType edge_ptr, int thread_id)
{
    lock_.lock();
    planner_stats_.num_evaluated_edges++;  
    planner_stats_.num_jobs_per_thread[thread_id] +=1;
    lock_.unlock();

    auto action = edge_ptr->action_ptr_;
    auto parent_state_ptr = edge_ptr->parent_state_ptr_;
    auto child_state_ptr = edge_ptr->child_state_ptr_;

    // auto t_start = chrono::steady_clock::now();
    auto action_successor = action->Evaluate(parent_state_ptr->GetStateVars(), child_state_ptr->GetStateVars(), thread_id);
    // auto t_end = chrono::steady_clock::now();
    // double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
    // edge_ptr->real_eval_time_ = 1e-9*t_elapsed;

    if (action_successor.success_)
    {
        edge_ptr->SetCost(action_successor.successor_state_vars_costs_.back().second);
    }
    else
    {
        edge_ptr->SetCost(DINF);
    }
    
    edge_ptr->is_eval_ = false;

    // Edge must be marked close after all other info like cost and booleans are set
    edge_ptr->is_closed_ = true;
}

void MplpPlanner::evaluateEdgeLoop(int thread_id)
{
    while (!terminate_)
    {

        lock_vec_[thread_id].lock();
        bool status = edge_evaluation_status_[thread_id];
        lock_vec_[thread_id].unlock();

        while ((!status) && (!terminate_))
        {
            lock_vec_[thread_id].lock();
            status = edge_evaluation_status_[thread_id];
            lock_vec_[thread_id].unlock();
            // cout << "Evaluation thread " << thread_id << " waiting! " << edge_evaluation_status_[thread_id] << endl;
        }

        if (terminate_)
            break;


        evaluateEdge(edge_evaluation_vec_[thread_id], thread_id);

        lock_vec_[thread_id].lock();
        edge_evaluation_vec_[thread_id] = NULL;
        edge_evaluation_status_[thread_id] = 0;
        lock_vec_[thread_id].unlock();
    }

}

void MplpPlanner::constructPlan(StatePtrType goal_state_ptr)
{
    auto state_ptr = goal_state_ptr;
    vector<EdgePtrType> edges_in_plan;

    while(state_ptr->GetIncomingEdgePtr())
    {       
        edges_in_plan.insert(edges_in_plan.begin(), state_ptr->GetIncomingEdgePtr());
        state_ptr = state_ptr->GetIncomingEdgePtr()->parent_state_ptr_;     
    }

    // Detect duplicate paths in Pi (lazy_plans_)
    bool duplicate_path = false;
    for (auto path: lazy_plans_)
    {
        if ((edges_in_plan == path) && (!duplicate_path))
        {
            duplicate_path = true;
            auto it_edges_in_plan = edges_in_plan.begin();

            for (auto edge_in_plan: path)
            {
                if ((*it_edges_in_plan)->edge_id_ != edge_in_plan->edge_id_)
                {
                    duplicate_path=false;
                    break;
                }
   
                it_edges_in_plan++;            
            }
        }
    }

    if (!duplicate_path)
    {
        lock_.lock();
        updateEdgePriority(edges_in_plan, 2);
        lock_.unlock();
    
        lock_2_.lock();
        lazy_plans_.emplace_back(edges_in_plan);
        lock_2_.unlock();        
    }
}

void MplpPlanner::monitorPaths()
{
    while (!terminate_)
    {
        lock_2_.lock();
        auto lazy_plans = lazy_plans_;
        lock_2_.unlock();

        for (int plan_idx = 0; plan_idx < lazy_plans.size(); ++plan_idx)
        {
            lock_2_.lock();
            int plan_outcome = plan_evaluation_outcomes_[plan_idx];
            lock_2_.unlock();

            if ((plan_outcome == 0) || (plan_outcome == 1))
                continue;

            auto lazy_plan = lazy_plans[plan_idx];

            std::vector<PlanElement> plan;
            bool curr_plan_valid = true;
            
            for (auto& edge_ptr: lazy_plan)
            {       
                
                if (edge_ptr->is_closed_)
                {
                    if (edge_ptr->GetCost() == DINF)
                    {
                        curr_plan_valid = false;
                        lock_2_.lock();
                        plan_evaluation_outcomes_[plan_idx] = 0;
                        lock_2_.unlock();                        
                        break;
                    }
                }
                else
                {
                    curr_plan_valid = false;
                    break;
                }

            }

            if (curr_plan_valid)
            {

                double cost = 0;
                for (auto& edge_ptr: lazy_plan)
                {                       
                    if (edge_ptr->parent_state_ptr_)
                    {
                        plan.emplace_back(PlanElement(edge_ptr->parent_state_ptr_->GetStateVars(), edge_ptr->action_ptr_, edge_ptr->GetCost()));        
                    }
                    cost = cost + edge_ptr->GetCost();
                }

                lock_2_.lock();
                // This condition needs to hold true to gurantee sub-optimality bound
                if (cost <= cost_bound_)
                {

                    if (VERBOSE)
                    {
                        cout << "VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV Valid plan found! VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV" << endl;
                        cout << "Plan index: " << plan_idx << endl;
                        cout << "Plan length: " << plan.size() << endl;
                        cout << "Plan cost: " << cost << endl;
                        cout << "VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV" << endl;                        
                    }
                                    
                    plan_evaluation_outcomes_[plan_idx] = 1;                    
                    
                    // Only update these for the first valid plan
                    if (!plan_found_)
                    {
                        plan_ = plan;
                        successful_plan_idx_ = plan_idx;
                        planner_stats_.path_cost = cost;
                        planner_stats_.path_length += plan_.size();
                        plan_found_ = true;                    
                    }

                }
                else
                {
                    plan_evaluation_outcomes_[plan_idx] = 0;
                    if (VERBOSE) cout << "Valid plan not optimal, cost bound: " << cost_bound_  << " | true cost of plan: " << cost << endl; 
                }

                lock_2_.unlock();
                
            }

        }

    }

}

void MplpPlanner::assignEdgePriority(EdgePtrType& edge_ptr)
{
    edge_ptr->evaluation_priority_ = 1;
}

void MplpPlanner::updateEdgePriority(vector<EdgePtrType>& edge_ptrs, double factor)
{
    for (auto& edge_ptr : edge_ptrs)
    {
        updateEdgePriority(edge_ptr, factor);
    }
}

void MplpPlanner::updateEdgePriority(EdgePtrType& edge_ptr, double factor)
{
    bool edge_found = false;

    if (edges_open_.contains(edge_ptr))
    {
        edge_found = true;
        edge_ptr->evaluation_priority_ = factor*edge_ptr->evaluation_priority_;

        if (factor > 1)
        {
            edges_open_.decrease(edge_ptr);
        }
        else
        {
            edges_open_.increase(edge_ptr);
        }
    }
}

void MplpPlanner::exit()
{
    delegate_edges_process_->join();
    monitor_paths_process_->join();

    bool all_edge_eval_threads_terminated = false;
    while (!all_edge_eval_threads_terminated)
    {
        all_edge_eval_threads_terminated = true;
        for (auto& fut : edge_evaluation_futures_)
        {
            if (!isFutureReady(fut))
            {
                all_edge_eval_threads_terminated = false;
                break;
            }
        }
    }

    // Clear open list
    while (!state_open_list_.empty())
    {
        state_open_list_.pop();
    }

    // Clear Eopen
    while (!edges_open_.empty())
    {
        edges_open_.pop();
    }

    lazy_plans_.clear();
    plan_evaluation_outcomes_.clear();
    edge_evaluation_vec_.clear();
    edge_evaluation_status_.clear();

    Planner::exit();
}
