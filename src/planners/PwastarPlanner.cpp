#include <iostream>
#include <algorithm>
#include <planners/PwastarPlanner.hpp>

using namespace std;
using namespace ps;

PwastarPlanner::PwastarPlanner(ParamsType planner_params):
WastarPlanner(planner_params)
{    
    num_threads_  = planner_params["num_threads"];
    vector<LockType> lock_vec(num_threads_-1);
    lock_vec_.swap(lock_vec);
}

PwastarPlanner::~PwastarPlanner()
{
    
}

bool PwastarPlanner::Plan()
{
    initialize();
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
            terminate_ = true;
            exit();
            return true;
        }

        expandState(state_ptr);        

        // auto t_end = chrono::steady_clock::now();
        // double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
        // if ((timeout_>0) && (1e-9*t_elapsed > timeout_))
        // {
        //     cout << "Planner took greater than timeout (" << timeout_ << "): " << 1e-9*t_elapsed << " | Exiting!" << endl;
        //     PrintWAstarStats(t_elapsed, exp_idx, false);
        //     return false;
        // }
    }

    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
    planner_stats_.total_time_ = 1e-9*t_elapsed;
    cout << "Goal not reached Number of states expanded: " << planner_stats_.num_state_expansions_ << endl;
    return false;
}

void PwastarPlanner::initialize()
{
    WastarPlanner::initialize();

    terminate_ = false;

    edge_evaluation_vec_.clear();
    edge_evaluation_vec_.resize(num_threads_-1, NULL);
    
    edge_evaluation_status_.clear();
    edge_evaluation_status_.resize(num_threads_-1, 0);

    edge_evaluation_futures_.clear();

    all_successors_.resize(max(1, num_threads_-1));
}

void PwastarPlanner::expandState(StatePtrType state_ptr)
{

    if (num_threads_ == 1)
        WastarPlanner::expandState(state_ptr);
    else
    {
        if (VERBOSE) state_ptr->Print("Expanding");

        state_ptr->SetVisited();
        planner_stats_.num_state_expansions_++;
       
        vector<shared_ptr<Action>> valid_action_ptrs;
        for (auto& action_ptr: actions_ptrs_)
            if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
                valid_action_ptrs.emplace_back(action_ptr);

        random_shuffle(valid_action_ptrs.begin(), valid_action_ptrs.end());


        for (auto& action_ptr: valid_action_ptrs)
        {
            int thread_id = 0;
            bool edge_evaluation_assigned = false;
            while (!edge_evaluation_assigned)
            {
                lock_vec_[thread_id].lock();
                bool status = edge_evaluation_status_[thread_id];
                lock_vec_[thread_id].unlock();

                if (!status)
                {
                    int num_threads_current = edge_evaluation_futures_.size();
                    if (thread_id >= num_threads_current)
                    {
                        // cout << "Spawning thread " << thread_id << endl;
                        edge_evaluation_futures_.emplace_back(async(launch::async, &PwastarPlanner::evaluateEdgeThread, this, thread_id));
                    }

                    lock_vec_[thread_id].lock();
                    // cout << "Assigning thread_id: " << thread_id <<endl;
                    edge_evaluation_vec_[thread_id] = new Edge(state_ptr, action_ptr);
                    edge_evaluation_status_[thread_id] = 1;
                    edge_evaluation_assigned = true;       
                    lock_vec_[thread_id].unlock();
                }
                else
                    thread_id = thread_id == num_threads_-2 ? 0 : thread_id+1;
            }
        }


        // Wait for evaluations to complete
        for (int thread_id = 0; thread_id < edge_evaluation_status_.size(); ++thread_id)
        {
            lock_vec_[thread_id].lock();
            bool status = edge_evaluation_status_[thread_id];
            lock_vec_[thread_id].unlock();

            while(status)
            {
                lock_vec_[thread_id].lock();
                status = edge_evaluation_status_[thread_id];
                lock_vec_[thread_id].unlock();
            }
        }

        for (auto& all_successors_thread : all_successors_)
            for (auto& action_successor_tup : all_successors_thread)
                updateState(state_ptr, action_successor_tup.first, action_successor_tup.second);

        all_successors_.clear();
        all_successors_.resize(max(1, num_threads_-1));
    }    
}

void PwastarPlanner::evaluateEdgeThread(int thread_id)
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
        }

        if (terminate_)
            break;

        auto action_ptr = edge_evaluation_vec_[thread_id]->action_ptr_;
        auto state_ptr = edge_evaluation_vec_[thread_id]->parent_state_ptr_;
        auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars());

        lock_vec_[thread_id].lock();
        all_successors_[thread_id].emplace_back(make_pair(action_ptr, action_successor));
        delete edge_evaluation_vec_[thread_id];
        edge_evaluation_status_[thread_id] = 0;
        planner_stats_.num_jobs_per_thread_[thread_id] +=1;
        lock_vec_[thread_id].unlock();
    }    
}

void PwastarPlanner::exit()
{
    planner_stats_.num_threads_spawned_ += edge_evaluation_futures_.size();
    WastarPlanner::exit();
}
