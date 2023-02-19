#include <iostream>
#include <cmath>
#include <planners/RrtConnectPlanner.hpp>

using namespace std;
using namespace ps;

RrtConnectPlanner::RrtConnectPlanner(ParamsType planner_params):
RrtPlanner(planner_params)
{    

}

RrtConnectPlanner::~RrtConnectPlanner()
{
    
}

void RrtConnectPlanner::SetGoalState(const StateVarsType& state_vars)
{
    goal_state_ptr_ = constructState(state_vars, state_map_goal_);
}

void RrtConnectPlanner::initialize()
{
    RrtPlanner::initialize();
    start_goal_flag_.resize(num_threads_, 0);
}

bool RrtConnectPlanner::connect(StatePtrType state_ptr, const StatePtrMapType& state_map)
{

}

void RrtConnectPlanner::rrtThread(int thread_id)
{
    while (!terminate_)
    {
        bool is_collision;
        StatePtrType state_ptr;

        if (start_goal_flag_[thread_id] == 0)
        {
            auto sampled_state = sampleState(start_state_ptr_);
            auto nearest_neighbor = getNearestNeighbor(sampled_state);
            state_ptr = extend(nearest_neighbor, sampled_state, is_collision, state_map_, thread_id);
            if ((!terminate_) && connect(state_ptr, state_map_goal_))
            {            
                // Reconstruct and return path
                constructPlan(state_ptr);   
                terminate_ = true;
                lock_.unlock();
                return;
            }
        }
        else
        {
            auto sampled_state = sampleState(goal_state_ptr_);
            auto nearest_neighbor = getNearestNeighbor(sampled_state);
            state_ptr = extend(nearest_neighbor, sampled_state, is_collision, state_map_goal_, thread_id);            
        
            if ((!terminate_) && connect(state_ptr, state_map_))
            {            
                // Reconstruct and return path
                constructPlan(state_ptr);   
                terminate_ = true;
                lock_.unlock();
                return;
            }
        }

        start_goal_flag_[thread_id] = 1 - start_goal_flag_[thread_id];        
    }
}

void RrtConnectPlanner::exit()
{
    for (auto& state_it : state_map_goal_)
    {        
        if (state_it.second)
        {
            delete state_it.second;
            state_it.second = NULL;
        }
    }
    state_map_goal_.clear();

    RrtPlanner::exit();
}
