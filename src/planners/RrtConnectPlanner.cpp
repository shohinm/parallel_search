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

bool RrtConnectPlanner::connect(StatePtrType state_ptr, StatePtrMapType& state_map, EdgePtrMapType& edge_map, int thread_id)
{
    auto nearest_neighbor = getNearestNeighbor(state_ptr->GetStateVars(), state_map);
    
    bool is_collision;
    auto final_valid_state_vars = collisionFree(nearest_neighbor->GetStateVars(), state_ptr->GetStateVars(), 
                                state_map, is_collision, thread_id);
   
    auto final_state = constructState(final_valid_state_vars, state_map);
    addEdge(nearest_neighbor, final_state, edge_map);
   
    if (is_collision)
    {
        return false;
    }
    else
    {   
        cout << "Connected \n";
        return true;
    }
}

void RrtConnectPlanner::rrtThread(int thread_id)
{
    while (!terminate_)
    {
        bool is_collision;
        StatePtrType state_ptr;

        if (start_goal_flag_[thread_id] == 0)
        {
            auto sampled_state = sampleState(start_state_ptr_, thread_id);
            auto nearest_neighbor = getNearestNeighbor(sampled_state, state_map_);
            state_ptr = extend(nearest_neighbor, sampled_state, is_collision, state_map_, thread_id);
            addEdge(nearest_neighbor, state_ptr, edge_map_);

            if ((!terminate_) && connect(state_ptr, state_map_goal_, edge_map_, thread_id))
            {            
                // Reconstruct and return path
                constructPlan(state_ptr);   
                terminate_ = true;
                return;
            }
        }
        else
        {
            auto sampled_state = sampleState(goal_state_ptr_, thread_id);
            auto nearest_neighbor = getNearestNeighbor(sampled_state, state_map_goal_);
            state_ptr = extend(nearest_neighbor, sampled_state, is_collision, state_map_goal_, thread_id);            
            addEdge(nearest_neighbor, state_ptr, edge_map_goal_);
        
            if ((!terminate_) && connect(state_ptr, state_map_, edge_map_goal_, thread_id))
            {            
                // Reconstruct and return path
                constructPlan(state_ptr);   
                terminate_ = true;
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

    for (auto& edge_it : edge_map_goal_)
    {
        if (edge_it.second)
        {
            delete edge_it.second;
            edge_it.second = NULL;
        }
    }
    edge_map_goal_.clear();

    RrtPlanner::exit();
}
