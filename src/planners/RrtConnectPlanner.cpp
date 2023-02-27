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
    start_goal_flag_.resize(planner_params_["num_threads"], 0);
}


bool RrtConnectPlanner::connect(StatePtrType state_ptr, StatePtrMapType& state_map, 
    EdgePtrMapType& edge_map, int thread_id, StatePtrType& connected_state)
{
    auto nearest_neighbor = getNearestNeighbor(state_ptr->GetStateVars(), state_map);
    
    bool is_collision;
    auto final_valid_state_vars = collisionFree(nearest_neighbor->GetStateVars(), state_ptr->GetStateVars(), 
                                state_map, is_collision, thread_id);
   
    // auto final_state = constructState(final_valid_state_vars, state_map);
   
    if (is_collision)
    {
        return false;
    }
    else
    {   
        cout << "Connected \n";
        connected_state = nearest_neighbor;
        addEdge(nearest_neighbor, state_ptr, edge_map);
        return true;
    }
}

void RrtConnectPlanner::constructPlan(StatePtrType& connected_state_start, StatePtrType& connected_state_goal)
{
    double cost = 0;
    while(connected_state_start)
    {
        if (connected_state_start->GetIncomingEdgePtr())
        {
            plan_.insert(plan_.begin(), PlanElement(connected_state_start->GetStateVars(), connected_state_start->GetIncomingEdgePtr()->action_ptr_, connected_state_start->GetIncomingEdgePtr()->GetCost()));        
            cost += connected_state_start->GetIncomingEdgePtr()->GetCost();
            connected_state_start = connected_state_start->GetIncomingEdgePtr()->parent_state_ptr_;     

        } // For start connected_state_start, there is no incoming edge
        else
        {
            plan_.insert(plan_.begin(), PlanElement(connected_state_start->GetStateVars(), NULL, 0));        
            connected_state_start = NULL;
        }

    }

    while(connected_state_goal)
    {
        if (connected_state_goal->GetIncomingEdgePtr())
        {
            plan_.insert(plan_.end(), PlanElement(connected_state_goal->GetStateVars(), connected_state_goal->GetIncomingEdgePtr()->action_ptr_, connected_state_goal->GetIncomingEdgePtr()->GetCost()));        
            cost += connected_state_goal->GetIncomingEdgePtr()->GetCost();
            connected_state_goal = connected_state_goal->GetIncomingEdgePtr()->parent_state_ptr_;     

        } // For start connected_state_goal, there is no incoming edge
        else
        {
            plan_.insert(plan_.end(), PlanElement(connected_state_goal->GetStateVars(), NULL, 0));        
            connected_state_goal = NULL;
        }

    }

    if (post_processor_)
    {
        post_processor_(plan_, cost);
    }

    planner_stats_.path_cost_= cost;
    planner_stats_.path_length_ = plan_.size();

}

void RrtConnectPlanner::rrtThread(int thread_id)
{
    while (!terminate_)
    {
        planner_stats_.num_jobs_per_thread_[thread_id] +=1;

        bool is_collision;
        StatePtrType state_ptr;

        if (start_goal_flag_[thread_id] == 0)
        {
            auto sampled_state = sampleState(start_state_ptr_, thread_id, 0);
            auto nearest_neighbor = getNearestNeighbor(sampled_state, state_map_);
            state_ptr = extend(nearest_neighbor, sampled_state, is_collision, state_map_, thread_id);
            auto edge = addEdge(nearest_neighbor, state_ptr, edge_map_);
            edge->SetCost(getCost(nearest_neighbor->GetStateVars(), state_ptr->GetStateVars(), thread_id));
            state_ptr->SetIncomingEdgePtr(edge);
            
            if(!isValidConfiguration(state_ptr->GetStateVars(), thread_id))
            {
                throw runtime_error("COLLISION!");
            }
            
            StatePtrType connected_state;
            if (connect(state_ptr, state_map_goal_, edge_map_, thread_id, connected_state))
            {            
                // Reconstruct and return path
                lock_.lock();
                if (!terminate_)
                {
                     if (VERBOSE) cout << "Connect succeeded from start_graph to goal_graph" << endl;
                    constructPlan(state_ptr, connected_state);   
                    terminate_ = true;
                    plan_found_ = true;
                }
                lock_.unlock();                    
                return;
            }
        }
        else
        {
            auto sampled_state = sampleState(goal_state_ptr_, thread_id, 0);
            auto nearest_neighbor = getNearestNeighbor(sampled_state, state_map_goal_);
            state_ptr = extend(nearest_neighbor, sampled_state, is_collision, state_map_goal_, thread_id);            
            auto edge = addEdge(nearest_neighbor, state_ptr, edge_map_goal_);
            edge->SetCost(getCost(nearest_neighbor->GetStateVars(), state_ptr->GetStateVars(), thread_id));
            state_ptr->SetIncomingEdgePtr(edge);

            if(!isValidConfiguration(state_ptr->GetStateVars(), thread_id))
            {
                throw runtime_error("COLLISION!");
            }
    
            StatePtrType connected_state;
            if (connect(state_ptr, state_map_, edge_map_goal_, thread_id, connected_state))
            {            
                // Reconstruct and return path
                lock_.lock();
                if (!terminate_)
                {
                    if (VERBOSE) cout << "Connect succeeded from goal_graph to start_graph" << endl;
                    constructPlan(connected_state, state_ptr);   
                    terminate_ = true;
                    plan_found_ = true;
                }
                lock_.unlock();                    
                return;
            }
        }

        start_goal_flag_[thread_id] = 1 - start_goal_flag_[thread_id];        
    }
}

void RrtConnectPlanner::exit()
{

    bool all_rrt_threads_terminated = false;
    while (!all_rrt_threads_terminated)
    {
        all_rrt_threads_terminated = true;
        for (auto& fut : rrt_futures_)
        {
            if (!isFutureReady(fut))
            {
                all_rrt_threads_terminated = false;
                break;
            }
        }
    }
    rrt_futures_.clear();


    cout << "Exiting!" << endl;
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

    Planner::exit();

}
