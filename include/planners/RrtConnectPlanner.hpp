#ifndef RRTCONNECT_PLANNER_HPP
#define RRTCONNECT_PLANNER_HPP

#include <future>
#include <random>
#include <planners/RrtPlanner.hpp>

namespace ps
{

class RrtConnectPlanner : public RrtPlanner
{
    public:
        RrtConnectPlanner(ParamsType planner_params);
        ~RrtConnectPlanner();
        void SetGoalState(const StateVarsType& state_vars);
        bool Plan();

    protected:
        void initialize();
        void constructPlan(StatePtrType& connected_state_start, StatePtrType& connected_state_goal);
        bool connect(StatePtrType state_ptr, StatePtrMapType& state_map, EdgePtrMapType& edge_map, int thread_id, StatePtrType& connected_state);
        void rrtThread(int thread_id);
        void exitThreads();
        void exit();
    
        StatePtrMapType state_map_goal_;
        EdgePtrMapType edge_map_goal_;

        std::vector<int> start_goal_flag_;

};

}


#endif
