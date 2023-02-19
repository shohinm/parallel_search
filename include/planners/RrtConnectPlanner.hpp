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
        void SetGoalState(const StateVarsType& state_vars);\

    protected:
        void initialize();
        bool connect(StatePtrType state_ptr, const StatePtrMapType& state_map);
        void rrtThread(int thread_id);
        void exit();
    
        StatePtrMapType state_map_goal_;
        std::vector<int> start_goal_flag_;

};

}


#endif
