#ifndef INSAT_PLANNER_HPP
#define INSAT_PLANNER_HPP

#include <future>
#include "planners/Planner.hpp"

namespace ps
{

    class InsatstarPlanner : public Planner
    {
    public:
        InsatstarPlanner(ParamsType planner_params);
        ~InsatstarPlanner();
        bool Plan();

    protected:
        void initialize();
        void expandState(StatePtrType state_ptr);
        void updateState(StatePtrType& state_ptr, ActionPtrType& action_ptr, ActionSuccessor& action_successor);
        void exit();

        int num_threads_;
        StateQueueMinType state_open_list_;

    };

}

#endif
