#ifndef WASTAR_PLANNER_HPP
#define WASTAR_PLANNER_HPP

#include <future>
#include <planners/Planner.hpp>

namespace ps
{

class WastarPlanner : public Planner
{
    public:
        WastarPlanner(ParamsType planner_params);
        ~WastarPlanner();
        virtual bool Plan();

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
