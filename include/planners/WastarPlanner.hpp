#ifndef WASTAR_PLANNER_HPP
#define WASTAR_PLANNER_HPP

#include <future>
#include <planners/Planner.hpp>

namespace epase
{

class WastarPlanner : public Planner
{
    public:
        WastarPlanner(ParamsType planner_params);
        ~WastarPlanner();
        bool Plan();

    protected:
        void initialize();
        void expandState(StatePtrType state_ptr);
        void exit();

        StateQueueMinType state_open_list_;

};

}

#endif
