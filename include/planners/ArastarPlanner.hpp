#ifndef ARASTAR_PLANNER_HPP
#define ARASTAR_PLANNER_HPP

#include <future>
#include <planners/Planner.hpp>
#include <planners/WastarPlanner.hpp>  

namespace ps
{

class ArastarPlanner : public WastarPlanner
{
    public:
        ArastarPlanner(ParamsType planner_params);
        ~ArastarPlanner();
        bool Plan();

    protected:
        void initialize();
        void improvePath();
        void expandState(StatePtrType state_ptr);
        void updateState(StatePtrType& state_ptr, ActionPtrType& action_ptr, ActionSuccessor& action_successor);
        void exit();

        std::vector<StatePtrType> state_incon_list_;
        double time_budget_;

};

}

#endif
