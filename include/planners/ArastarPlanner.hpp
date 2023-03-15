#ifndef ARASTAR_PLANNER_HPP
#define ARASTAR_PLANNER_HPP

#define ADAPTIVE 0

#include <future>
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
        void updateState(StatePtrType& state_ptr, ActionPtrType& action_ptr, EdgePtrType& edge_ptr);
        void exit();

        std::vector<StatePtrType> state_incon_list_;
        double delta_w_;
};

}

#endif
