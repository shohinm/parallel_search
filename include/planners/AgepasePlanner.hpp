#ifndef AGEPASE_PLANNER_HPP
#define AGEPASE_PLANNER_HPP

#include <future>
#include <condition_variable>
#include <planners/GepasePlanner.hpp>

namespace ps
{

class AgepasePlanner : public GepasePlanner
{
    public:
        AgepasePlanner(ParamsType planner_params);
        ~AgepasePlanner();
        bool Plan();

    protected:
        void initialize();
        bool improvePath();
        void expand(EdgePtrType edge_ptr, int thread_id);
        void expandEdge(EdgePtrType edge_ptr, int thread_id);
        void exit();

        std::vector<EdgePtrType> edge_incon_list_;
        double time_budget_;
        bool found_plan_;
        bool found_plan_optimal_;
};

}

#endif
