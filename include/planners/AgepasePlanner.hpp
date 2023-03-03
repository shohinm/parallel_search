#ifndef AGEPASE_PLANNER_HPP
#define AGEPASE_PLANNER_HPP

#define NAIVE 0
#define ADAPTIVE 0

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
        void improvePath();
        void expand(EdgePtrType edge_ptr, int thread_id);
        void expandEdge(EdgePtrType edge_ptr, int thread_id);
        void exitMultiThread();
        void exit();

        std::vector<EdgePtrType> edge_incon_list_;
        double delta_w_;
        double best_cost_;
        std::vector<PlanElement> best_plan_;

        // Experiment
        std::vector<double> data_list_;
};

}

#endif
