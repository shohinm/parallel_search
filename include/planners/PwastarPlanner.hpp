#ifndef PWASTAR_PLANNER_HPP
#define PWASTAR_PLANNER_HPP

#include <future>
#include <planners/WastarPlanner.hpp>

namespace ps
{

class PwastarPlanner : public WastarPlanner
{
    public:
        PwastarPlanner(ParamsType planner_params);
        ~PwastarPlanner();
        bool Plan();

    protected:
        void initialize();
        void expandState(StatePtrType state_ptr);
        void evaluateEdgeThread(int thread_idx);
        void exit();

        mutable std::vector<LockType> lock_vec_; 
        std::vector<std::future<void>> edge_evaluation_futures_;
        std::vector<EdgePtrType> edge_evaluation_vec_;
        std::vector<int> edge_evaluation_status_;
        std::vector<std::vector<std::pair<ActionPtrType, ActionSuccessor>>> all_successors_;

        bool terminate_;

};

}

#endif
