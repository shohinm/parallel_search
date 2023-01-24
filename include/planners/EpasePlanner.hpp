#ifndef EPASE_PLANNER_HPP
#define EPASE_PLANNER_HPP

#include <future>
#include <condition_variable>
#include <planners/GepasePlanner.hpp>

namespace ps
{

class EpasePlanner : public GepasePlanner
{
    public:
        EpasePlanner(ParamsType planner_params);
        ~EpasePlanner();
        bool Plan();

    protected:
        void expandEdgeLoop(int thread_id);    
        void expandEdge(EdgePtrType edge_ptr, int thread_id);
        void exit();

        std::vector<StatePtrType> being_expanded_states_;    
};

}

#endif
