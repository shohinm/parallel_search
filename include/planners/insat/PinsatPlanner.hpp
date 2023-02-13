#ifndef PINSAT_PLANNER_HPP
#define PINSAT_PLANNER_HPP

#include <future>
#include <utility>
#include <planners/GepasePlanner.hpp>
#include <planners/insat/InsatPlanner.hpp>
#include <common/insat/InsatState.hpp>
#include <common/insat/InsatEdge.hpp>

namespace ps
{

class PinsatPlanner : virtual public GepasePlanner, virtual public InsatPlanner
{
    public:
        typedef smpl::intrusive_heap<InsatEdge, IsLesserEdge> EdgeQueueMinType;
        typedef smpl::intrusive_heap<InsatState, IsLesserState> BEType;

        PinsatPlanner(ParamsType planner_params);
        ~PinsatPlanner();
        bool Plan();

    protected:
        void initialize();
        void expandEdgeLoop(int thread_id);    
        void expand(InsatEdgePtrType edge_ptr, int thread_id);
        void expandEdge(InsatEdgePtrType edge_ptr, int thread_id);
        void exit();

        EdgeQueueMinType edge_open_list_;
        BEType being_expanded_states_;    

        std::vector<InsatEdgePtrType> edge_expansion_vec_;
        InsatActionPtrType dummy_action_ptr_;


};

}

#endif
    
