#ifndef GEPASE_PLANNER_HPP
#define GEPASE_PLANNER_HPP

#include <future>
#include <condition_variable>
#include <planners/Planner.hpp>

namespace ps
{

class GepasePlanner : public Planner
{
    public:
        typedef smpl::intrusive_heap<State, IsLesserState> BEType;
        GepasePlanner(ParamsType planner_params);
        ~GepasePlanner();
        virtual bool Plan();

    protected:
        void initialize();
        void notifyMainThread();
        void expandEdgeLoop(int thread_id);    
        void expand(EdgePtrType edge_ptr, int thread_id);
        void expandEdge(EdgePtrType edge_ptr, int thread_id);
        void exit();

        EdgeQueueMinType edge_open_list_;
        BEType being_expanded_states_;    

        // Multi-threading members
        int num_threads_;
        mutable LockType lock_;
        mutable std::vector<LockType> lock_vec_; 
        std::vector<std::future<void>> edge_expansion_futures_;
        std::vector<EdgePtrType> edge_expansion_vec_;
        std::vector<int> edge_expansion_status_;
        std::vector<std::condition_variable> cv_vec_;
        std::condition_variable cv_;

        // Control variables
        std::atomic<bool> recheck_flag_;
        std::atomic<bool> terminate_;
        ActionPtrType dummy_action_ptr_;
};

}

#endif
