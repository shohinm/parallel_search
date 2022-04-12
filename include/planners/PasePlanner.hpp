#include <future>
#include <planners/Planner.hpp>

namespace epase
{

class PasePlanner : public Planner
{
    public:
        PasePlanner(ParamsType planner_params);
        ~PasePlanner();
        bool Plan(int exp_idx = 1);

    protected:
        void initialize();
        void expandEdge(Edge* edge_ptr, int thread_id);
        void exit();

        StateQueueMinType state_open_list_;
        std::vector<StatePtrType> being_expanded_states_;    

        // Multi-threading members
        int num_threads_;
        mutable LockType lock_;
        mutable std::vector<LockType> lock_vec_; 
        std::vector<std::future<void>> state_expansion_futures_;

        // Control variables
        std::atomic<bool> recheck_flag_;
        std::atomic<bool> terminate_;
};

}