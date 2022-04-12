#include <future>
#include "Planner.hpp"

namespace epase
{

class EpasePlanner : public Planner
{
    public:
        EpasePlanner(ParamsType planner_params);
        ~EpasePlanner();
    	bool Plan(int exp_idx = 1);

    protected:
    	void initialize();
        void expandEdgeLoop(int thread_id);    
        void expandEdge(Edge* edge_ptr, int thread_id);
    	void exit();

    	EdgeQueueMinType edge_open_list_;
	    std::vector<StatePtrType> being_expanded_states_;    

        // Multi-threading members
        int num_threads_;
        mutable LockType lock_;
	    mutable std::vector<LockType> lock_vec_; 
	    std::vector<std::future<void>> edge_expansion_futures_;
		std::vector<Edge*> edge_expansion_vec_;
		std::vector<int> edge_expansion_status_;

        // Control variables
	    std::atomic<bool> recheck_flag_;
        std::atomic<bool> terminate_;
        ActionPtrType dummy_action_ptr_;

	    double h_val_min_;

};

}