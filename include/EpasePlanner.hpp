#include "Planner.hpp"

namespace epase
{

class EpasePlanner : public Planner
{
    public:
        EpasePlanner();
        ~EpasePlanner();
    	bool Plan();

    protected:
    	void initialize();
    	EdgeQueueMinType edge_open_list_;
	    std::vector<StatePtrType> being_expanded_states_;    
	    std::atomic<bool> terminate_;


 	
};

}