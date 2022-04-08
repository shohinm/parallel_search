#include "EpasePlanner.hpp"

using namespace std;
using namespace epase;

EpasePlanner::EpasePlanner():
Planner()
{
    
}

EpasePlanner::~EpasePlanner()
{
    
}

bool EpasePlanner::Plan()
{
    
}

void EpasePlanner::initialize()
{
    Planner::initialize();
    terminate_ = false;

    edge_open_list_ = EdgeQueueMinType();
 
    // Insert proxy edge with start state
    auto edge_ptr = new Edge(start_state_ptr_, Action("dummy"));
    edge_ptr->expansion_priority_ = heuristic_w_*computeHeuristic(start_state_ptr_);

    edge_map_.insert(make_pair(getEdgeKey(edge_ptr), edge_ptr));
    edge_open_list_.push(edge_ptr);   

}
