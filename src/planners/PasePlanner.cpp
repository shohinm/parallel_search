#include <iostream>
#include <planners/PasePlanner.hpp>

#define VERBOSE 0

using namespace std;
using namespace epase;

PasePlanner::PasePlanner(ParamsType planner_params):
Planner(planner_params)
{    
    num_threads_  = planner_params["num_threads"];
    vector<LockType> lock_vec(num_threads_-1);
    lock_vec_.swap(lock_vec);
}

PasePlanner::~PasePlanner()
{
    
}

bool PasePlanner::Plan(int exp_idx)
{
    


}

void PasePlanner::initialize()
{
    Planner::initialize();
    terminate_ = false;
    recheck_flag_ = true;

    state_expansion_futures_.clear();

    being_expanded_states_.clear();
}

void PasePlanner::expandEdge(Edge* edge_ptr, int thread_id)
{


}

void PasePlanner::exit()
{
    bool all_expansion_threads_terminated = false;
    while (!all_expansion_threads_terminated)
    {
        all_expansion_threads_terminated = true;
        for (auto& fut : state_expansion_futures_)
        {
            if (!isFutureReady(fut))
            {
                all_expansion_threads_terminated = false;
                break;
            }
        }
    }
    state_expansion_futures_.clear();

    Planner::exit();
}
