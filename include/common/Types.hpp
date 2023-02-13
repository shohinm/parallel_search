#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <unordered_map>
#include <common/EigenTypes.h>
#include "intrusive_heap.h"

namespace ps
{
    class State;
    class InsatState;
    class Edge;
    class InsatEdge;
    class Action;
    class InsatAction;

    #define DINF std::numeric_limits<double>::infinity()

    typedef std::vector<double> StateVarsType;
    typedef State* StatePtrType;
    typedef InsatState* InsatStatePtrType;
    typedef std::shared_ptr<Action> ActionPtrType;
    typedef std::shared_ptr<InsatAction> InsatActionPtrType;
    typedef Edge* EdgePtrType;
    typedef InsatEdge* InsatEdgePtrType;
    typedef std::mutex LockType;
    typedef std::unordered_map<std::string, double> ParamsType;
    typedef MatDf TrajType;

    struct ActionSuccessor
    {
        ActionSuccessor(){};
        ActionSuccessor(bool success, std::vector<std::pair<StateVarsType, double>> successor_state_vars_costs): 
        success_(success), successor_state_vars_costs_(successor_state_vars_costs){};
        bool success_;
        std::vector<std::pair<StateVarsType, double>> successor_state_vars_costs_;
    };

    struct PlanElement 
    {
        PlanElement(StateVarsType state, ActionPtrType action_ptr, double cost): state_(state), incoming_action_ptr_(action_ptr), cost_(cost) {};
        ~PlanElement(){};
        StateVarsType state_;
        ActionPtrType incoming_action_ptr_;
        double cost_;
    };

    struct PlannerStats
    {
        double total_time_=  0; // seconds
        double path_cost_ = 0;
        double path_length_  =0;
        
        int num_state_expansions_ = 0;
        int num_evaluated_edges_ = 0;
        int num_threads_spawned_ = 0;
        
        std::vector<int> num_jobs_per_thread_;
        
	    double lock_time_ = 0;
        double cumulative_expansions_time_ = 0;

        std::unordered_map<std::string, std::vector<double>> action_eval_times_;
    };
}

#endif
