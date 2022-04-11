#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <unordered_map>
#include "intrusive_heap.h"

namespace epase
{
    class State;
    class Edge;
    class Action;

	#define DINF std::numeric_limits<double>::infinity()

    typedef std::vector<double> StateVarsType;
    typedef State* StatePtrType;
    typedef std::shared_ptr<Action> ActionPtrType;
    typedef Edge* EdgePtrType;
    typedef std::mutex LockType;
	typedef std::unordered_map<std::string, double> ParamsType;
	
	struct ActionSuccessor
	{
		ActionSuccessor(bool success, std::vector<std::pair<StateVarsType, double>> successor_state_vars_costs): 
		success_(success), successor_state_vars_costs_(successor_state_vars_costs){};
		bool success_;
		std::vector<std::pair<StateVarsType, double>> successor_state_vars_costs_;
	};



}

#endif