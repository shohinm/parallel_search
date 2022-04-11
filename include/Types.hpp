#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <memory>
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

	struct ActionSuccessor
	{
		ActionSuccessor(bool success, StateVarsType successor_state_vars, double cost): success_(success), successor_state_vars_(successor_state_vars), cost_(cost) {}
		bool success_;
		StateVarsType successor_state_vars_;
		double cost_;	
	};



}

#endif