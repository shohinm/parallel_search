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

	typedef std::vector<double> StateVarsType;
	typedef State* StatePtrType;
	typedef std::shared_ptr<Action> ActionPtrType;
	typedef Edge* EdgePtrType;
	typedef std::mutex LockType;
}

#endif