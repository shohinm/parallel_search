#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include "intrusive_heap.h"

namespace epase
{
	class State;
	class Edge;

	typedef std::vector<double> StateVarsType;
	typedef State* StatePtrType;
	typedef Edge* EdgePtrType;

}

#endif