#ifndef STATE_HPP
#define STATE_HPP

#include <vector>
#include <memory>
#include <atomic>
#include <climits>
#include <common/Types.hpp>

namespace epase
{

class State
{

public:
	// State();
	State(const StateVarsType& vars=StateVarsType());
	~State() {};

	std::size_t GetStateID() const {return state_id_;};
	static void ResetStateIDCounter() {id_counter_=0;};

    void SetStateVars(StateVarsType& vars) {vars_ = vars;};
    StateVarsType GetStateVars() const {return vars_;};

	void SetGValue(const double& g_val) {g_val_ = g_val;};
	double GetGValue() const {return g_val_;};
	void ResetGValue() {g_val_ = std::numeric_limits<double>::max();};

	void SetHValue(const double& h_val) {h_val_ = h_val;};
	double GetHValue() const {return h_val_;};
	void ResetHValue() {h_val_ = -1;};

	void SetFValue(const double& f_val) {f_val_ = f_val;};
	double GetFValue() const {return f_val_;}; 
	void ResetFValue() {f_val_ = std::numeric_limits<double>::max();};

	void SetVisited() {is_visited_ = true;};
	void UnsetVisited() {is_visited_ = false;};
	bool IsVisited() {return is_visited_;};

    void SetBeingExpanded() {being_expanded_ = true;};
    void UnsetBeingExpanded() {being_expanded_ = false;};
    bool IsBeingExpanded() {return being_expanded_;};

    void SetIncomingEdgePtr(EdgePtrType edge_ptr) {incoming_edge_ptr_ = edge_ptr;};
    EdgePtrType GetIncomingEdgePtr() {return incoming_edge_ptr_;};
    void ResetIncomingEdgePtr() {incoming_edge_ptr_ = NULL;};

    void Print(std::string str="");

    std::atomic<int> num_successors_;
    std::atomic<int> num_expanded_successors_;

protected:

private:
	static std::size_t id_counter_;

	std::size_t state_id_;
    StateVarsType vars_;
	double g_val_;
    double h_val_;
	double f_val_;
	std::atomic<bool> is_visited_;
    std::atomic<bool> being_expanded_;
    EdgePtrType incoming_edge_ptr_;
};

class IsLesserState
{
public:
    bool operator() (const State& lhs, const State& rhs);
};

}



#endif
