#ifndef STATE_HPP
#define STATE_HPP

#include <vector>
#include <memory>
#include "Types.hpp"

namespace epase
{

class State
{

public:
	// State();
	State(const StateVarsType& vars=StateVarsType());
	~State(){};

	std::size_t GetStateID() const {return state_id_;};
	static void ResetStateIDCounter(){id_counter_=0;};

    void SetStateVars(StateVarsType& vars){vars_ = vars;};
    StateVarsType GetStateVars() const {return vars_;};

	void SetGValue(const double& g_val){g_val_ = g_val;};
	double GetGValue() const {return g_val_;};
	void ResetGValue();

	void SetHValue(const double& h_val){h_val_ = h_val;};
	double GetHValue() const {return h_val_;};
	void ResetHValue();

	void SetFValue(const double& f_val){f_val_ = f_val;};
	double GetFValue() const {return f_val_;}; 
	void ResetFValue();

protected:

private:
	static std::size_t id_counter_;

	std::size_t state_id_;
    StateVarsType vars_;
	double g_val_;
    double h_val_;
	double f_val_;
	bool is_visited_;
	std::shared_ptr<State> parent_state_ptr_;


};

class IsLesserState
{
public:
    bool operator() (const State& lhs, const State& rhs);
};

}



#endif
