#include "ActionsTemplate.hpp"
#include <planners/EpasePlanner.hpp>

using namespace std;
using namespace ps;

/**
 * @brief      Calculates the heuristic.
 *
 * @param[in]  state_ptr  The state pointer
 *
 * @return     The heuristic.
 */
double computeHeuristic(const StatePtrType& state_ptr)
{

}

/**
 * @brief      Calculates the heuristic state to state.
 *
 * @param[in]  state_ptr_1  The state pointer 1
 * @param[in]  state_ptr_2  The state pointer 2
 *
 * @return     The heuristic state to state.
 */
double computeHeuristicStateToState(const StatePtrType& state_ptr_1, const StatePtrType& state_ptr_2)
{

}

/**
 * @brief      Determines whether the specified state pointer is goal state.
 *
 * @param[in]  state_ptr  The state pointer
 *
 * @return     True if the specified state pointer is goal state, False otherwise.
 */
bool isGoalState(const StatePtrType& state_ptr)
{

}

/**
 * @brief      { function_description }
 *
 * @param[in]  state_vars  The state variables
 *
 * @return     { description_of_the_return_value }
 */
size_t StateKeyGenerator(const StateVarsType& state_vars)
{ 

}

/**
 * @brief      { function_description }
 *
 * @param[in]  edge_ptr  The edge pointer
 *
 * @return     { description_of_the_return_value }
 */
size_t EdgeKeyGenerator(const EdgePtrType& edge_ptr)
{

}

int main(int argc, char* argv[])
{
    // Define planner parameters
    ParamsType planner_params;
    // planner_params["num_threads"] = 5;
    // planner_params["heuristic_weight"] = 50;

    // Define action parameters. Actions can share parameters or have different parameters.
    ParamsType action_params;

    // Construct actions
    vector<shared_ptr<Action>> action_ptrs;

    // Construct and set start state
    StateVarsType start_state_vars;

    // Construct planner
    shared_ptr<Planner> planner_ptr = make_shared<EpasePlanner>(planner_params);

    // Set actions
    planner_ptr->SetActions(action_ptrs);
    
    // Set state key generator
    planner_ptr->SetStateMapKeyGenerator(bind(StateKeyGenerator, placeholders::_1));
    
    // Set edge key generator
    planner_ptr->SetEdgeKeyGenerator(bind(EdgeKeyGenerator, placeholders::_1));
    
    // Set unary heuristic
    planner_ptr->SetHeuristicGenerator(bind(computeHeuristic, placeholders::_1));
    
    // Set binary heuristic
    planner_ptr->SetStateToStateHeuristicGenerator(bind(computeHeuristicStateToState, placeholders::_1, placeholders::_2));
    
    // Set goal condition
    planner_ptr->SetGoalChecker(bind(isGoalState, placeholders::_1));

    // Set start state
    planner_ptr->SetStartState(start_state_vars);

    // Call planner
    bool plan_found = planner_ptr->Plan();

    // Get planner Stats        
    auto planner_stats = planner_ptr->GetStats();

}