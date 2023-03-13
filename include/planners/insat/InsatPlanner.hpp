#ifndef INSAT_PLANNER_HPP
#define INSAT_PLANNER_HPP

#include <future>
#include <utility>
#include "planners/Planner.hpp"
#include <common/insat/InsatState.hpp>
#include <common/insat/InsatEdge.hpp>

namespace ps
{

    class InsatPlanner : virtual public Planner
    {
    public:

        // Typedefs
        typedef std::unordered_map<size_t, InsatStatePtrType> InsatStatePtrMapType;
        typedef smpl::intrusive_heap<InsatState, IsLesserState> InsatStateQueueMinType;

        InsatPlanner(ParamsType planner_params);;

        ~InsatPlanner() {};

        void SetStartState(const StateVarsType& state_vars);

        bool Plan();

        TrajType getSolutionTraj();

    protected:
        void initialize();

        std::vector<InsatStatePtrType> getStateAncestors(const InsatStatePtrType state_ptr, bool reverse=false) const;

        void expandState(InsatStatePtrType state_ptr);

        void updateState(InsatStatePtrType& state_ptr,
                         std::vector<InsatStatePtrType>& ancestors,
                         InsatActionPtrType& action_ptr,
                         ActionSuccessor& action_successor);

        void constructInsatActions();

        InsatStatePtrType constructInsatState(const StateVarsType& state);

        void cleanUp();

        void resetStates();

        void constructPlan(InsatStatePtrType& insat_state_ptr);

        void exit();


        std::vector<std::shared_ptr<InsatAction>> insat_actions_ptrs_;
        InsatStatePtrType start_state_ptr_;
        InsatStatePtrType goal_state_ptr_;
        InsatStateQueueMinType insat_state_open_list_;
        InsatStatePtrMapType insat_state_map_;
        TrajType soln_traj_;

    };

}

#endif
