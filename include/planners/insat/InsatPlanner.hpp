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

        InsatPlanner(ParamsType planner_params):
                Planner(planner_params)
        {

        };

        ~InsatPlanner() {};

        void SetStartState(const StateVarsType& state_vars)
        {
            start_state_ptr_ = constructInsatState(state_vars);
        }

        bool Plan()
        {
            initialize();
            auto t_start = std::chrono::steady_clock::now();

            while (!insat_state_open_list_.empty())
            {
                auto state_ptr = insat_state_open_list_.min();
                insat_state_open_list_.pop();

                // Return solution if goal state is expanded
                if (isGoalState(state_ptr))
                {
                    auto t_end = std::chrono::steady_clock::now();
                    double t_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end-t_start).count();
                    goal_state_ptr_ = state_ptr;

                    // Reconstruct and return path
                    constructPlan(state_ptr);
                    planner_stats_.total_time_ = 1e-9*t_elapsed;
                    exit();
                    return true;
                }

                expandState(state_ptr);

            }

            auto t_end = std::chrono::steady_clock::now();
            double t_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end-t_start).count();
            planner_stats_.total_time_ = 1e-9*t_elapsed;
            return false;
        }

    protected:
        void initialize()
        {

            plan_.clear();

            // Initialize planner stats
            planner_stats_ = PlannerStats();

            // Initialize start state
            start_state_ptr_->SetGValue(0);
            start_state_ptr_->SetHValue(computeHeuristic(start_state_ptr_));

            // Reset goal state
            goal_state_ptr_ = NULL;

            // Reset state
            planner_stats_ = PlannerStats();

            // Reset h_min
            h_val_min_ = DINF;

            planner_stats_.num_jobs_per_thread_.resize(1, 0);
            // Initialize open list
            start_state_ptr_->SetFValue(start_state_ptr_->GetGValue() + heuristic_w_*start_state_ptr_->GetHValue());
            insat_state_open_list_.push(start_state_ptr_);

            constructInsatActions();
        }

        std::vector<InsatStatePtrType> getStateAncestors(const InsatStatePtrType state_ptr, bool reverse=false) const
        {
            // Get ancestors
            std::vector<InsatStatePtrType> ancestors;
            ancestors.push_back(state_ptr);
            auto bp = state_ptr->GetIncomingEdgePtr();
            while (bp)
            {
                ancestors.push_back(bp->lowD_parent_state_ptr_);
                bp = bp->lowD_parent_state_ptr_->GetIncomingEdgePtr();
            }
            if (reverse)
            {
                std::reverse(ancestors.begin(), ancestors.end());
            }
            return ancestors;
        }

        void expandState(InsatStatePtrType state_ptr)
        {

            if (VERBOSE) state_ptr->Print("Expanding");

            state_ptr->SetVisited();

            auto ancestors = getStateAncestors(state_ptr);

            for (auto& action_ptr: insat_actions_ptrs_)
            {
                if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
                {
                    // Evaluate the edge
                    auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars());
                    planner_stats_.num_evaluated_edges_++; // Only the edges controllers that satisfied pre-conditions and args are in the open list
                    //********************

                    updateState(state_ptr, ancestors, action_ptr, action_successor);
                }
            }
        }

        void updateState(InsatStatePtrType& state_ptr,
                         std::vector<InsatStatePtrType>& ancestors,
                         InsatActionPtrType& action_ptr,
                         ActionSuccessor& action_successor)
        {
            if (action_successor.success_)
            {
                auto successor_state_ptr = constructInsatState(action_successor.successor_state_vars_costs_.back().first);

                if (!successor_state_ptr->IsVisited())
                {
                    InsatStatePtrType best_anc;
                    TrajType traj;
                    double cost = 0;
                    double inc_cost = 0;
                    bool root=true;
                    for (auto& anc: ancestors)
                    {
                        TrajType inc_traj = action_ptr->optimize(anc->GetStateVars(), successor_state_ptr->GetStateVars());
                        if (inc_traj.size() > 0)
                        {
                            inc_cost = action_ptr->getCost(inc_traj);
                            if (anc->GetIncomingEdgePtr()) /// When anc is not start
                            {
                                traj = action_ptr->warmOptimize(anc->GetIncomingEdgePtr()->traj_, inc_traj);
                            }
                            else
                            {
                                traj = action_ptr->warmOptimize(inc_traj);
                            }

                            if (traj.isValid())
                            {
                                best_anc = anc;
                                break;
                            }
                        }
                        else
                        {
                            continue;
                        }

//                        if (root && inc_traj.size() > 0)
//                        {
//                            root = false;
//                            inc_cost = action_ptr->getCost(inc_traj);
//                            traj = inc_traj;
//                            best_anc = anc;
//                            break;
//                        }
//                        else if (root && inc_traj.size() == 0)
//                        {
//                            root = false;
//                            continue;
//                        }
//                        else if (inc_traj.size() == 0)
//                        {
//                            continue;
//                        }
//                        else
//                        {
//                            inc_cost = action_ptr->getCost(inc_traj);
//                            traj = action_ptr->warmOptimize(anc->GetIncomingEdgePtr()->traj_, inc_traj);
//                            best_anc = anc;
//                            break;
//                        }
                    }

                    if (traj.size()==0)
                    {
                        return;
                    }

                    cost = action_ptr->getCost(traj);
                    double new_g_val = cost;

                    if (successor_state_ptr->GetGValue() > new_g_val)
                    {

                        double h_val = successor_state_ptr->GetHValue();
                        if (h_val == -1)
                        {
                            h_val = computeHeuristic(successor_state_ptr);
                            successor_state_ptr->SetHValue(h_val);
                        }

                        if (h_val != DINF)
                        {
                            h_val_min_ = h_val < h_val_min_ ? h_val : h_val_min_;
                            successor_state_ptr->SetGValue(new_g_val); //
                            successor_state_ptr->SetFValue(new_g_val + heuristic_w_*h_val); //

                            auto edge_ptr = new InsatEdge(state_ptr, action_ptr, best_anc, successor_state_ptr);
                            edge_ptr->SetTraj(traj);
                            edge_ptr->SetTrajCost(cost);
                            edge_ptr->SetCost(inc_cost);
                            edge_map_.insert(std::make_pair(getEdgeKey(edge_ptr), edge_ptr));
                            successor_state_ptr->SetIncomingEdgePtr(edge_ptr); //

                            if (insat_state_open_list_.contains(successor_state_ptr))
                            {
                                insat_state_open_list_.decrease(successor_state_ptr);
                            }
                            else
                            {
                                insat_state_open_list_.push(successor_state_ptr);
                            }
                        }
                    }
                }
            }
        }

        void constructInsatActions()
        {
            for (auto& action_ptr : actions_ptrs_)
            {
                insat_actions_ptrs_.emplace_back(std::dynamic_pointer_cast<InsatAction>(action_ptr));
            }
        }

        InsatStatePtrType constructInsatState(const StateVarsType& state)
        {
            size_t key = state_key_generator_(state);
            auto it = insat_state_map_.find(key);
            InsatStatePtrType insat_state_ptr;

            // Check if state exists in the search state map
            if (it == insat_state_map_.end())
            {
                insat_state_ptr = new InsatState(state);
                insat_state_map_.insert(std::pair<size_t, InsatStatePtrType>(key, insat_state_ptr));
            }
            else
            {
                insat_state_ptr = it->second;
            }

            return insat_state_ptr;
        }

        void cleanUp()
        {
            for (auto& state_it : insat_state_map_)
            {
                if (state_it.second)
                {
                    delete state_it.second;
                    state_it.second = NULL;
                }
            }
            insat_state_map_.clear();

            for (auto& edge_it : edge_map_)
            {
                if (edge_it.second)
                {
                    delete edge_it.second;
                    edge_it.second = NULL;
                }
            }
            edge_map_.clear();

            State::ResetStateIDCounter();
            Edge::ResetStateIDCounter();
        }

        void resetStates()
        {
            for (auto it = insat_state_map_.begin(); it != insat_state_map_.end(); ++it)
            {
                it->second->ResetGValue();
                it->second->ResetFValue();
                // it->second->ResetVValue();
                it->second->ResetIncomingEdgePtr();
                it->second->UnsetVisited();
                it->second->UnsetBeingExpanded();
                it->second->num_successors_ = 0;
                it->second->num_expanded_successors_ = 0;
            }
        }

        void constructPlan(InsatStatePtrType& state_ptr)
        {
            if (state_ptr->GetIncomingEdgePtr())
            {
//                planner_stats_.path_cost_ = state_ptr->GetIncomingEdgePtr()->GetTrajCost();
                planner_stats_.path_cost_ =
                        insat_actions_ptrs_[0]->getCost(state_ptr->GetIncomingEdgePtr()->traj_);
                soln_traj_ = state_ptr->GetIncomingEdgePtr()->traj_;
            }
            while(state_ptr->GetIncomingEdgePtr())
            {
                if (state_ptr->GetIncomingEdgePtr()) // For start state_ptr, there is no incoming edge
                    plan_.insert(plan_.begin(), PlanElement(state_ptr->GetStateVars(), state_ptr->GetIncomingEdgePtr()->action_ptr_, state_ptr->GetIncomingEdgePtr()->GetCost()));
                else
                    plan_.insert(plan_.begin(), PlanElement(state_ptr->GetStateVars(), NULL, 0));

                state_ptr = state_ptr->GetIncomingEdgePtr()->fullD_parent_state_ptr_;
            }
            planner_stats_.path_length_ += plan_.size();
        }
        

        void exit()
        {
            // Clear open list
            while (!insat_state_open_list_.empty())
            {
                insat_state_open_list_.pop();
            }

            cleanUp();
        }


        std::vector<std::shared_ptr<InsatAction>> insat_actions_ptrs_;
        InsatStatePtrType start_state_ptr_;
        InsatStatePtrType goal_state_ptr_;
        InsatStateQueueMinType insat_state_open_list_;
        InsatStatePtrMapType insat_state_map_;
        TrajType soln_traj_;

    };

}

#endif
