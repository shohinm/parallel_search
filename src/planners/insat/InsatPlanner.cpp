/*
 * Copyright (c) 2023, Ramkumar Natarajan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   InsatPlanner.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   2/26/23
 */

#include <planners/insat/InsatPlanner.hpp>

namespace ps
{


    InsatPlanner::InsatPlanner(ParamsType planner_params) :
            Planner(planner_params)
    {
        if (planner_params.find("adaptive_opt") == planner_params.end())
        {
            planner_params["adaptive_opt"] = false;
        }
    }

    void InsatPlanner::SetStartState(const StateVarsType &state_vars) {
        start_state_ptr_ = constructInsatState(state_vars);
    }

    bool InsatPlanner::Plan() {
        initialize();
        startTimer();
        while (!insat_state_open_list_.empty() && !checkTimeout())
        {
            auto state_ptr = insat_state_open_list_.min();
            insat_state_open_list_.pop();

            // Return solution if goal state is expanded
            if (isGoalState(state_ptr))
            {
                auto t_end = std::chrono::steady_clock::now();
                double t_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end-t_start_).count();
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
        double t_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end-t_start_).count();
        planner_stats_.total_time_ = 1e-9*t_elapsed;
        return false;
    }

    TrajType InsatPlanner::getSolutionTraj() {
        return soln_traj_;
    }

    void InsatPlanner::initialize() {

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

    std::vector<InsatStatePtrType>
    InsatPlanner::getStateAncestors(const InsatStatePtrType state_ptr, bool reverse) const {
        // Get ancestors
        std::vector<InsatStatePtrType> ancestors;
        ancestors.push_back(state_ptr);
        auto bp = state_ptr->GetIncomingInsatEdgePtr();
        while (bp)
        {
            ancestors.push_back(bp->lowD_parent_state_ptr_);
            bp = bp->lowD_parent_state_ptr_->GetIncomingInsatEdgePtr();
        }
        if (reverse)
        {
            std::reverse(ancestors.begin(), ancestors.end());
        }
        return ancestors;
    }

    void InsatPlanner::expandState(InsatStatePtrType state_ptr) {

        if (VERBOSE) state_ptr->Print("Expanding");
        planner_stats_.num_state_expansions_++;

        state_ptr->SetVisited();

        auto ancestors = getStateAncestors(state_ptr);

        for (auto& action_ptr: insat_actions_ptrs_)
        {
            if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
            {
                // Evaluate the edge
                auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars());

                updateState(state_ptr, ancestors, action_ptr, action_successor);
            }
        }
    }

    void InsatPlanner::updateState(InsatStatePtrType &state_ptr, std::vector<InsatStatePtrType> &ancestors,
                                   InsatActionPtrType &action_ptr, ActionSuccessor &action_successor) {
        planner_stats_.num_evaluated_edges_++;

        if (action_successor.success_)
        {
            auto successor_state_ptr = constructInsatState(action_successor.successor_state_vars_costs_.back().first);

            if (!successor_state_ptr->IsVisited())
            {
                InsatStatePtrType best_anc;
                TrajType traj;
                double cost = 0;
                double inc_cost = 0;

                if (planner_params_["smart_opt"] == true)
                {
                    std::vector<StateVarsType> anc_states;
                    for (auto& anc: ancestors)
                    {
                        anc_states.emplace_back(anc->GetStateVars());
                    }

                    if (state_ptr->GetIncomingInsatEdgePtr()) /// When anc is not start
                    {
                        traj = action_ptr->optimize(state_ptr->GetIncomingInsatEdgePtr()->GetTraj(),
                                                    anc_states,
                                                    successor_state_ptr->GetStateVars());
                        inc_cost = action_ptr->getCost(traj) - action_ptr->getCost(state_ptr->GetIncomingInsatEdgePtr()->GetTraj());
                    }
                    else
                    {
                        traj = action_ptr->optimize(TrajType(),
                                                    anc_states,
                                                    successor_state_ptr->GetStateVars());
                        inc_cost = action_ptr->getCost(traj);
                    }
                    if (traj.isValid())
                    {
                        best_anc = start_state_ptr_;
                    }
                }
                else
                {
                    for (auto& anc: ancestors)
                    {
                        if (planner_params_["adaptive_opt"] == true)
                        {
                            if (anc->GetIncomingInsatEdgePtr()) /// When anc is not start
                            {
                                traj = action_ptr->optimize(anc->GetIncomingInsatEdgePtr()->GetTraj(),
                                                            anc->GetStateVars(),
                                                            successor_state_ptr->GetStateVars());
                                inc_cost = action_ptr->getCost(traj) - action_ptr->getCost(
                                    anc->GetIncomingInsatEdgePtr()->GetTraj());
                            }
                            else
                            {
                                traj = action_ptr->optimize(TrajType(),
                                                            anc->GetStateVars(),
                                                            successor_state_ptr->GetStateVars());
                                inc_cost = action_ptr->getCost(traj);
                            }
                        }
                        else
                        {
                            TrajType inc_traj = action_ptr->optimize(anc->GetStateVars(), successor_state_ptr->GetStateVars());
                            if (inc_traj.size() > 0)
                            {
                                inc_cost = action_ptr->getCost(inc_traj);
                                if (anc->GetIncomingInsatEdgePtr()) /// When anc is not start
                                {
                                    traj = action_ptr->warmOptimize(anc->GetIncomingInsatEdgePtr()->GetTraj(), inc_traj);
                                }
                                else
                                {
                                    traj = action_ptr->warmOptimize(inc_traj);
                                }
                            }
                            else
                            {
                                continue;
                            }

                        }
                        if (traj.isValid())
                        {
                            best_anc = anc;
                        }
                    }
                }

                if (traj.disc_traj_.cols()<=2)
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

                        auto edge_ptr = new Edge(state_ptr, action_ptr, successor_state_ptr);
                        edge_ptr->SetCost(inc_cost);
                        successor_state_ptr->SetIncomingEdgePtr(edge_ptr);

                        auto insat_edge_ptr = new InsatEdge(state_ptr, action_ptr, best_anc, successor_state_ptr);
                        insat_edge_ptr->SetTraj(traj);
                        insat_edge_ptr->SetTrajCost(cost);
                        insat_edge_ptr->SetCost(cost);
                        if (isGoalState(successor_state_ptr))
                        {
                            insat_edge_ptr->SetTrajCost(0);
                            insat_edge_ptr->SetCost(0);
                            successor_state_ptr->SetFValue(0.0);
                        }
                        edge_map_.insert(std::make_pair(getEdgeKey(insat_edge_ptr), insat_edge_ptr));
                        successor_state_ptr->SetIncomingInsatEdgePtr(insat_edge_ptr); //

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

    void InsatPlanner::constructInsatActions() {
        for (auto& action_ptr : actions_ptrs_)
        {
            insat_actions_ptrs_.emplace_back(std::dynamic_pointer_cast<InsatAction>(action_ptr));
        }
    }

    InsatStatePtrType InsatPlanner::constructInsatState(const StateVarsType &state) {
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

    void InsatPlanner::cleanUp() {
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

    void InsatPlanner::resetStates() {
        for (auto it = insat_state_map_.begin(); it != insat_state_map_.end(); ++it)
        {
            it->second->ResetGValue();
            it->second->ResetFValue();
            // it->second->ResetVValue();
          it->second->ResetIncomingInsatEdgePtr();
            it->second->UnsetVisited();
            it->second->UnsetBeingExpanded();
            it->second->num_successors_ = 0;
            it->second->num_expanded_successors_ = 0;
        }
    }

    void InsatPlanner::constructPlan(InsatStatePtrType &insat_state_ptr) {
      StatePtrType state_ptr = insat_state_ptr;
      Planner::constructPlan(state_ptr);

      if (insat_state_ptr->GetIncomingInsatEdgePtr())
        {
//                planner_stats_.path_cost_ = insat_state_ptr->GetIncomingInsatEdgePtr()->GetTrajCost();
            planner_stats_.path_cost_ =
                    insat_actions_ptrs_[0]->getCost(insat_state_ptr->GetIncomingInsatEdgePtr()->GetTraj());
            soln_traj_ = insat_state_ptr->GetIncomingInsatEdgePtr()->GetTraj();
        }
    }

    void InsatPlanner::exit() {
        // Clear open list
        while (!insat_state_open_list_.empty())
        {
            insat_state_open_list_.pop();
        }

        cleanUp();
    }
}
