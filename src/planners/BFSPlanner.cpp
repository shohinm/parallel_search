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
 * \file   BFSPlanner.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   3/11/23
 */

#include <iostream>
#include <planners/BFSPlanner.hpp>

namespace ps
{
  BFSPlanner::BFSPlanner(ParamsType planner_params):
      Planner(planner_params)
  {

  }

  BFSPlanner::~BFSPlanner()
  {

  }

  bool BFSPlanner::Plan()
  {
    initialize();
    startTimer();
    while (!state_open_list_.empty() && !checkTimeout())
    {
      auto state_ptr = state_open_list_.front();
      state_open_list_.pop_front();

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

  void BFSPlanner::initialize()
  {
    Planner::initialize();
    planner_stats_.num_jobs_per_thread_.resize(1, 0);

    // Initialize the queue
    state_open_list_.push_back(start_state_ptr_);

    planner_stats_.num_threads_spawned_ = 1;
  }

  void BFSPlanner::expandState(StatePtrType state_ptr)
  {

    if (VERBOSE) state_ptr->Print("Expanding");

    planner_stats_.num_jobs_per_thread_[0] +=1;
    planner_stats_.num_state_expansions_++;

    state_ptr->SetVisited();

    for (auto& action_ptr: actions_ptrs_)
    {
      if (action_ptr->CheckPreconditions(state_ptr->GetStateVars()))
      {
        // Evaluate the edge
        auto action_successor = action_ptr->GetSuccessor(state_ptr->GetStateVars());
        planner_stats_.num_evaluated_edges_++; // Only the edges controllers that satisfied pre-conditions and args are in the open list
        //********************

        updateState(state_ptr, action_ptr, action_successor);
      }
    }
  }

  void BFSPlanner::updateState(StatePtrType& state_ptr, ActionPtrType& action_ptr, ActionSuccessor& action_successor)
  {
    if (action_successor.success_)
    {
      auto successor_state_ptr = constructState(action_successor.successor_state_vars_costs_.back().first);

      if (!successor_state_ptr->IsVisited())
      {
        auto edge_ptr = new Edge(state_ptr, action_ptr, successor_state_ptr);
        edge_ptr->SetCost(1.0);
        edge_map_.insert(std::make_pair(getEdgeKey(edge_ptr), edge_ptr));
        successor_state_ptr->SetIncomingEdgePtr(edge_ptr);
        state_open_list_.push_back(successor_state_ptr);
      }
    }
  }

  void BFSPlanner::exit()
  {
    // Clear open list
    while (!state_open_list_.empty())
    {
      state_open_list_.pop_front();
    }

    Planner::exit();
  }
}
