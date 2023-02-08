#ifndef INSAT_PLANNER_HPP
#define INSAT_PLANNER_HPP

#include <future>
#include "planners/Planner.hpp"

namespace ps
{

    template<typename EnvType, typename OptType>
    class InsatstarPlanner : public Planner
    {
    public:

        typedef typename EnvType::Ptr EnvPtrType;
        typedef typename EnvType::TrajType TrajType;

        InsatstarPlanner(ParamsType planner_params):
                Planner(planner_params)
        {

        };

        ~InsatstarPlanner() {};

        bool Plan()
        {
            initialize();
            auto t_start = std::chrono::steady_clock::now();

            while (!state_open_list_.empty())
            {
                auto state_ptr = state_open_list_.min();
                state_open_list_.pop();

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
            Planner::initialize();
            planner_stats_.num_jobs_per_thread_.resize(1, 0);

            // Initialize open list
            start_state_ptr_->SetFValue(start_state_ptr_->GetGValue() + heuristic_w_*start_state_ptr_->GetHValue());
            state_open_list_.push(start_state_ptr_);

            planner_stats_.num_threads_spawned_ = 1;
        }

        void expandState(StatePtrType state_ptr)
        {

            if (VERBOSE) state_ptr->Print("Expanding");

            planner_stats_.num_jobs_per_thread_[0] +=1;
            planner_stats_.num_state_expansions_++;

            state_ptr->SetVisited();

            // Get ancestors
            std::vector<StatePtrType> ancestors;
            StatePtrType bp = state_ptr;
            while (bp)
            {
                ancestors.push_back(bp);
                bp = state_ptr->GetIncomingEdgePtr()->parent_state_ptr_;
            }
            std::reverse(ancestors.begin(), ancestors.end());

            for (auto& action_ptr: actions_ptrs_)
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

        void updateState(StatePtrType& state_ptr,
                         std::vector<StatePtrType>& ancestors,
                         ActionPtrType& action_ptr,
                         ActionSuccessor& action_successor)
        {
            if (action_successor.success_)
            {
                auto successor_state_ptr = constructState(action_successor.successor_state_vars_costs_.back().first);

                TrajType traj;
                bool root=true;
                for (auto& anc: ancestors)
                {
                    TrajType inc_traj = env_->optimize(anc->GetStateVars(), successor_state_ptr->GetStateVars());
                    if (root && inc_traj.size() == 0)
                    {
                      root = false;
                      continue;
                    }
                    else if (inc_traj.size() == 0)
                    {
                      continue;
                    }
                    else
                    {
                      traj = env_->warmOptimize(anc->GetInsatEdge(), inc_traj);
                    }
                }

                double cost = env_->calculateCost(traj);

                if (!successor_state_ptr->IsVisited())
                {
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
                            successor_state_ptr->SetGValue(new_g_val);
                            successor_state_ptr->SetFValue(new_g_val + heuristic_w_*h_val);

                            auto edge_ptr = new Edge(state_ptr, successor_state_ptr, action_ptr);
                            edge_ptr->SetCost(cost);
                            edge_map_.insert(make_pair(getEdgeKey(edge_ptr), edge_ptr));

                            successor_state_ptr->SetIncomingEdgePtr(edge_ptr);
                            successor_state_ptr->SetInsatEdge(traj);

                            if (state_open_list_.contains(successor_state_ptr))
                            {
                                state_open_list_.decrease(successor_state_ptr);
                            }
                            else
                            {
                                state_open_list_.push(successor_state_ptr);
                            }

                        }

                    }
                }
            }
        }


        void exit()
        {
            // Clear open list
            while (!state_open_list_.empty())
            {
                state_open_list_.pop();
            }

            Planner::exit();
        }


        int num_threads_;
        StateQueueMinType state_open_list_;

        EnvPtrType  env_;
        OptType opt_;

    };

}

#endif
