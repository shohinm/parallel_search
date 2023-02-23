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
 * \file   ManipulationActions.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   2/19/23
 */

#include "ManipulationActions.hpp"
#include <common/GlorifiedAngles.h>
#include <cstdlib>

namespace ps
{
  ManipulationAction::ManipulationAction(const std::string& type,
                                         ParamsType params,
                                         std::string& mj_modelpath,
                                         VecDf ang_discretization,
                                         OptVecPtrType& opt,
                                         MjModelVecType& m_vec, MjDataVecType& d_vec,
                                         int num_threads,
                                         bool is_expensive) : InsatAction(type, params, is_expensive),
                                                          discretization_(ang_discretization),
                                                          opt_(opt), m_(m_vec), d_(d_vec)
  {

    for (int i=0; i<num_threads; ++i)
    {
      mjModel* mod = mj_loadXML(mj_modelpath.c_str(), nullptr, nullptr, 0);
      mjData* dat = mj_makeData(mod);
      m_.push_back(mod);
      d_.push_back(dat);
//      m_.emplace_back(std::make_shared<mjModel>());
//      d_.emplace_back(mj_makeData(m_[i]));
    }

    // Caching discrete angles per DOF in the range -M_PI to M_PI
    for (int i=0; i<m_[0]->nq; ++i)
    {
      double ang_lim = discretization_(i)*static_cast<int>(M_PI/discretization_(i));
      int num_angles = 1+(2*static_cast<int>(M_PI/discretization_(i)));
      discrete_angles_[i] = VecDf::LinSpaced(num_angles, -ang_lim, ang_lim);
    }

    joint_limits_ = getJointLimits(0);
    gen_ = std::mt19937(1);

  }

  bool ManipulationAction::CheckPreconditions(StateVarsType state)
  {
    return true;
  }

  ActionSuccessor ManipulationAction::GetSuccessor(StateVarsType state_vars, int thread_id)
  {
    Eigen::Map<const VecDf> state(&state_vars[0], state_vars.size());
    VecDf successor = GetSuccessor(state, thread_id);

    if (successor.size()==0)
    {
        return ActionSuccessor(false, {make_pair(StateVarsType(), -DINF)});
    }
    else
    {
        std::vector<std::pair<StateVarsType, double>> action_successors;
        StateVarsType succ;
        succ.resize(m_[thread_id]->nq);
        VecDf::Map(&succ[0], successor.size()) = successor;
        double cost = getCostToSuccessor(state, successor, thread_id);
        return ActionSuccessor(true, {std::make_pair(succ, cost)});
    }
  }

  ActionSuccessor ManipulationAction::GetSuccessorLazy(StateVarsType state_vars, int thread_id)
  {

  }

  ActionSuccessor ManipulationAction::Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id)
  {
    return GetSuccessor(parent_state_vars, thread_id);
  }

  /// Snap to Grid
  VecDf ManipulationAction::contToDisc(const VecDf & cont_state, int thread_id)
  {
    VecDf disc_state(m_[thread_id]->nq);
    for (int i=0; i<m_[thread_id]->nq; ++i)
    {
      // Normalize angle to -pi to pi. Should already be in that range.
      // cont_state(i) = angles::normalize_angle(cont_state(i));
      // Number of discrete angles in the DOF i
      int n_disc = discrete_angles_[i].size();
      // The offset to be added to find the bin because of negative to positive range
      int offset = (n_disc-1)/2;
      // One of the indexes of the bin (idx1)
      int idx1 = static_cast<int>(cont_state(i)/discretization_(i)) + offset;
      // The idx1 should not exceed bounds
      assert(idx1>=0 && idx1<discrete_angles_[i].size());
      // The second index (idx2) based on idx1
      int idx2 = cont_state(i)>discrete_angles_[i](idx1)?idx1+1:idx1-1;
      idx2 = (idx2<0)?discrete_angles_[i].size()-1:idx2;
      idx2 = (idx2==discrete_angles_[i].size())?0:idx2;
      // The distance to the angles from two instances
      double d1 = fabs(angles::shortest_angular_distance(cont_state(i), discrete_angles_[i](idx1)));
      double d2 = fabs(angles::shortest_angular_distance(cont_state(i), discrete_angles_[i](idx2)));
      // The distance to the angles from two instances
      disc_state(i) = (d1 < d2)?
                      discrete_angles_[i](idx1):
                      discrete_angles_[i](idx2);
    }
    return disc_state;
  }

  /// MuJoCo
  VecDf ManipulationAction::GetSuccessor(const VecDf &state, int thread_id)
  {
    int prim_id = std::stoi(Action::type_);
    if (prim_id < 2*m_[thread_id]->nq)
    {
        VecDf succ(m_[thread_id]->nq);
        for (int j=0; j<m_[thread_id]->nq; ++j)
        {
            succ(j) = state(j) + mprims_(prim_id,j)*discretization_(j);
            succ(j) = angles::normalize_angle(succ(j));
        }
        succ = contToDisc(succ, thread_id);

        if (!validateJointLimits(succ, thread_id))
        {
            VecDf empty;
            assert(empty.size() == 0);
            return empty;
        }

        VecDf free_state(m_[thread_id]->nq), con_state(m_[thread_id]->nq);

        // state coll check
        if (isCollisionFree(succ, thread_id))
        {
            // edge check only if state check passes
            if (isCollisionFree(state, succ, free_state, thread_id))
            {
                return succ;
            }
            VecDf empty;
            assert(empty.size() == 0);
            return empty;
        }
    }
    else
    {
        /// Direct edge to goal
      VecDf free_state(m_[thread_id]->nq);
      if (isCollisionFree(state, goal_, free_state, thread_id))
      {
        return goal_;
      }
    }
    VecDf empty;
    assert(empty.size() == 0);
    return empty;
  }

  bool ManipulationAction::IsFeasible(const StateVarsType& state_vars, int thread_id)
  {
    return isCollisionFree(state_vars, thread_id);
  }

  bool ManipulationAction::isCollisionFree(const StateVarsType &state_vars, int thread_id) const
  {
    Eigen::Map<const VecDf> state(&state_vars[0], state_vars.size());
    return isCollisionFree(state, thread_id);
  }

  bool ManipulationAction::isCollisionFree(const VecDf &state, int thread_id) const
  {
    if (!validateJointLimits(state, thread_id))
    {
      return false;
    }
    // Set curr configuration
    mju_copy(d_[thread_id]->qpos, state.data(), m_[thread_id]->nq);
    mju_zero(d_[thread_id]->qvel, m_[thread_id]->nv);
    mju_zero(d_[thread_id]->qacc, m_[thread_id]->nv);
    mj_fwdPosition(m_[thread_id], d_[thread_id]);

    return d_[thread_id]->ncon>0? false: true;
  }

  bool ManipulationAction::isCollisionFree(const VecDf &curr, const VecDf &succ, VecDf &free_state, int thread_id) const
  {
    double ang_dist = angles::calcAngDist(curr, succ);
    int n = static_cast<int>(ceil(ang_dist/(1.5*2e-3)));
    double rho = 1.0/n;

    double coll_free = true;
    free_state = curr;
    for (int k=0; k<=n; ++k)
    {
      VecDf interp = angles::interpolateAngle(curr, succ, rho*k);
      if (!isCollisionFree(interp, thread_id))
      {
        coll_free = false;
        break;
      }
      free_state = interp;
    }
    return coll_free;
  }

  double getRandomNumberBetween(double min, double max, std::mt19937& gen)
  {
      std::uniform_real_distribution<double> distr(min, max);
      return distr(gen);
  }

  StateVarsType ManipulationAction::SampleFeasibleState(int thread_id)
  {
    std::vector<double> sampled_joints(m_[thread_id]->njnt);
    bool is_feasible = false;
    while (!is_feasible)
    {

      for (int i=0; i<m_[thread_id]->njnt; ++i)
      {
        sampled_joints[i] = getRandomNumberBetween(joint_limits_[i].first, joint_limits_[i].second, gen_);
      }

      is_feasible = validateJointLimits(sampled_joints, thread_id);
    }

    return sampled_joints;

  }

  std::vector<std::pair<double, double>> ManipulationAction::getJointLimits(int thread_id) const
  {
    std::vector<std::pair<double, double>> joint_limits(m_[thread_id]->njnt);
    for (int i=0; i<m_[thread_id]->njnt; ++i)
    {
      joint_limits[i] = std::make_pair(m_[thread_id]->jnt_range[2*i], m_[thread_id]->jnt_range[2*i+1]);
    }
    return joint_limits;
  }

  bool ManipulationAction::validateJointLimits(const VecDf &state, int thread_id) const
  {
    bool valid = true;
    for (int i=0; i<m_[thread_id]->njnt; ++i)
    {
      if (state(i) >= m_[thread_id]->jnt_range[2*i] && state(i) <= m_[thread_id]->jnt_range[2*i+1])
      {
        continue;
      }
      valid = false;
      break;
    }
    return valid;
  }

  bool ManipulationAction::validateJointLimits(const StateVarsType &state_vars, int thread_id) const
  {
    Eigen::Map<const VecDf> state(&state_vars[0], state_vars.size());
    return validateJointLimits(state, thread_id);
  }

  double ManipulationAction::getCostToSuccessor(const VecDf &current_state, const VecDf &successor_state, int thread_id)
  {
    VecDf angle_dist(m_[thread_id]->nq);
    for (int i=0; i<m_[thread_id]->nq; ++i)
    {
      angle_dist(i) = angles::shortest_angular_distance(current_state(i),
                                                        successor_state(i));
    }
    return angle_dist.norm();
  }


  /// INSAT
  void ManipulationAction::setOpt(OptVecPtrType& opt)
  {
    opt_ = opt;
  }

  bool ManipulationAction::isFeasible(MatDf &traj, int thread_id) const
  {
    bool feas = true;
    for (int i=0; i<traj.cols(); ++i)
    {
      if (!isCollisionFree(traj.col(i), thread_id))
      {
        feas = false;
        break;
      }
    }
    return feas;
  }

  TrajType ManipulationAction::optimize(const StateVarsType &s1,
                                      const StateVarsType &s2,
                                      int thread_id) const
  {
    Eigen::Map<const VecDf> p1(&s1[0], s1.size());
    Eigen::Map<const VecDf> p2(&s2[0], s2.size());

    return (*opt_)[thread_id].optimize(this, p1, p2, thread_id);
  }

  TrajType ManipulationAction::warmOptimize(const TrajType &t1,
                                          const TrajType &t2,
                                          int thread_id) const
  {
    return (*opt_)[thread_id].warmOptimize(this, t1, t2, thread_id);
  }

  double ManipulationAction::getCost(const TrajType &traj, int thread_id) const
  {
    return (*opt_)[thread_id].calculateCost(traj);
  }

}
