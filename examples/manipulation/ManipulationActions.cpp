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

#include <cstdlib>
#include "ManipulationActions.hpp"

namespace ps
{
  ManipulationAction::ManipulationAction(const std::string& type,
                                         ParamsType params,
                                         Mode mode,
                                         double discretization,
                                         MatDf& mprims,
                                         OptVecPtrType opt,
                                         MjModelVecType& m_vec, MjDataVecType& d_vec,
                                         int num_threads,
                                         bool is_expensive) : InsatAction(type, params, is_expensive),
                                                          mprim_mode_(mode),  discretization_(discretization),
                                                          opt_(opt), m_(m_vec), d_(d_vec), mprims_(mprims)
  {
    if (mprim_mode_ == Mode::CSPACE)
    {
      // Caching discrete angles per DOF in the robot joint angle range
      VecDf disc_min_ang(m_[0]->nq);
      VecDf disc_max_ang(m_[0]->nq);
      VecDf num_angles(m_[0]->nq);

      for (int i=0; i<m_[0]->nq; ++i)
      {
        disc_min_ang(i) = discretization_*
                          static_cast<int>(robot_params_.min_q_(i)/discretization_);
        disc_max_ang(i) = discretization_*
                          static_cast<int>(robot_params_.max_q_(i)/discretization_);
        num_angles(i) = 1+(static_cast<int>((disc_max_ang(i)-disc_min_ang(i))/discretization_));
        discrete_angles_[i] = VecDf::LinSpaced(num_angles(i), disc_min_ang(i), disc_max_ang(i));
      }

      joint_limits_ = getJointLimits(0);
      gen_ = std::mt19937(1);
    }
    else if (mprim_mode_ == Mode::TASKSPACE3D)
    {
      // Caching discrete angles per DOF in the robot joint angle range
      VecDf disc_min_xyz(3);
      VecDf disc_max_xyz(3);
      VecDf num_xyz(3);

      for (int i=0; i<3; ++i)
      {
        disc_min_xyz(i) = discretization_*
                          static_cast<int>(m_vec[0]->numeric_data[i]/discretization_);
        disc_max_xyz(i) = discretization_*
                          static_cast<int>(m_vec[0]->numeric_data[3+i]/discretization_);
        num_xyz(i) = 1+(static_cast<int>((disc_max_xyz(i)-disc_min_xyz(i))/discretization_));
        discrete_angles_[i] = VecDf::LinSpaced(num_xyz(i), disc_min_xyz(i), disc_max_xyz(i));
      }
    }

  }

  bool ManipulationAction::CheckPreconditions(const StateVarsType& state)
  {
    return true;
  }

  ActionSuccessor ManipulationAction::GetSuccessor(const StateVarsType& state_vars, int thread_id)
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
        if (mprim_mode_ == Mode::CSPACE)
        {
          succ.resize(m_[thread_id]->nq);
        }
        else if (mprim_mode_ == Mode::TASKSPACE3D)
        {
          succ.resize(3);
        }
        VecDf::Map(&succ[0], successor.size()) = successor;
        double cost = getCostToSuccessor(state, successor, thread_id);
        return ActionSuccessor(true, {std::make_pair(succ, cost)});
    }
  }

  ActionSuccessor ManipulationAction::GetSuccessorLazy(const StateVarsType& state_vars, int thread_id)
  {

  }

  ActionSuccessor ManipulationAction::Evaluate(const StateVarsType& parent_state_vars, const StateVarsType& child_state_vars, int thread_id)
  {
    return GetSuccessor(parent_state_vars, thread_id);
  }

  /// Snap to Grid
  VecDf ManipulationAction::contToDisc(const VecDf & cont_state, int thread_id)
  {
    VecDf disc_state;
    if (mprim_mode_ == Mode::CSPACE)
    {
      disc_state.resize(m_[thread_id]->nq);
      for (int i=0; i<m_[thread_id]->nq; ++i)
      {
        Eigen::Index index;
        VecDf candi = cont_state(i)*VecDf::Ones(discrete_angles_[i].size());
        (discrete_angles_[i] - candi).rowwise().squaredNorm().minCoeff(&index);
        disc_state(i) = discrete_angles_[i](index);
      }
    }
    else if (mprim_mode_ == Mode::TASKSPACE3D)
    {
      disc_state.resize(3);
      for (int i=0; i<3; ++i)
      {
        Eigen::Index index;
        VecDf candi = cont_state(i)*VecDf::Ones(discrete_angles_[i].size());
        (discrete_angles_[i] - candi).rowwise().squaredNorm().minCoeff(&index);
        disc_state(i) = discrete_angles_[i](index);
      }
    }

    return disc_state;
  }

  /// MuJoCo
  VecDf ManipulationAction::GetSuccessor(const VecDf &state, int thread_id)
  {
    int prim_id = std::stoi(Action::type_);
    if (mprim_mode_ == Mode::CSPACE)
    {
      if (prim_id < mprims_.rows())
      {
        VecDf succ(m_[thread_id]->nq);
        for (int j=0; j<m_[thread_id]->nq; ++j)
        {
          succ(j) = state(j) + mprims_(prim_id,j);
        }
        succ = contToDisc(succ, thread_id);

        /// If successor is the same as the state. This will happen if the joint angle is close its limit.
        if (state.isApprox(succ, 1e-3)) /// assuming discretization is never finer than 1e-3.
        {
          VecDf empty;
          assert(empty.size() == 0);
          return empty;
        }

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
        /// If successor is the same as the state. This will happen if the joint angle is close its limit.
        /// if expanded state is too close to goal
        if (state.isApprox(goal_, 1e-3) || params_["planner_type"]==1) /// assuming discretization is never finer than 1e-3.
        {
          return goal_;
        }
        /// if goal is in LoS then return it
        VecDf free_state(m_[thread_id]->nq);
        if (isCollisionFree(state, goal_, free_state, thread_id))
        {
          return goal_;
        }
      }
    }
    else if (mprim_mode_ == Mode::TASKSPACE3D)
    {
      VecDf succ(3);
      for (int j=0; j<3; ++j)
      {
        succ(j) = state(j) + mprims_(prim_id,j);
      }
      succ = contToDisc(succ, thread_id);

      /// If successor is the same as the state. This will happen if the joint angle is close its limit.
      if (state.isApprox(succ, 1e-3)) /// assuming discretization is never finer than 1e-3.
      {
        VecDf empty;
        assert(empty.size() == 0);
        return empty;
      }

      if (!validateJointLimits(succ, thread_id))
      {
        VecDf empty;
        assert(empty.size() == 0);
        return empty;
      }

      // state coll check
      if (isCollisionFree(succ, thread_id))
      {
        return succ;
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
    if (mprim_mode_ == Mode::CSPACE)
    {
      mju_copy(d_[thread_id]->qpos, state.data(), m_[thread_id]->nq);
    }
    else if (mprim_mode_ == Mode::TASKSPACE3D)
    {
      VecDf fullstate(7);
      fullstate << state(0), state(1), state(2), 1, 0, 0, 0;
      mju_copy(d_[thread_id]->qpos, fullstate.data(), m_[thread_id]->nq);
    }
    mju_zero(d_[thread_id]->qvel, m_[thread_id]->nv);
    mju_zero(d_[thread_id]->qacc, m_[thread_id]->nv);
    mj_fwdPosition(m_[thread_id], d_[thread_id]);


    return d_[thread_id]->ncon>0? false: true;
  }


  bool ManipulationAction::isCollisionFree(const VecDf &curr, const VecDf &succ, VecDf &free_state, int thread_id) const
  {
    if (mprim_mode_ != Mode::CSPACE)
    {
      std::runtime_error("ManipulationAction::isCollisionFree NOT implemented except in CSPACE mode");
    }

    double ang_dist = (succ-curr).norm();
    double dx = discretization_/3.0; /// Magic number 3.0 will prevent roundoff errors
    int n = static_cast<int>(ceil(ang_dist/(dx)));
    MatDf edge = linInterp(curr, succ, n);

    double coll_free = true;
    free_state = curr;
    for (int k=0; k<n; ++k)
    {
      if (!isCollisionFree(edge.col(k), thread_id))
      {
        coll_free = false;
        break;
      }
      free_state = edge.col(k);
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
    if (mprim_mode_ == Mode::CSPACE)
    {
      bool is_feasible = false;
      while (!is_feasible)
      {

        for (int i=0; i<m_[thread_id]->njnt; ++i)
        {
          sampled_joints[i] = getRandomNumberBetween(joint_limits_[i].first, joint_limits_[i].second, gen_);
        }

        is_feasible = validateJointLimits(sampled_joints, thread_id);
      }
    }
    else
    {
      std::runtime_error("ManipulationAction::SampleFeasibleState NOT implemented except in CSPACE mode");
    }

    return sampled_joints;

  }

  std::vector<std::pair<double, double>> ManipulationAction::getJointLimits(int thread_id) const
  {
    std::vector<std::pair<double, double>> joint_limits(m_[thread_id]->njnt);
    if (mprim_mode_ == Mode::CSPACE)
    {
      for (int i=0; i<m_[thread_id]->njnt; ++i)
      {
        joint_limits[i] = std::make_pair(m_[thread_id]->jnt_range[2*i], m_[thread_id]->jnt_range[2*i+1]);
      }
    }
    else
    {
      std::runtime_error("ManipulationAction::getJointLimits NOT implemented except in CSPACE mode");
    }
    return joint_limits;
  }

  bool ManipulationAction::validateJointLimits(const VecDf &state, int thread_id) const
  {
    bool valid = true;
    if (mprim_mode_ == Mode::CSPACE)
    {
      for (int i=0; i<m_[thread_id]->njnt; ++i)
      {
        if (state(i) >= m_[thread_id]->jnt_range[2*i] && state(i) <= m_[thread_id]->jnt_range[2*i+1])
        {
          continue;
        }
        valid = false;
        break;
      }
    }
    else if (mprim_mode_ == Mode::TASKSPACE3D)
    {
      valid = false;
      if (state(0) >= m_[thread_id]->numeric_data[0] &&
          state(1) >= m_[thread_id]->numeric_data[1] &&
          state(2) >= m_[thread_id]->numeric_data[2] &&
          state(0) <= m_[thread_id]->numeric_data[3] &&
          state(1) <= m_[thread_id]->numeric_data[4] &&
          state(2) <= m_[thread_id]->numeric_data[5])
      {
        valid = true;
      }
    }

    return valid;
  }

  bool ManipulationAction::validateJointLimits(const StateVarsType &state_vars, int thread_id) const
  {
    Eigen::Map<const VecDf> state(&state_vars[0], state_vars.size());
    return validateJointLimits(state, thread_id);
  }


  double ManipulationAction::GetCostToSuccessor(const StateVarsType &current_state, const StateVarsType &successor_state, int thread_id)
  {
    Eigen::Map<const VecDf> current_state_eig(&current_state[0], current_state.size());
    Eigen::Map<const VecDf> successor_state_eig(&successor_state[0], successor_state.size());
    return getCostToSuccessor(current_state_eig, successor_state_eig, thread_id);
  }

  double ManipulationAction::getCostToSuccessor(const VecDf &current_state, const VecDf &successor_state, int thread_id)
  {
    return (successor_state-current_state).norm();
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

  TrajType ManipulationAction::warmOptimize(const TrajType& t, int thread_id) const
  {
      return (*opt_)[thread_id].warmOptimize(this, t, thread_id);
  }

  TrajType ManipulationAction::optimize(const TrajType& incoming_traj,
                                        const StateVarsType &s1,
                                        const StateVarsType &s2,
                                        int thread_id)
  {
      Eigen::Map<const VecDf> p1(&s1[0], s1.size());
      Eigen::Map<const VecDf> p2(&s2[0], s2.size());
      return (*opt_)[thread_id].optimize(this, incoming_traj, p1, p2, thread_id);
  }

    TrajType ManipulationAction::optimize(const TrajType &incoming_traj,
                                          const std::vector<StateVarsType> &ancestors,
                                          const StateVarsType &successor,
                                          int thread_id)
    {
        return (*opt_)[thread_id].optimize(this, incoming_traj, ancestors, successor, thread_id);
    }

  double ManipulationAction::getCost(const TrajType &traj, int thread_id) const
  {
    return (*opt_)[thread_id].calculateCost(traj);
  }

  MatDf ManipulationAction::linInterp(const VecDf& p1, const VecDf& p2, int N) const
  {
     MatDf traj(p1.size(), N);
     for (int i=0.0; i<N; ++i)
     {
       double j = i/static_cast<double>(N);
       traj.col(i) = p1*(1-j) + p2*j;
     }
     traj.rightCols(1) = p2;

     return traj;
    }

    void ManipulationAction::setGoal(StateVarsType & goal)
    {
        goal_.resize(m_[0]->nq);
        for (int i=0; i<goal.size(); ++i) { goal_(i) = goal[i]; }
//        goal_ = contToDisc(goal_, 0);
    }


}
