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
 * \file   ManipulationActions..hpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   2/19/23
 */

#ifndef MANIPULATIONACTIONS_HPP
#define MANIPULATIONACTIONS_HPP

#include <random>
#include <common/Types.hpp>
#include <common/insat/InsatAction.hpp>
#include <common/robots/Abb.hpp>
#include "planners/insat/opt/BSplineOpt.hpp"

// MuJoCo
#include <mujoco/mujoco.h>

namespace ps
{

  class ManipulationAction : public InsatAction
  {

  public:

    typedef std::shared_ptr<ManipulationAction> Ptr;
    typedef BSplineOpt OptType;
    typedef std::vector<OptType> OptVecType;
    typedef std::shared_ptr<OptVecType> OptVecPtrType;
    typedef std::vector<mjModel*> MjModelVecType; /// MuJoCo
    typedef std::vector<mjData*> MjDataVecType; /// MuJoCo

    enum Mode
    {
      CSPACE = 0,
      TASKSPACE,
      TASKSPACE3D
    };

    ManipulationAction(const std::string& type,
                     ParamsType params,
                     Mode mode,
                     double discretization,
                     MatDf& mprims,
                     OptVecPtrType& opt,
                     MjModelVecType& m_vec, MjDataVecType& d_vec,
                     int num_threads=1,
                     bool is_expensive = true);

    virtual bool CheckPreconditions(const StateVarsType& state);
    ActionSuccessor GetSuccessor(const StateVarsType& state_vars, int thread_id);
    ActionSuccessor GetSuccessorLazy(const StateVarsType& state_vars, int thread_id);
    ActionSuccessor Evaluate(const StateVarsType& parent_state_vars, const StateVarsType& child_state_vars, int thread_id);

    /// Snap to Grid
    VecDf contToDisc(const VecDf & cont_state, int thread_id);

    /// MuJoCo
    VecDf GetSuccessor(const VecDf& state, int thread_id);
    bool IsFeasible(const StateVarsType& state_vars, int thread_id);
    bool isCollisionFree(const VecDf& state, int thread_id) const;
    bool isCollisionFree(const StateVarsType& state_vars, int thread_id) const;
    bool isCollisionFree(const VecDf& curr,
                         const VecDf& succ,
                         VecDf& free_state, int thread_id) const; /// Edge collision check and return the last free state
    StateVarsType SampleFeasibleState(int thread_id);
    bool validateJointLimits(const VecDf& state, int thread_id) const;
    std::vector<std::pair<double,double>> getJointLimits(int thread_id) const;
    bool validateJointLimits(const StateVarsType& state_vars, int thread_id) const;
    double GetCostToSuccessor(const StateVarsType& current_state, const StateVarsType& successor_state, int thread_id);
    double getCostToSuccessor(const VecDf& current_state, const VecDf& successor_state, int thread_id);

    /// INSAT
    void setOpt(OptVecPtrType& opt);
    bool isFeasible(MatDf& traj, int thread_id) const;
    TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id) const;
    TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id) const;
    TrajType warmOptimize(const TrajType& t, int thread_id) const;
    TrajType optimize(const TrajType& incoming_traj,
                      const StateVarsType &s1,
                      const StateVarsType &s2,
                      int thread_id);
    TrajType optimize(const TrajType& incoming_traj,
                      const std::vector<StateVarsType> &ancestors,
                      const StateVarsType& successor,
                      int thread_id);
    double getCost(const TrajType& traj, int thread_id) const;
    MatDf linInterp(const VecDf& p1, const VecDf& p2, int N) const;

    /// goal
    void setGoal(StateVarsType& goal);

  protected:
    LockType lock_;

    VecDf goal_;

    /// Discretization stuff
    MatDf mprims_;
    double discretization_;
    std::unordered_map<int, VecDf> discrete_angles_;
    Mode mprim_mode_;

    /// Optimizer stuff
    OptVecPtrType opt_;

    /// Robot stuff
    IRB1600 robot_params_;

    /// MuJoCo
    MjModelVecType m_;
    MjDataVecType d_;

    std::vector<std::pair<double, double>> joint_limits_;
    std::mt19937 gen_;
  };

  class OneJointAtATime : public ManipulationAction
  {

  public:
    OneJointAtATime(const std::string& type,
                    ParamsType params,
                    Mode mode,
                    double discretization, MatDf& mprims,
                    OptVecPtrType& opt,
                    MjModelVecType& m_vec, MjDataVecType& d_vec,
                    int num_threads=1,
                    bool is_expensive = true):
            ManipulationAction(type,
                               params,
                               mode,
                               discretization,
                               mprims,
                               opt, m_vec, d_vec,
                               num_threads,
                               is_expensive)
    {
    };
  };

  class TaskSpaceAction : public ManipulationAction
  {

  public:
    TaskSpaceAction(const std::string& type,
                    ParamsType params,
                    Mode mode,
                    double discretization, MatDf& mprims,
                    OptVecPtrType& opt,
                    MjModelVecType& m_vec, MjDataVecType& d_vec,
                    int num_threads=1,
                    bool is_expensive = true):
        ManipulationAction(type,
                           params,
                           mode,
                           discretization,
                           mprims,
                           opt, m_vec, d_vec,
                           num_threads,
                           is_expensive)
    {
      if (!(mprims.cols()==2 || mprims.cols()==3))
      {
        std::runtime_error("Task space actions primitives should be in 2D or 3D...");
      }
    };
  };

}


#endif //MANIPULATIONACTIONS_HPP
