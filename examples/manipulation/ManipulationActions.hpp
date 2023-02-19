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

#include <common/Types.hpp>
#include <common/insat/InsatAction.hpp>
#include "planners/insat/opt/DummyOpt.hpp"

// MuJoCo
#include <mujoco/mujoco.h>

namespace ps
{

  class ManipulationAction : public InsatAction
  {

  public:

    typedef std::shared_ptr<ManipulationAction> Ptr;
    typedef DummyOpt OptType;
    typedef std::vector<OptType> OptVecType;
    typedef std::shared_ptr<OptVecType> OptVecPtrType;

    ManipulationAction(const std::string& type,
                     ParamsType params,
                     std::string& mj_modelpath,
                     VecDf ang_discretization,
                     OptVecPtrType& opt,
                     bool is_expensive = true);

    virtual bool CheckPreconditions(StateVarsType state);
    ActionSuccessor GetSuccessor(StateVarsType state_vars, int thread_id);
    ActionSuccessor GetSuccessorLazy(StateVarsType state_vars, int thread_id);
    ActionSuccessor Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id=0);

    /// Snap to Grid
    VecDf contToDisc(const VecDf & cont_state);

    /// MuJoCo
    std::vector<VecDf> GetSuccessor(const VecDf& state);
    bool isCollisionFree(const VecDf& state) const;
    bool isCollisionFree(StateVarsType& state_vars) const;
    bool isCollisionFree(const VecDf& curr,
                         const VecDf& succ,
                         VecDf& free_state) const; /// Edge collision check and return the last free state
    bool validateJointLimits(const VecDf& state);
    double getCostToSuccessor(const VecDf& current_state, const VecDf& successor_state);

    /// INSAT
    void setOpt(OptVecPtrType& opt);
    bool isFeasible(TrajType& traj) const;
    TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const;
    TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const;
    double getCost(const TrajType& traj, int thread_id=0) const;

  protected:
    LockType lock_;

    /// Discretization stuff
    MatDf mprims_;
    VecDf discretization_;
    std::unordered_map<int, VecDf> discrete_angles_;

    /// Optimizer stuff
    OptVecPtrType opt_;

    /// MuJoCo
    mjModel* m_;
    mjData* d_=nullptr;

  };

  class OneJointAtATime : public ManipulationAction
  {

  public:
    OneJointAtATime(const std::string& type,
                    ParamsType params,
                    std::string& mj_modelpath,
                    VecDf ang_discretization,
                    OptVecPtrType opt,
                    bool is_expensive = true):
            ManipulationAction(type,
                               params,
                               mj_modelpath,
                               ang_discretization,
                               opt,
                               is_expensive)
    {
      mprims_.resize(2*m_->nq, m_->nq);
      mprims_.setZero();
      for (int i=0; i<m_->nq; ++i)
      {
        for (int j=0; j<m_->nq; ++j)
        {
          if (i==j)
          {
            mprims_(i, j) = 1;
          }
        }
      }
      mprims_.block(m_->nq,0,m_->nq,m_->nq) = -1*mprims_.block(0,0,m_->nq,m_->nq);
    };

    void setOpt(OptVecPtrType& opt) {ManipulationAction::setOpt(opt);};
    bool isFeasible(TrajType& traj) const {return ManipulationAction::isFeasible(traj);};
    TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
    {return ManipulationAction::optimize(s1, s2, thread_id);}
    TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
    {return ManipulationAction::warmOptimize(t1, t2, thread_id);}
    double getCost(const TrajType& traj, int thread_id=0) const
    {return ManipulationAction::getCost(traj, thread_id);}
  };


}


#endif //MANIPULATIONACTIONS_HPP
