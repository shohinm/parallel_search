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
 * \file   run_manipulation.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   2/21/23
 */

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <numeric>
#include <boost/functional/hash.hpp>
#include <planners/insat/InsatPlanner.hpp>
#include <planners/insat/PinsatPlanner.hpp>
#include <planners/RrtPlanner.hpp>
#include <planners/RrtConnectPlanner.hpp>
#include <planners/EpasePlanner.hpp>
#include <planners/GepasePlanner.hpp>
#include <planners/WastarPlanner.hpp>
#include <planners/BFSPlanner.hpp>
#include "ManipulationActions.hpp"
#include <mujoco/mujoco.h>
#include "bfs3d.h"
#include <planners/insat/opt/BSplineOpt.hpp>

using namespace std;
using namespace ps;

#define TERMINATION_DIST 0.1
#define BFS_DISCRETIZATION 0.01
#define DISCRETIZATION 0.05

namespace rm
{
  vector<double> goal;
  Vec3f goal_ee_pos;

  int dof;
  VecDf discretization;

  // Mujoco
  mjModel* global_m = nullptr;
  mjData* global_d = nullptr;
  /// LoS heuristic
  std::unordered_map<size_t, double> heuristic_cache;
  /// shield heuristic weight
  VecDf shield_h_w;

  // BFS
  /// BFS model Mj Handles
  mjModel* global_bfs_m = nullptr;
  mjData* global_bfs_d = nullptr;
  /// State Map for BFS heuristic
  ps::Planner::StatePtrMapType bfs_state_map;
  shared_ptr<smpl::BFS_3D> bfs3d;

}

double roundOff(double value, unsigned char prec)
{
    double pow_10 = pow(10.0, (double)prec);
    return round(value * pow_10) / pow_10;
}

Vec3f getEEPosition(const VecDf& state)
{
  mju_copy(rm::global_d->qpos, state.data(), rm::global_m->nq);
  mj_fwdPosition(rm::global_m, rm::global_d);

  VecDf ee_pos(3);
  mju_copy(ee_pos.data(), rm::global_d->xpos + 3*(rm::global_m->nbody-1), 3);

  return ee_pos;
}

Vec3f getEEPosition(const StateVarsType& state_vars)
{
  Eigen::Map<const VecDf> state(&state_vars[0], state_vars.size());
  return getEEPosition(state);
}

Vec4f getEERotation(const VecDf& state)
{
  mju_copy(rm::global_d->qpos, state.data(), rm::global_m->nq);
  mj_fwdPosition(rm::global_m, rm::global_d);

  VecDf ee_rot(4);
  mju_copy(ee_rot.data(), rm::global_d->xquat + 4 * (rm::global_m->nbody - 1), 4);

  return ee_rot;
}

Vec4f getEERotation(const StateVarsType& state_vars)
{
  Eigen::Map<const VecDf> state(&state_vars[0], state_vars.size());
  return getEERotation(state);
}

bool isGoalState(const StateVarsType& state_vars, double dist_thresh)
{
    /// Joint-wise threshold
    for (int i=0; i < rm::dof; ++i)
    {
        if (fabs(rm::goal[i] - state_vars[i]) > dist_thresh)
        {
            return false;
        }
    }
    return true;

    /// Euclidean threshold
//    return (computeHeuristic(state_vars) < dist_thresh);
}

bool isEEGoalState(const StateVarsType& state_vars, double dist_thresh)
{
  Vec3f ee_pos = getEEPosition(state_vars);

  /// Joint-wise threshold
  for (int i=0; i < 3; ++i)
  {
    if (fabs(rm::goal_ee_pos[i] - ee_pos[i]) > dist_thresh)
    {
      return false;
    }
  }
  return true;
}


bool isBFS3DGoalState(const StateVarsType& state_vars, double dist_thresh)
{
  return false;
}

size_t StateKeyGenerator(const StateVarsType& state_vars)
{
    size_t seed = 0;
    for (int i=0; i < rm::dof; ++i)
    {
        boost::hash_combine(seed, state_vars[i]);
    }
    return seed;
}

size_t BFS3DStateKeyGenerator(const StateVarsType& state_vars)
{
  size_t seed = 0;
  for (int i=0; i < 3; ++i)
  {
    boost::hash_combine(seed, state_vars[i]);
  }
  return seed;
}

size_t EdgeKeyGenerator(const EdgePtrType& edge_ptr)
{
    int controller_id;
    auto action_ptr = edge_ptr->action_ptr_;

    controller_id = std::stoi(action_ptr->GetType());

    size_t seed = 0;
    boost::hash_combine(seed, edge_ptr->parent_state_ptr_->GetStateID());
    boost::hash_combine(seed, controller_id);

    return seed;
}

double computeHeuristicStateToState(const StateVarsType& state_vars_1, const StateVarsType& state_vars_2)
{
  double dist = 0.0;
  for (int i=0; i < rm::dof; ++i)
  {
    dist += pow(state_vars_2[i]-state_vars_1[i], 2);
  }
  return std::sqrt(dist);
}

double zeroHeuristic(const StateVarsType& state_vars)
{
  return 0.0;
}

double computeHeuristic(const StateVarsType& state_vars)
{
  return computeHeuristicStateToState(state_vars, rm::goal);
}

double computeEEHeuristic(const StateVarsType& state_vars)
{
  Eigen::Map<const VecDf> state(&state_vars[0], state_vars.size());
  Vec3f ee_pos = getEEPosition(state);

  return (rm::goal_ee_pos - ee_pos).norm();
}

double computeLoSHeuristic(const StateVarsType& state_vars)
{
    size_t state_key = StateKeyGenerator(state_vars);
    if (rm::heuristic_cache.find(state_key) != rm::heuristic_cache.end())
    {
        return rm::heuristic_cache[state_key];
    }

    double h = computeHeuristic(state_vars);
    rm::heuristic_cache[state_key] = h;
    int N = static_cast<int>(h)/9e-1;
    Eigen::Map<const VecDf> p1(&state_vars[0], state_vars.size());
    Eigen::Map<const VecDf> p2(&rm::goal[0], rm::goal.size());

    for (int i=0; i<N; ++i)
    {
        double j = i/static_cast<double>(N);
        VecDf intp_pt = p1*(1-j) + p2*j;

        mju_copy(rm::global_d->qpos, intp_pt.data(), rm::global_m->nq);
        mj_fwdPosition(rm::global_m, rm::global_d);

        if (rm::global_d->ncon > 0)
        {
            rm::heuristic_cache[state_key] = 100;
            break;
        }
    }

    return rm::heuristic_cache[state_key];
}

double computeShieldHeuristic(const StateVarsType& state_vars)
{

//    double cost = shield_h_w(0) * pow((goal[0] - state_vars[0]),2) +
//                  shield_h_w(1) * pow((state_vars[1]),2) +
//                  shield_h_w(2) * pow((goal[2] - state_vars[2]),2) +
//                  shield_h_w(3) * pow((goal[3] - state_vars[3]),2) +
//                  shield_h_w(4) * pow((goal[4] - state_vars[4]),2) +
//                  shield_h_w(5) * pow((goal[5] - state_vars[5]),2);

    double cost = rm::shield_h_w(0) * pow((rm::goal[0] - state_vars[0]), 2) +
                  rm::shield_h_w(1) * pow((state_vars[1]), 2) +
                  rm::shield_h_w(2) * pow((-M_PI / 2 - state_vars[2]), 2);
    return std::sqrt(cost);
}

// double computeBFSHeuristic(const StateVarsType& state_vars)
// {
//   size_t state_key = StateKeyGenerator(state_vars);
//   return rm::bfs_state_map[state_key]->GetFValue();
// }

void initializeBFS(int length, int width, int height, vector<vector<int>> occupied_cells)
{
    rm::bfs3d = make_shared<smpl::BFS_3D>(length, width, height);
    for (auto& c : occupied_cells)
    {
        rm::bfs3d->setWall(c[0], c[1], c[2]);
    }
}

void setupSmplBFS()
{
  Vec3f lwh;
  lwh << rm::global_bfs_m->numeric_data[3] - rm::global_bfs_m->numeric_data[0],
      rm::global_bfs_m->numeric_data[4] - rm::global_bfs_m->numeric_data[1],
      rm::global_bfs_m->numeric_data[5] - rm::global_bfs_m->numeric_data[2];

  int length = static_cast<int>(lwh(0)/BFS_DISCRETIZATION)+1;
  int width = static_cast<int>(lwh(1)/BFS_DISCRETIZATION)+1;
  int height = static_cast<int>(lwh(2)/BFS_DISCRETIZATION)+1;

  std::vector<std::vector<int>> occupied_cells;
  for (int i=0; i<length; ++i)
  {
    for (int j=0; j<width; ++j)
    {
      for (int k=0; k<height; ++k)
      {
        Vec3f xyz;
        xyz << i*BFS_DISCRETIZATION + rm::global_bfs_m->numeric_data[0],
            j*BFS_DISCRETIZATION + rm::global_bfs_m->numeric_data[1],
            k*BFS_DISCRETIZATION + rm::global_bfs_m->numeric_data[2];

        VecDf fullstate(7);
        fullstate << xyz(0), xyz(1), xyz(2), 1, 0, 0, 0;
        mju_copy(rm::global_bfs_d->qpos, fullstate.data(), rm::global_bfs_m->nq);
        mj_fwdPosition(rm::global_bfs_m, rm::global_bfs_d);

        if (rm::global_bfs_d->ncon>0)
        {
          std::vector<int> occupied_cell;
          occupied_cell.emplace_back(i);
          occupied_cell.emplace_back(j);
          occupied_cell.emplace_back(k);

          occupied_cells.push_back(occupied_cell);
        }
      }
    }
  }

  initializeBFS(length, width, height, occupied_cells);

  std::cout << "Finished setting up SMPL bfs3d environment of size " <<  length << "x" << width << "x" << height
      << " cells containing " << occupied_cells.size() << " occupied cells." << std::endl;
}

void recomputeBFS()
{
    int x = static_cast<int>((rm::goal_ee_pos(0)-rm::global_bfs_m->numeric_data[0])/BFS_DISCRETIZATION);
    int y = static_cast<int>((rm::goal_ee_pos(1)-rm::global_bfs_m->numeric_data[1])/BFS_DISCRETIZATION);
    int z = static_cast<int>((rm::goal_ee_pos(2)-rm::global_bfs_m->numeric_data[2])/BFS_DISCRETIZATION);

    rm::bfs3d->run(x, y, z);
}

double computeBFSHeuristic(const StateVarsType& state_vars)
{
    Vec3f ee_pos = getEEPosition(state_vars);

    int x = static_cast<int>((ee_pos(0)-rm::global_bfs_m->numeric_data[0])/BFS_DISCRETIZATION);
    int y = static_cast<int>((ee_pos(1)-rm::global_bfs_m->numeric_data[1])/BFS_DISCRETIZATION);
    int z = static_cast<int>((ee_pos(2)-rm::global_bfs_m->numeric_data[2])/BFS_DISCRETIZATION);
    double cost_per_cell = 1;

    if (!rm::bfs3d->inBounds(x, y, z)) {
        return DINF;
    }
    else if (rm::bfs3d->getDistance(x, y, z) == smpl::BFS_3D::WALL) {
        return DINF;
    }
    else {
        return cost_per_cell * rm::bfs3d->getDistance(x, y, z);
    }
}

void postProcess(std::vector<PlanElement>& path, double& cost, double allowed_time, const shared_ptr<Action>& act, BSplineOpt& opt)
{
    cout << "Post processing with timeout: " << allowed_time << endl;
    std::shared_ptr<InsatAction> ins_act = std::dynamic_pointer_cast<InsatAction>(act);
    opt.postProcess(path, cost, allowed_time, ins_act.get());
}

void postProcessWithControlPoints(std::vector<PlanElement>& path, double& cost, double allowed_time, const shared_ptr<Action>& act, BSplineOpt& opt)
{
    cout << "Post processing with timeout: " << allowed_time << endl;
    std::shared_ptr<InsatAction> ins_act = std::dynamic_pointer_cast<InsatAction>(act);
    opt.postProcessWithControlPoints(path, cost, allowed_time, ins_act.get());
}

void setupMujoco(mjModel **m, mjData **d, std::string modelpath)
{
  *m = nullptr;
  if (std::strlen(modelpath.c_str()) > 4 && !strcmp(modelpath.c_str() + std::strlen(modelpath.c_str()) - 4, ".mjb"))
  {
    *m = mj_loadModel(modelpath.c_str(), nullptr);
  }
  else
  {
    *m = mj_loadXML(modelpath.c_str(), nullptr, nullptr, 0);
  }
  if (!m)
  {
    mju_error("Cannot load the model");
  }
  *d = mj_makeData(*m);
}

MatDf loadMPrims(std::string mprim_file)
{
  if (!rm::global_m)
  {
    std::runtime_error("Attempting to load motion primitives before Mujoco model. ERROR!");
  }

  /// Load input prims
  MatDf mprims = loadEigenFromFile<MatDf>(mprim_file, ' ');

  /// Input prims contain only one direction. Flip the sign for adding prims in the other direction
  int num_input_prim = mprims.rows();
  mprims.conservativeResize(2*mprims.rows(), mprims.cols());
  mprims.block(num_input_prim, 0, num_input_prim, mprims.cols()) =
      -1*mprims.block(0, 0, num_input_prim, mprims.cols());

  return mprims;
}

void constructActions(vector<shared_ptr<Action>>& action_ptrs,
                      ParamsType& action_params,
                      std::string& mj_modelpath, std::string& mprimpath,
                      ManipulationAction::OptVecPtrType& opt,
                      int num_threads)
{
    /// Vectorize simulator handle
    ManipulationAction::MjModelVecType m_vec;
    ManipulationAction::MjDataVecType d_vec;
    for (int i=0; i<num_threads; ++i)
    {
      mjModel* act_m= nullptr;
      mjData * act_d= nullptr;
      setupMujoco(&act_m, &act_d, mj_modelpath);
      m_vec.push_back(act_m);
      d_vec.push_back(act_d);
    }

    /// Load mprims
    auto mprims = loadMPrims(mprimpath);
    mprims *= (M_PI/180.0); /// Input is in degrees. Convert to radians
    action_params["length"] = mprims.rows();

    for (int i=0; i<=action_params["length"]; ++i)
    {
        if (i == action_params["length"])
        {
            auto one_joint_action = std::make_shared<OneJointAtATime>(std::to_string(i), action_params,
                                                                      DISCRETIZATION, mprims,
                                                                      opt, m_vec, d_vec, num_threads, 1);
            action_ptrs.emplace_back(one_joint_action);
        }
        else
        {
            bool is_expensive = (action_params["planner_type"] == 1) ? 1 : 0;
            auto one_joint_action = std::make_shared<OneJointAtATime>(std::to_string(i), action_params,
                                                                      DISCRETIZATION, mprims,
                                                                      opt, m_vec, d_vec, num_threads, is_expensive);
            action_ptrs.emplace_back(one_joint_action);
        }
    }

    // So that the adaptive primitive is tried first
    reverse(action_ptrs.begin(), action_ptrs.end());
}


void constructBFSActions(vector<shared_ptr<Action>>& action_ptrs,
                         ParamsType& action_params,
                         std::string& mj_modelpath, std::string& mprimpath,
                         int num_threads)
{
  /// Vectorize simulator handle
  ManipulationAction::MjModelVecType m_vec;
  ManipulationAction::MjDataVecType d_vec;
  for (int i=0; i<num_threads; ++i)
  {
    mjModel* act_m= nullptr;
    mjData * act_d= nullptr;
    setupMujoco(&act_m, &act_d, mj_modelpath);
    m_vec.push_back(act_m);
    d_vec.push_back(act_d);
  }

  /// Load mprims
  auto mprims = loadMPrims(mprimpath);
  action_params["length"] = mprims.rows();

  for (int i=0; i<action_params["length"]; ++i)
  {
    auto one_joint_action = std::make_shared<TaskSpaceAction>(std::to_string(i), action_params,
                                                              BFS_DISCRETIZATION, mprims,
                                                              m_vec, d_vec, num_threads, 0);
    action_ptrs.emplace_back(one_joint_action);
  }
}


void constructPlanner(string planner_name, shared_ptr<Planner>& planner_ptr, vector<shared_ptr<Action>>& action_ptrs, ParamsType& planner_params, ParamsType& action_params, BSplineOpt& opt)
{
    if (planner_name == "epase")
       planner_ptr = std::make_shared<EpasePlanner>(planner_params);    
    else if (planner_name == "gepase")
       planner_ptr = std::make_shared<GepasePlanner>(planner_params);    
    else if (planner_name == "insat")
        planner_ptr = std::make_shared<InsatPlanner>(planner_params);
    else if (planner_name == "pinsat")
        planner_ptr = std::make_shared<PinsatPlanner>(planner_params);
    else if (planner_name == "rrt")
        planner_ptr = std::make_shared<RrtPlanner>(planner_params);
    else if (planner_name == "rrtconnect")
        planner_ptr = std::make_shared<RrtConnectPlanner>(planner_params);
    else if (planner_name == "wastar")
        planner_ptr = std::make_shared<WastarPlanner>(planner_params);
    else
        throw runtime_error("Planner type not identified!");

    /// Heuristic
//    planner_ptr->SetHeuristicGenerator(bind(computeHeuristic, placeholders::_1));
//    planner_ptr->SetHeuristicGenerator(bind(computeLoSHeuristic, placeholders::_1));
//    planner_ptr->SetHeuristicGenerator(bind(computeShieldHeuristic, placeholders::_1));
    planner_ptr->SetHeuristicGenerator(bind(computeBFSHeuristic, placeholders::_1));
//    planner_ptr->SetHeuristicGenerator(bind(computeEEHeuristic, placeholders::_1));

    planner_ptr->SetActions(action_ptrs);
    planner_ptr->SetStateMapKeyGenerator(bind(StateKeyGenerator, placeholders::_1));
    planner_ptr->SetEdgeKeyGenerator(bind(EdgeKeyGenerator, placeholders::_1));
    planner_ptr->SetStateToStateHeuristicGenerator(bind(computeHeuristicStateToState, placeholders::_1, placeholders::_2));
//    planner_ptr->SetGoalChecker(bind(isGoalState, placeholders::_1, TERMINATION_DIST));
    planner_ptr->SetGoalChecker(bind(isEEGoalState, placeholders::_1, TERMINATION_DIST));
    if ((planner_name == "epase") || (planner_name == "gepase") || (planner_name == "rrt") || (planner_name == "rrtconnect"))
    {
        planner_ptr->SetPostProcessor(bind(postProcess, placeholders::_1, placeholders::_2, placeholders::_3, action_ptrs[0], opt));
    }
    else if (!(planner_name == "insat" || planner_name == "pinsat"))
    {
        planner_ptr->SetPostProcessor(bind(postProcessWithControlPoints, placeholders::_1, placeholders::_2, placeholders::_3, action_ptrs[0], opt));        
    }
}

void setBFSHeuristic(StateVarsType& start, std::shared_ptr<Planner>& bfs_planner_ptr,
                     std::vector<shared_ptr<Action>>& bfs_action_ptrs, ParamsType& planner_params)
{
  bfs_planner_ptr->SetActions(bfs_action_ptrs);
  bfs_planner_ptr->SetStateMapKeyGenerator(bind(BFS3DStateKeyGenerator, placeholders::_1));
  bfs_planner_ptr->SetEdgeKeyGenerator(bind(EdgeKeyGenerator, placeholders::_1));
  bfs_planner_ptr->SetGoalChecker(bind(isBFS3DGoalState, placeholders::_1, TERMINATION_DIST));
  bfs_planner_ptr->SetHeuristicGenerator(bind(zeroHeuristic, placeholders::_1));

  mju_copy(rm::global_d->qpos, start.data(), rm::global_m->nq);
  mju_zero(rm::global_d->qvel, rm::global_m->nv);
  mju_zero(rm::global_d->qacc, rm::global_m->nv);
  mj_kinematics(rm::global_m, rm::global_d);
  // ee pose
  std::vector<double> ee_pos(3, 0.0);
  double xpos[rm::global_m->nbody*3];
  mju_copy(xpos, rm::global_d->xpos, rm::global_m->nbody*3);
  ee_pos[0] = xpos[3*(rm::global_m->nbody-1)];
  ee_pos[1] = xpos[3*(rm::global_m->nbody-1) + 1];
  ee_pos[2] = xpos[3*(rm::global_m->nbody-1) + 2];

  bfs_planner_ptr->SetStartState(ee_pos);
  bfs_planner_ptr->Plan();

  rm::bfs_state_map = bfs_planner_ptr->GetStateMap();
}

std::random_device rd;
std::mt19937 gen(0);  //here you could set the seed, but std::random_device already does that
std::uniform_real_distribution<float> dis(-1.0, 1.0);
VecDf genRandomVector(VecDf& low, VecDf& high, int size)
{
    VecDf range = high-low;
//    VecDf randvec = VecDf::Random(size);
    VecDf randvec = VecDf::NullaryExpr(size,1,[&](){return dis(gen);});
    randvec += VecDf::Constant(size, 1, 1.0);
    randvec /= 2.0;
    randvec = randvec.cwiseProduct(range);
    randvec += low;

    return randvec;
}

void generateStartsAndGoals(vector<vector<double>>& starts, vector<vector<double>>& goals, int num_runs, mjModel* m, mjData* d)
{
    bool valid = true;
    VecDf hi(rm::dof), lo(rm::dof);
    hi.setZero(); lo.setZero();
    for (int i=0; i<m->njnt; ++i)
    {
        lo(i) = m->jnt_range[2*i];
        hi(i) = m->jnt_range[2*i+1];
    }

    for (int i=0; i<num_runs; ++i)
    {
        VecDf st = genRandomVector(lo, hi, rm::dof);
        VecDf go = genRandomVector(lo, hi, rm::dof);
        for (int i=0; i < rm::dof; ++i)
        {
            if (i==3 || i==5) { st(i) = go(i) = 0.0;}
        }

        std::vector<double> v_st, v_go;
        v_st.resize(rm::dof);
        v_go.resize(rm::dof);
        VecDf::Map(&v_st[0], st.size()) = st;
        VecDf::Map(&v_go[0], go.size()) = go;

        starts.emplace_back(v_st);
        goals.emplace_back(v_go);
    }
}

void loadStartsAndGoalsFromFile(vector<vector<double>>& starts,
                                vector<vector<double>>& goals,
                                const string& start_path, const string& goal_path)
{
    MatDf start_mat = loadEigenFromFile<MatDf>(start_path);
    MatDf goal_mat = loadEigenFromFile<MatDf>(goal_path);

    for (int i=0; i<start_mat.rows(); ++i)
    {
//        for (int i=0; i<dof; ++i)
//        {
//            if (i==3 || i==5) { st(i) = go(i) = 0.0;}
//        }

        std::vector<double> v_st, v_go;
        v_st.resize(rm::dof);
        v_go.resize(rm::dof);
        VecDf::Map(&v_st[0], start_mat.cols()) = start_mat.row(i);
        VecDf::Map(&v_go[0], goal_mat.cols()) = goal_mat.row(i);

        starts.emplace_back(v_st);
        goals.emplace_back(v_go);
    }
}


MatDf sampleTrajectory(const drake::trajectories::BsplineTrajectory<double>& traj, double dt=1e-1)
{
    MatDf sampled_traj;
    int i=0;
    for (double t=0.0; t<=traj.end_time(); t+=dt)
    {
        sampled_traj.conservativeResize(rm::dof, sampled_traj.cols() + 1);
        sampled_traj.col(i) = traj.value(t);
        ++i;
    }
    return sampled_traj;
}


int main(int argc, char* argv[])
{
    int num_threads;

    if (!strcmp(argv[1], "insat") || !strcmp(argv[1], "wastar"))
    {
      if (argc != 2) throw runtime_error("Format: run_robot_nav_2d insat");
      num_threads = 1;
    }
    else if (!strcmp(argv[1], "pinsat") || !strcmp(argv[1], "rrt") || !strcmp(argv[1], "rrtconnect") || !strcmp(argv[1], "epase") || !strcmp(argv[1], "gepase"))
    {
      if (argc != 3) throw runtime_error("Format: run_robot_nav_2d pinsat [num_threads]");
      num_threads = atoi(argv[2]);
    }
    else
    {
      throw runtime_error("Planner " + string(argv[1]) + " not identified");
    }

    string planner_name = argv[1];


    /// Load MuJoCo model
    std::string modelpath = "../third_party/mujoco-2.3.2/model/abb/irb_1600/irb1600_6_12_shield.xml";
    mjModel *m = nullptr;
    mjData *d = nullptr;

    setupMujoco(&m,&d,modelpath);
    setupMujoco(&rm::global_m, &rm::global_d, modelpath);
    rm::dof = m->nq;
    rm::shield_h_w.resize(rm::dof);
    rm::shield_h_w << 0, 10, 7, 0.1, 1, 0.1;



    // Experiment parameters
    int num_runs;
    vector<int> scale_vec = {5, 5, 5, 10, 5};
    bool visualize_plan = true;
    bool load_starts_goals_from_file = true;

    // Define planner parameters
    ParamsType planner_params;
    planner_params["num_threads"] = num_threads;
    planner_params["heuristic_weight"] = 10;
    planner_params["timeout"] = 20;
    planner_params["adaptive_opt"] = 0;
    planner_params["smart_opt"] = 1;
    planner_params["min_exec_duration"] = 0.5;
    planner_params["max_exec_duration"] = 0.9;
    planner_params["num_ctrl_points"] = 7;
    planner_params["min_ctrl_points"] = 4;
    planner_params["max_ctrl_points"] = 7;
    planner_params["spline_order"] = 4;
    planner_params["sampling_dt"] = 5e-3;

    ofstream log_file;

    if ((planner_params["smart_opt"] == 1) && ((planner_name == "insat") || (planner_name == "pinsat")))
    {
        log_file.open("../logs/" + planner_name + "_smart_" + to_string(num_threads) + ".txt");
    }
    else if ((planner_params["adaptive_opt"] == 1) && ((planner_name == "insat") || (planner_name == "pinsat")))
    {
       log_file.open("../logs/" + planner_name + "_adaptive_" + to_string(num_threads) + ".txt"); 
    }
    else
    {
        log_file.open("../logs/" + planner_name + "_" + to_string(num_threads) + ".txt");    
    }

    if ((planner_name == "rrt") || (planner_name == "rrtconnect"))
    {
        planner_params["eps"] = 1.0;
        planner_params["goal_bias_probability"] = 0.05;
        planner_params["termination_distance"] = TERMINATION_DIST;  
    }

    // Generate random starts and goals
    std::vector<vector<double>> starts, goals;
    if (load_starts_goals_from_file)
    {
        std::string starts_path = "../examples/manipulation/resources/shield/starts.txt";
        std::string goals_path = "../examples/manipulation/resources/shield/goals.txt";
        loadStartsAndGoalsFromFile(starts, goals, starts_path, goals_path);
    }
    else
    {
        generateStartsAndGoals(starts, goals, num_runs, m, d);
    }

    // Robot Params
    IRB1600 robot_params;
    // Insat Params
    InsatParams insat_params(rm::dof, 2 * rm::dof, rm::dof);
    // spline params
    BSplineOpt::BSplineOptParams spline_params(rm::dof,
                                               planner_params["num_ctrl_points"],
                                               planner_params["spline_order"],
                                               planner_params["min_exec_duration"],
                                               planner_params["max_exec_duration"],
                                               BSplineOpt::BSplineOptParams::ConstraintMode::CONTROLPT);
    spline_params.setAdaptiveParams(planner_params["min_ctrl_points"], planner_params["max_ctrl_points"]);
    // discretization
    rm::discretization.resize(rm::dof);
    rm::discretization.setOnes();
    rm::discretization *= DISCRETIZATION;
    // discretization = (robot_params.max_q_ - robot_params.min_q_)/50.0;

    vector<double> all_maps_time_vec, all_maps_cost_vec;
    vector<int> all_maps_num_edges_vec;
    unordered_map<string, vector<double>> all_action_eval_times;
    vector<double> all_execution_time;

    /// save logs
    MatDf start_log, goal_log, traj_log;
    std::string traj_path ="../logs/" + planner_name +"_abb_traj.txt";
    std::string starts_path ="../logs/" + planner_name + "_abb_starts.txt";
    std::string goals_path ="../logs/" + planner_name +"_abb_goals.txt";

    // create opt
    auto opt = BSplineOpt(insat_params, robot_params, spline_params, planner_params);
    opt.SetGoalChecker(bind(isGoalState, placeholders::_1, TERMINATION_DIST));
    auto opt_vec_ptr = std::make_shared<ManipulationAction::OptVecType>(num_threads, opt);

    // Construct actions
    ParamsType action_params;
    action_params["planner_type"] = planner_name=="insat" || planner_name=="pinsat"? 1: -1;
    std::string mprimpath = "../examples/manipulation/resources/shield/irb1600_6_12.mprim";
    vector<shared_ptr<Action>> action_ptrs;
    constructActions(action_ptrs, action_params,
                     modelpath,
                     mprimpath,
                     opt_vec_ptr, num_threads);

    // Construct BFS actions
    std::string bfsmodelpath = "../third_party/mujoco-2.3.2/model/abb/irb_1600/shield_bfs_heuristic.xml";
    setupMujoco(&rm::global_bfs_m, &rm::global_bfs_d, bfsmodelpath);
    std::string bfsmprimpath = "../examples/manipulation/resources/shield/bfs3d.mprim";
    vector<shared_ptr<Action>> bfs_action_ptrs;
    // constructBFSActions(bfs_action_ptrs, action_params,
    //                    bfsmodelpath, bfsmprimpath, num_threads);
    
    /// SMPL bfs3d
    setupSmplBFS();

    std::vector<std::shared_ptr<ManipulationAction>> manip_action_ptrs;
    for (auto& a : action_ptrs)
    {
        std::shared_ptr<ManipulationAction> manip_action_ptr = std::dynamic_pointer_cast<ManipulationAction>(a);
        manip_action_ptrs.emplace_back(manip_action_ptr);
    }


    int num_success = 0;
    vector<vector<PlanElement>> plan_vec;

    int run_offset = 0;
    num_runs = starts.size();
    num_runs = 500;
    for (int run = run_offset; run < run_offset+num_runs; ++run)
    {
        // Set goal conditions
        rm::goal = goals[run];
        rm::goal_ee_pos = getEEPosition(rm::goal);
        /// Call SMPL bfs3d after updating ee goal
        recomputeBFS();
        
        auto start = starts[run];

        for (auto& op : *opt_vec_ptr)
        {
            op.updateStartAndGoal(start, rm::goal);
        }

        for (auto& m : manip_action_ptrs)
        {
            m->setGoal(goals[run]);
        }

        /// Clear heuristic cache
        rm::heuristic_cache.clear();

        /// Set BFS heuristic
        std::shared_ptr<Planner> bfs_planner_ptr = std::make_shared<BFSPlanner>(planner_params);
//        setBFSHeuristic(goals[run], bfs_planner_ptr, bfs_action_ptrs, planner_params);

        // Construct planner
        shared_ptr<Planner> planner_ptr;
        constructPlanner(planner_name, planner_ptr, action_ptrs, planner_params, action_params, opt_vec_ptr->at(0));

        // Run experiments
        vector<double> time_vec, cost_vec;
        vector<int> num_edges_vec, threads_used_vec;
        vector<int> jobs_per_thread(planner_params["num_threads"], 0);
        unordered_map<string, vector<double>> action_eval_times;

        cout << " | Planner: " << planner_name
             << " | Heuristic weight: " << planner_params["heuristic_weight"]
             << " | Number of threads: " << planner_params["num_threads"]
             << " | Number of runs: " << num_runs
             << endl;
        cout <<  "---------------------------------------------------" << endl;

        cout << "Experiment: " << run << endl;
        // print start and goal
        std::cout << "start: ";
        for (double i: starts[run])
            std::cout << i << ' ';
        std::cout << std::endl;
        std::cout << "goal: ";
        for (double i: goals[run])
            std::cout << i << ' ';
        std::cout << std::endl;


        // Set start state
        planner_ptr->SetStartState(start);
        if ((planner_name == "rrt") || (planner_name == "rrtconnect"))
        {
            planner_ptr->SetGoalState(rm::goal);
        }


        double t=0, cost=0;
        int num_edges=0;

        bool plan_found = planner_ptr->Plan();
        auto planner_stats = planner_ptr->GetStats();
    
        cout << " | Time (s): " << planner_stats.total_time_
             << " | Cost: " << planner_stats.path_cost_
             << " | Length: " << planner_stats.path_length_
             << " | State expansions: " << planner_stats.num_state_expansions_
             << " | State expansions rate: " << planner_stats.num_state_expansions_/planner_stats.total_time_
             << " | Lock time: " <<  planner_stats.lock_time_
             << " | Expand time: " << planner_stats.cumulative_expansions_time_
             << " | Threads: " << planner_stats.num_threads_spawned_ << "/" << planner_params["num_threads"] << endl;

        for (auto& [action, times] : planner_stats.action_eval_times_)
        {
            auto total_time = accumulate(times.begin(), times.end(), 0.0);
            cout << action << " mean time: " << total_time/times.size()  
            << " | total: " << total_time 
            << " | num: " << times.size()
            << endl;
        }
        // cout << endl << "------------- Jobs per thread -------------" << endl;
        // for (int tidx = 0; tidx < planner_params["num_threads"]; ++tidx)
        // {
        //     cout << "thread: " << tidx << " jobs: " << planner_stats.num_jobs_per_thread_[tidx] << endl;
        // }
        // cout << "************************" << endl;
        // getchar();

        double exec_duration = -1;
        if (plan_found)
        {

            time_vec.emplace_back(planner_stats.total_time_);
            all_maps_time_vec.emplace_back(planner_stats.total_time_);
            cost_vec.emplace_back(planner_stats.path_cost_);
            all_maps_cost_vec.emplace_back(planner_stats.path_cost_);
            num_edges_vec.emplace_back(planner_stats.num_evaluated_edges_);
            all_maps_num_edges_vec.emplace_back(planner_stats.num_evaluated_edges_);

            for (auto& [action, times] : planner_stats.action_eval_times_)
            {
                action_eval_times[action].insert(action_eval_times[action].end(), times.begin(), times.end());
                all_action_eval_times[action].insert(all_action_eval_times[action].end(), times.begin(), times.end());
            }

            threads_used_vec.emplace_back(planner_stats.num_threads_spawned_);
            for (int tidx = 0; tidx < planner_params["num_threads"]; ++tidx)
                jobs_per_thread[tidx] += planner_stats.num_jobs_per_thread_[tidx];

            num_success++;
    
            cout << endl << "************************" << endl;
            cout << "Number of runs: " << num_runs << endl;
            cout << "Mean time: " << accumulate(time_vec.begin(), time_vec.end(), 0.0)/time_vec.size() << endl;
            cout << "Mean cost: " << accumulate(cost_vec.begin(), cost_vec.end(), 0.0)/cost_vec.size() << endl;
            cout << "Mean threads used: " << accumulate(threads_used_vec.begin(), threads_used_vec.end(), 0.0)/threads_used_vec.size() << "/" << planner_params["num_threads"] << endl;
            cout << "Mean evaluated edges: " << roundOff(accumulate(num_edges_vec.begin(), num_edges_vec.end(), 0.0)/double(num_edges_vec.size()), 2) << endl;
            // cout << endl << "------------- Mean jobs per thread -------------" << endl;
            // for (int tidx = 0; tidx < planner_params["num_threads"]; ++tidx)
            // {
            //     cout << "thread: " << tidx << " jobs: " << jobs_per_thread[tidx]/num_success << endl;
            // }
            // cout << "************************" << endl;

            // cout << endl << "------------- Mean action eval times -------------" << endl;
            // for (auto [action, times] : action_eval_times)
            // {
            //     cout << action << ": " << accumulate(times.begin(), times.end(), 0.0)/times.size() << endl;
            // }
            // cout << "************************" << endl;

            /// track logs
            start_log.conservativeResize(start_log.rows()+1, insat_params.lowD_dims_);
            goal_log.conservativeResize(goal_log.rows()+1, insat_params.lowD_dims_);
            for (int i=0; i < rm::dof; ++i)
            {
                Eigen::Map<const VecDf> svec(&starts[run][0], rm::dof);
                Eigen::Map<const VecDf> gvec(&goals[run][0], rm::dof);
                start_log.bottomRows(1) = svec.transpose();
                goal_log.bottomRows(1) = gvec.transpose();
            }

            if ((planner_name == "insat") || (planner_name == "pinsat"))
            {
                std::shared_ptr<InsatPlanner> insat_planner = std::dynamic_pointer_cast<InsatPlanner>(planner_ptr);
                auto soln_traj = insat_planner->getSolutionTraj();
                auto samp_traj = sampleTrajectory(soln_traj.traj_, planner_params["sampling_dt"]);
                traj_log.conservativeResize(insat_params.lowD_dims_, traj_log.cols()+samp_traj.cols());
                traj_log.rightCols(samp_traj.cols()) = samp_traj;
                traj_log.conservativeResize(insat_params.lowD_dims_, traj_log.cols()+1);
                traj_log.rightCols(1) = -1*VecDf::Ones(insat_params.lowD_dims_);
                all_execution_time.push_back(soln_traj.traj_.end_time());
                cout << "Execution time: " << soln_traj.traj_.end_time() << endl;
                cout << "Traj converged in: " << soln_traj.story_ << endl;
                exec_duration = soln_traj.traj_.end_time();

                auto plan = planner_ptr->GetPlan();
                plan_vec.emplace_back(plan);
            }
            else
            {
                auto plan = planner_ptr->GetPlan();
                plan_vec.emplace_back(plan);
                exec_duration = plan_vec.size()*planner_params["sampling_dt"];
            }


            if (visualize_plan)
            {
            }
    
        }
        else
        {
            cout << " | Plan not found!" << endl;
        }


        log_file << run << " " 
        << planner_stats.total_time_ << " " 
        << planner_stats.path_cost_<< " " 
        << planner_stats.path_length_<< " " 
        << planner_stats.num_state_expansions_<< " " 
        << planner_stats.num_evaluated_edges_<< " " 
        << planner_stats.num_threads_spawned_<< " " 
        << exec_duration<< " "
        << endl;
    }

    StateVarsType dummy_wp(6, -1);
    if ((planner_name == "insat") || (planner_name == "pinsat"))
    {
        traj_log.transposeInPlace();
        writeEigenToFile(traj_path, traj_log);

        ofstream traj_fout("../logs/" + planner_name + "_abb_path.txt");

        for (auto& p : plan_vec)
        {
          for (auto& wp : p)
          {
            for (auto& j : wp.state_)
            {
              traj_fout << j << " ";
            }
            traj_fout << endl;
          }

          for (auto& j : dummy_wp)
          {
            traj_fout << j << " ";
          }
          traj_fout << endl;
        }

        traj_fout.close();
    }
    else
    {
        ofstream traj_fout("../logs/" + planner_name + "_abb_traj.txt");

        for (auto& p : plan_vec)
        {
            for (auto& wp : p)
            {
                for (auto& j : wp.state_)
                {
                    traj_fout << j << " ";
                }
                traj_fout << endl;
            }

            for (auto& j : dummy_wp)
            {
                traj_fout << j << " ";
            }
            traj_fout << endl;
        }

        traj_fout.close();
    }

    writeEigenToFile(starts_path, start_log);
    writeEigenToFile(goals_path, goal_log);


    cout << endl << "************ Global Stats ************" << endl;
    cout << "Success rate: " << double(num_success)/num_runs << endl;
    cout << "Mean time: " << accumulate(all_maps_time_vec.begin(), all_maps_time_vec.end(), 0.0)/all_maps_time_vec.size() << endl;
    cout << "Mean cost: " << accumulate(all_maps_cost_vec.begin(), all_maps_cost_vec.end(), 0.0)/all_maps_cost_vec.size() << endl;
    cout << "Mean evaluated edges: " << roundOff(accumulate(all_maps_num_edges_vec.begin(), all_maps_num_edges_vec.end(), 0.0)/double(all_maps_num_edges_vec.size()), 2) << endl;
    cout << "Mean trajectory duration: " << roundOff(reduce(all_execution_time.begin(), all_execution_time.end())/double(all_execution_time.size()), 2) << endl;
    cout << endl << "************************" << endl;

    cout << endl << "------------- Mean action eval times -------------" << endl;
    for (auto [action, times] : all_action_eval_times)
    {
        cout << action << ": " << accumulate(times.begin(), times.end(), 0.0)/times.size() << endl;
    }
    cout << "************************" << endl;

}

