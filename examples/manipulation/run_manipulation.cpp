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
#include "ManipulationActions.hpp"
#include <mujoco/mujoco.h>
#include <planners/insat/opt/BSplineOpt.hpp>

using namespace std;
using namespace ps;

#define TERMINATION_DIST 0.1
#define DISCRETIZATION 0.05

vector<double> goal;
int dof;
VecDf discretization;

// Mujoco and LoS heuristic
mjModel* global_m = nullptr;
mjData* global_d = nullptr;
std::unordered_map<size_t, double> heuristic_cache;

// shield heuristic weight
VecDf shield_h_w;

double roundOff(double value, unsigned char prec)
{
    double pow_10 = pow(10.0, (double)prec);
    return round(value * pow_10) / pow_10;
}

double computeHeuristicStateToState(const StateVarsType& state_vars_1, const StateVarsType& state_vars_2)
{
    double dist = 0.0;
    for (int i=0; i<dof-1; ++i)
    {
        dist += pow(state_vars_2[i]-state_vars_1[i], 2);
    }
    return std::sqrt(dist);

//    VecDf ds(dof);
//    for (int i=0; i<dof; ++i)
//    {
//        ds(i) = state_vars_2[2] - state_vars_1[i];
//        ds(i) /= discretization(i);
//    }
//    return ds.norm();
}

double computeHeuristic(const StateVarsType& state_vars)
{
    return computeHeuristicStateToState(state_vars, goal);
}

bool isGoalState(const StateVarsType& state_vars, double dist_thresh)
{
    for (int i=0; i<dof-2; ++i)
    {
        if (fabs(goal[i]-state_vars[i]) > dist_thresh)
        {
            return false;
        }
    }
    return true;

//    double dist=0;
//    for (int i=0; i<state_vars.size()-2; ++i)
//    {
//        dist += std::sqrt((goal[i]-state_vars[i])*(goal[i]-state_vars[i]));
//    }
//    return (dist < dist_thresh);

//    return (computeHeuristic(state_vars) < dist_thresh);
}

size_t StateKeyGenerator(const StateVarsType& state_vars)
{
    size_t seed = 0;
    for (int i=0; i<dof; ++i)
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

    if (controller_id > 2*dof)
    {
        throw runtime_error("Controller type not recognized in getEdgeKey!");
    }

    size_t seed = 0;
    boost::hash_combine(seed, edge_ptr->parent_state_ptr_->GetStateID());
    boost::hash_combine(seed, controller_id);

    return seed;
}

double computeLoSHeuristic(const StateVarsType& state_vars)
{
    size_t state_key = StateKeyGenerator(state_vars);
    if (heuristic_cache.find(state_key) != heuristic_cache.end())
    {
        return heuristic_cache[state_key];
    }

    double h = computeHeuristic(state_vars);
    heuristic_cache[state_key] = h;
    int N = static_cast<int>(h)/9e-1;
    Eigen::Map<const VecDf> p1(&state_vars[0], state_vars.size());
    Eigen::Map<const VecDf> p2(&goal[0], goal.size());

    for (int i=0; i<N; ++i)
    {
        double j = i/static_cast<double>(N);
        VecDf intp_pt = p1*(1-j) + p2*j;

        mju_copy(global_d->qpos, intp_pt.data(), global_m->nq);
        mj_fwdPosition(global_m, global_d);

        if (global_d->ncon>0)
        {
            heuristic_cache[state_key] = 100;
            break;
        }
    }

    return heuristic_cache[state_key];
}

double computeShieldHeuristic(const StateVarsType& state_vars)
{

//    double cost = shield_h_w(0) * pow((goal[0] - state_vars[0]),2) +
//                  shield_h_w(1) * pow((state_vars[1]),2) +
//                  shield_h_w(2) * pow((goal[2] - state_vars[2]),2) +
//                  shield_h_w(3) * pow((goal[3] - state_vars[3]),2) +
//                  shield_h_w(4) * pow((goal[4] - state_vars[4]),2) +
//                  shield_h_w(5) * pow((goal[5] - state_vars[5]),2);

    double cost = shield_h_w(0) * pow((goal[0] - state_vars[0]),2) +
                  shield_h_w(1) * pow((state_vars[1]),2) +
                  shield_h_w(2) * pow((-M_PI/2 - state_vars[2]),2);
    return std::sqrt(cost);
}

void postProcess(std::vector<PlanElement>& path, double& cost, double allowed_time, const shared_ptr<Action>& act, BSplineOpt& opt)
{
    cout << "Post processing with timeout: " << allowed_time << endl;
    std::shared_ptr<InsatAction> ins_act = std::dynamic_pointer_cast<InsatAction>(act);
//    opt.postProcess(path, cost, allowed_time, ins_act.get());
    opt.postProcessWithControlPoints(path, cost, allowed_time, ins_act.get());
}

void constructActions(vector<shared_ptr<Action>>& action_ptrs,
                      ParamsType& action_params, std::string& mj_modelpath,
                      VecDf& ang_discretization,
                      ManipulationAction::OptVecPtrType& opt,
                      ManipulationAction::MjModelVecType m_vec,
                      ManipulationAction::MjDataVecType d_vec,
                      int num_threads)
{
    for (int i=0; i<=2*dof; ++i)
    {
        auto one_joint_action = std::make_shared<OneJointAtATime>(std::to_string(i), action_params, mj_modelpath, ang_discretization, opt, m_vec, d_vec, num_threads);
        action_ptrs.emplace_back(one_joint_action);
    }
}

void constructPlanner(string planner_name, shared_ptr<Planner>& planner_ptr, vector<shared_ptr<Action>>& action_ptrs, ParamsType& planner_params, ParamsType& action_params, BSplineOpt& opt)
{
    if (planner_name == "epase")
	planner_ptr = std::make_shared<EpasePlanner>(planner_params);	
    else if (planner_name == "insat")
        planner_ptr = std::make_shared<InsatPlanner>(planner_params);
    else if (planner_name == "pinsat")
        planner_ptr = std::make_shared<PinsatPlanner>(planner_params);
    else if (planner_name == "rrt")
        planner_ptr = std::make_shared<RrtPlanner>(planner_params);
    else if (planner_name == "rrtconnect")
        planner_ptr = std::make_shared<RrtConnectPlanner>(planner_params);
    else
        throw runtime_error("Planner type not identified!");

    planner_ptr->SetActions(action_ptrs);
    planner_ptr->SetStateMapKeyGenerator(bind(StateKeyGenerator, placeholders::_1));
    planner_ptr->SetEdgeKeyGenerator(bind(EdgeKeyGenerator, placeholders::_1));
//    planner_ptr->SetHeuristicGenerator(bind(computeHeuristic, placeholders::_1));
    planner_ptr->SetHeuristicGenerator(bind(computeLoSHeuristic, placeholders::_1));
//    planner_ptr->SetHeuristicGenerator(bind(computeShieldHeuristic, placeholders::_1));
    planner_ptr->SetStateToStateHeuristicGenerator(bind(computeHeuristicStateToState, placeholders::_1, placeholders::_2));
    planner_ptr->SetGoalChecker(bind(isGoalState, placeholders::_1, TERMINATION_DIST));
    planner_ptr->SetPostProcessor(bind(postProcess, placeholders::_1, placeholders::_2, placeholders::_3, action_ptrs[0], opt));
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

template<typename M>
M load_csv (const std::string & path, char delim=' ') {
    using namespace Eigen;
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, delim)) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

//https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
void writeToCSVfile(std::string& fname, const MatDf& matrix)
{
    std::ofstream file(fname);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}

void generateStartsAndGoals(vector<vector<double>>& starts, vector<vector<double>>& goals, int num_runs, mjModel* m, mjData* d)
{
    bool valid = true;
    VecDf hi(dof), lo(dof);
    hi.setZero(); lo.setZero();
    for (int i=0; i<m->njnt; ++i)
    {
        lo(i) = m->jnt_range[2*i];
        hi(i) = m->jnt_range[2*i+1];
    }

    for (int i=0; i<num_runs; ++i)
    {
        VecDf st = genRandomVector(lo, hi, dof);
        VecDf go = genRandomVector(lo, hi, dof);
        for (int i=0; i<dof; ++i)
        {
            if (i==3 || i==5) { st(i) = go(i) = 0.0;}
        }

        std::vector<double> v_st, v_go;
        v_st.resize(dof);
        v_go.resize(dof);
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
    MatDf start_mat = load_csv<MatDf>(start_path);
    MatDf goal_mat = load_csv<MatDf>(goal_path);

    for (int i=0; i<start_mat.rows(); ++i)
    {
//        for (int i=0; i<dof; ++i)
//        {
//            if (i==3 || i==5) { st(i) = go(i) = 0.0;}
//        }

        std::vector<double> v_st, v_go;
        v_st.resize(dof);
        v_go.resize(dof);
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
        sampled_traj.conservativeResize(dof, sampled_traj.cols()+1);
        sampled_traj.col(i) = traj.value(t);
        ++i;
    }
    return sampled_traj;
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

int main(int argc, char* argv[])
{
    int num_threads;

    if (!strcmp(argv[1], "insat"))
    {
      if (argc != 2) throw runtime_error("Format: run_robot_nav_2d insat");
      num_threads = 1;
    }
    else if (!strcmp(argv[1], "pinsat") || !strcmp(argv[1], "rrt") || !strcmp(argv[1], "rrtconnect") || !strcmp(argv[1], "epase"))
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
    setupMujoco(&global_m, &global_d, modelpath);
    dof = m->nq;
    shield_h_w.resize(dof);
    shield_h_w << 0, 10, 7, 0.1, 1, 0.1;

    ManipulationAction::MjModelVecType m_vec;
    ManipulationAction::MjDataVecType d_vec;

    for (int i=0; i<num_threads; ++i)
    {
        mjModel* act_m= nullptr;
        mjData * act_d= nullptr;
        setupMujoco(&act_m, &act_d, modelpath);
        m_vec.push_back(act_m);
        d_vec.push_back(act_d);
    }


    // Experiment parameters
    int num_runs;
    vector<int> scale_vec = {5, 5, 5, 10, 5};
    bool visualize_plan = true;
    bool load_starts_goals_from_file = true;

    // Define planner parameters
    ParamsType planner_params;
    planner_params["num_threads"] = num_threads;
    planner_params["heuristic_weight"] = 10;
    planner_params["timeout"] = 15;
    planner_params["adaptive_opt"] = 0;
    planner_params["smart_opt"] = 1;
    planner_params["min_exec_duration"] = 0.2;
    planner_params["max_exec_duration"] = 1.5;
    planner_params["num_ctrl_points"] = 7;
    planner_params["min_ctrl_points"] = 4;
    planner_params["max_ctrl_points"] = 7;
    planner_params["spline_order"] = 4;

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
    InsatParams insat_params(dof, 2*dof, dof);
    // spline params
    BSplineOpt::BSplineOptParams spline_params(dof,
                                               planner_params["num_ctrl_points"],
                                               planner_params["spline_order"],
                                               planner_params["min_exec_duration"],
                                               planner_params["max_exec_duration"],
                                               BSplineOpt::BSplineOptParams::ConstraintMode::CONTROLPT);
    spline_params.setAdaptiveParams(planner_params["min_ctrl_points"], planner_params["max_ctrl_points"]);
    // discretization
    discretization.resize(dof);
    discretization.setOnes();
    discretization *= DISCRETIZATION;
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
    std::vector<BSplineOpt> opt_vec(num_threads, opt);
    auto opt_vec_ptr = std::make_shared<ManipulationAction::OptVecType>(opt_vec);

    // Construct actions
    ParamsType action_params;
    action_params["planner_name"] = planner_name=="insat" || planner_name=="pinsat"? 1: -1;
    vector<shared_ptr<Action>> action_ptrs;
    constructActions(action_ptrs, action_params, modelpath, discretization, opt_vec_ptr, m_vec, d_vec, num_threads);

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
//    num_runs = 10;
    for (int run = run_offset; run < run_offset+num_runs; ++run)
    {
        // Set goal conditions
        goal = goals[run];
        auto start = starts[run];

        for (auto& op : *opt_vec_ptr)
        {
            op.updateStartAndGoal(start, goal);
        }

        for (auto& m : manip_action_ptrs)
        {
            m->setGoal(goals[run]);
        }

        // Clear heuristic cache
        heuristic_cache.clear();

        // Construct planner
        shared_ptr<Planner> planner_ptr;
        constructPlanner(planner_name, planner_ptr, action_ptrs, planner_params, action_params, opt_vec[0]);

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
            planner_ptr->SetGoalState(goal);
        }


        double t=0, cost=0;
        int num_edges=0;

        bool plan_found = planner_ptr->Plan();
        auto planner_stats = planner_ptr->GetStats();

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
            cout << " | Time (s): " << planner_stats.total_time_
                 << " | Cost: " << planner_stats.path_cost_
                 << " | Length: " << planner_stats.path_length_
                 << " | State expansions: " << planner_stats.num_state_expansions_
                 << " | Lock time: " <<  planner_stats.lock_time_
                 << " | Expand time: " << planner_stats.cumulative_expansions_time_
                 << " | Threads: " << planner_stats.num_threads_spawned_ << "/" << planner_params["num_threads"] << endl;

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
            for (int i=0; i<dof; ++i)
            {
                Eigen::Map<const VecDf> svec(&starts[run][0], dof);
                Eigen::Map<const VecDf> gvec(&goals[run][0], dof);
                start_log.bottomRows(1) = svec.transpose();
                goal_log.bottomRows(1) = gvec.transpose();
            }

            if ((planner_name == "insat") || (planner_name == "pinsat"))
            {
                std::shared_ptr<InsatPlanner> insat_planner = std::dynamic_pointer_cast<InsatPlanner>(planner_ptr);
                auto soln_traj = insat_planner->getSolutionTraj();
                auto samp_traj = sampleTrajectory(soln_traj.traj_, 5e-3);
                traj_log.conservativeResize(insat_params.lowD_dims_, traj_log.cols()+samp_traj.cols());
                traj_log.rightCols(samp_traj.cols()) = samp_traj;
                all_execution_time.push_back(soln_traj.traj_.end_time());
                cout << "Execution time: " << soln_traj.traj_.end_time() << endl;
            }
            else
            {
                auto plan = planner_ptr->GetPlan();
                plan_vec.emplace_back(plan);
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
        << endl;
    }

    StateVarsType dummy_wp(6, -1);
    if ((planner_name == "insat") || (planner_name == "pinsat"))
    {
        traj_log.transposeInPlace();
        writeToCSVfile(traj_path, traj_log);
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

    writeToCSVfile(starts_path, start_log);
    writeToCSVfile(goals_path, goal_log);


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

