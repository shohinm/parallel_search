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
 * \file   BSplineOpt.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   2/26/23
 */

#include <planners/insat/opt/BSplineOpt.hpp>
#include <common/insat/InsatAction.hpp>

namespace ps
{

    /////////////////////////////////////////// BSplineOptParams /////////////////////////////////////////////////

    BSplineOpt::BSplineOptParams::BSplineOptParams() : num_positions_(6),
                                                       num_control_points_(10),
                                                       spline_order_(4),
                                                       max_duration_(1.0)
    {
        if (num_control_points_ < spline_order_)
        {
            throw std::runtime_error("Number of control points should be less than the number of basis functions.");
        }
        min_ctrl_points_ = max_ctrl_points_ = num_control_points_;
    }

    BSplineOpt::BSplineOptParams::BSplineOptParams(int num_positions, int num_control_points, int spline_order,
                                                   double min_duration, double max_duration, ConstraintMode mode) :
                                                   num_positions_(num_positions),
                                                   num_control_points_(num_control_points),
                                                   spline_order_(spline_order),
                                                   min_duration_(min_duration),
                                                   max_duration_(max_duration),
                                                   constraint_mode_(mode)
    {
        if (num_control_points_ < spline_order_)
        {
            throw std::runtime_error("Number of control points should NOT be less than the number of basis functions.");
        }
        min_ctrl_points_ = max_ctrl_points_ = num_control_points_;
    }

    void BSplineOpt::BSplineOptParams::setAdaptiveParams(int min_ctrl_points, int max_ctrl_points) {
        assert(min_ctrl_points <= max_ctrl_points);
        if (max_ctrl_points < spline_order_)
        {
            throw std::runtime_error("Number of control points should NOT be less than the number of basis functions.");
        }

        min_ctrl_points_ = min_ctrl_points;
        max_ctrl_points_ = max_ctrl_points;
    }

    void BSplineOpt::BSplineOptParams::setDurationCostWeight(double duration_cost_w) { duration_cost_w_ = duration_cost_w; }

    void BSplineOpt::BSplineOptParams::setTrajLengthCostWeight(double length_cost_w) { length_cost_w_ = length_cost_w; }

    /////////////////////////////////////////// BSplineOpt /////////////////////////////////////////////////

    BSplineOpt::BSplineOpt(const InsatParams &insat_params, const BSplineOpt::RobotParamsType &robot_params,
                           const BSplineOpt::BSplineOptParams &opt_params) : insat_params_(insat_params),
                                                                             robot_params_(robot_params),
                                                                             opt_params_(opt_params)
    {
    }

    void BSplineOpt::SetGoalChecker(std::function<bool(const StateVarsType &)> callback) {
        goal_checker_ = callback;
    }

    void BSplineOpt::updateStartAndGoal(StateVarsType &start, StateVarsType &goal) {
        Eigen::Map<VecDf> st(&start[0], start.size());
        Eigen::Map<VecDf> go(&goal[0], goal.size());
        opt_params_.global_start_ = st;
        opt_params_.global_goal_ = go;
        opt_params_.start_goal_dist_ = (go - st).norm();
    }

    bool BSplineOpt::isGoal(const VecDf &state) const {
        StateVarsType state_var;
        state_var.resize(insat_params_.lowD_dims_);
        VecDf::Map(&state_var[0], state.size()) = state;

        return goal_checker_(state_var);
    }

    MatDf BSplineOpt::sampleTrajectory(const BSplineTraj::TrajInstanceType &traj, double dt) const {
        MatDf sampled_traj;
        int i=0;
        for (double t=0.0; t<=traj.end_time(); t+=dt)
        {
            sampled_traj.conservativeResize(insat_params_.lowD_dims_, sampled_traj.cols()+1);
            sampled_traj.col(i) = traj.value(t);
            ++i;
        }
        return sampled_traj;
    }

    MatDf BSplineOpt::sampleTrajectory(const BSplineTraj &traj, double dt) const {
        return sampleTrajectory(traj.traj_, dt);
    }

    MatDf BSplineOpt::sampleTrajectory(const BSplineTraj::TrajInstanceType &traj) const {
        int n = static_cast<int>(traj.end_time()/opt_params_.min_duration_);
        double dt = n>=2? opt_params_.min_duration_: opt_params_.min_duration_/3.0;

        MatDf sampled_traj;
        /// save the init state
        VecDf x1 = traj.value(0.0);
        sampled_traj.conservativeResize(insat_params_.lowD_dims_, sampled_traj.cols()+1);
        sampled_traj.rightCols(1) = x1;
        /// loop over t
        for (double t=0; t<traj.end_time(); )
        {
            /// adaptive dt called ddt
            double ddt = dt;

            VecDf x2 = traj.value(std::min(t+ddt, traj.end_time()));

            /// keep decreasing adaptive t until distance is smaller than threshold
            while ((x2-x1).norm() > robot_params_.collision_delta_ && ddt > opt_params_.min_duration_/5.0)
            {
                ddt /= 2.0;
                x2 = traj.value(std::min(t+ddt, traj.end_time()));
            }

            /// once distance smaller than threshold collect the sample
            sampled_traj.conservativeResize(insat_params_.lowD_dims_, sampled_traj.cols()+1);
            sampled_traj.rightCols(1) = x2;

            /// update t with adaptive t
            t=t+ddt;
        }
        return sampled_traj;
    }

    MatDf BSplineOpt::sampleTrajectory(const BSplineTraj &traj) const {
        return sampleTrajectory(traj.traj_);
    }

    void BSplineOpt::addDurationAndPathCost(BSplineOpt::OptType &opt) const {
        opt.AddDurationCost(opt_params_.duration_cost_w_);
        opt.AddPathLengthCost(opt_params_.length_cost_w_);
    }

    void BSplineOpt::addStateSpaceBounds(BSplineOpt::OptType &opt) const {
        opt.AddPositionBounds(robot_params_.min_q_, robot_params_.max_q_);
        opt.AddVelocityBounds(robot_params_.min_dq_, robot_params_.max_dq_);
        opt.AddAccelerationBounds(robot_params_.min_ddq_, robot_params_.max_ddq_);
        opt.AddJerkBounds(robot_params_.min_ddq_, robot_params_.max_ddq_);
    }

    void BSplineOpt::addDurationConstraint(BSplineOpt::OptType &opt, double min_t, double max_t) const {
//            opt.AddDurationConstraint(opt_params_.max_duration_, opt_params_.max_duration_);
        opt.AddDurationConstraint(min_t, max_t);
    }

    std::vector<BSplineTraj::TrajInstanceType>
    BSplineOpt::optimizeWithCallback(const BSplineOpt::OptType &opt, drake::solvers::MathematicalProgram &prog) const {
        std::vector<BSplineTraj::TrajInstanceType> traj_trace;
        auto convergenceCallback = [&](const Eigen::Ref<const Eigen::VectorXd>& control_vec)
        {
            int r = opt.control_points().rows();
            int c = opt.control_points().cols();
//                MatDf control_matix(r, c);
//                control_matrix << control_vec;
            MatDf control_matrix(r,c);
            /// The control vector transformed to control matrix (this MUST BE DONE IN COLUMN MAJOR ORDER)
            for (int j=0; j<c; ++j)
            {
                for (int i=0; i<r; ++i)
                {
                    control_matrix(i,j) = control_vec(j*r+i);
                }
            }

            std::vector<MatDf> control_points;
            for (int i=0; i<control_matrix.cols(); ++i)
            {
                control_points.emplace_back(control_matrix.col(i));
            }

            BSplineTraj::TrajInstanceType traj = BSplineTraj::TrajInstanceType(opt.basis(), control_points);
            traj_trace.emplace_back(traj);
        };

        drake::solvers::MatrixXDecisionVariable control_points = opt.control_points();
        Eigen::Map<drake::solvers::VectorXDecisionVariable> control_vec(control_points.data(), control_points.size());

        prog.AddVisualizationCallback(convergenceCallback, control_vec);
        drake::solvers::Solve(prog);

        return traj_trace;
    }

    BSplineTraj BSplineOpt::optimize(const InsatAction *act, const VecDf &s1, const VecDf &s2, int thread_id) {
        MatDf dummy_traj(insat_params_.lowD_dims_, 2);
        dummy_traj << s1, s2;
        BSplineTraj traj;
        traj.disc_traj_ = dummy_traj;
        return traj;
    }

    BSplineTraj
    BSplineOpt::warmOptimize(const InsatAction *act, const TrajType &traj1, const TrajType &traj2, int thread_id) {
        int N = traj1.disc_traj_.cols()+traj2.disc_traj_.cols();
        MatDf init_traj(traj1.disc_traj_.rows(), N);

        init_traj << traj1.disc_traj_, traj2.disc_traj_;

        const VecDf& q0 = init_traj.leftCols(1);
        const VecDf& qF = init_traj.rightCols(1);
        VecDf dq0(insat_params_.aux_dims_);
        dq0.setZero();

        assert(q0.isApprox(opt_params_.global_start_, 5e-2));

        OptType opt(opt_params_.num_positions_,
                    opt_params_.num_control_points_,
                    opt_params_.spline_order_,
                    opt_params_.max_duration_);
        drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

        addDurationAndPathCost(opt);
        addStateSpaceBounds(opt);
        addDurationConstraint(opt, opt_params_.min_duration_, opt_params_.max_duration_);

        /// Start constraint
        opt.AddPathPositionConstraint(q0, q0, 0); // Linear constraint
        opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
        /// Goal constraint
        opt.AddPathPositionConstraint(qF, qF, 1); // Linear constraint
        if (isGoal(qF))
        {
            VecDf dqF(insat_params_.aux_dims_);
            dqF.setZero();
            opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint
        }

        /// Cost
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   q0,opt.control_points().leftCols(1));
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   qF, opt.control_points().rightCols(1));

        if (traj1.result_.is_success())
        {
//                opt.SetInitialGuess(opt.ReconstructTrajectory(traj1.result_));
            opt.SetInitialGuess(traj1.traj_);
        }

        /// Solve
        BSplineTraj traj;
        traj.result_ = drake::solvers::Solve(prog);

        bool is_feasible = false;
        if (traj.result_.is_success())
        {
            traj.traj_ = opt.ReconstructTrajectory(traj.result_);

            auto disc_traj = sampleTrajectory(traj);
            if (act->isFeasible(disc_traj, thread_id))
            {
                traj.disc_traj_ = disc_traj;
                is_feasible = true;
            }
            else
            {
                /// re-solve with callback
                if (traj1.result_.is_success())
                {
                    opt.SetInitialGuess(traj1.traj_);
                }
                std::vector<BSplineTraj::TrajInstanceType> traj_trace = optimizeWithCallback(opt, prog);
                for (int i=traj_trace.size()-1; i>=0; --i)
                {
                    auto samp_traj = sampleTrajectory(traj_trace[i]);
                    if (act->isFeasible(samp_traj, thread_id))
                    {
                        traj.traj_ = traj_trace[i];
                        traj.disc_traj_ = samp_traj;
                        is_feasible = true;
                        break;
                    }
                }
            }
        }

        if (!is_feasible)
        {
            traj.result_ = TrajType::OptResultType();
            traj.traj_ = TrajType::TrajInstanceType();
            assert(traj.disc_traj_.size() == 0);
        }

        return traj;
    }

    BSplineTraj BSplineOpt::warmOptimize(const InsatAction *act, const TrajType &traj, int thread_id) {
        assert(traj.disc_traj_.cols() >= 2);

        TrajType t1, t2;
        t1.disc_traj_ = traj.disc_traj_.leftCols(1);
        t2.disc_traj_ = traj.disc_traj_.rightCols(1);

        return warmOptimize(act, t1, t2, thread_id);
    }

    BSplineTraj
    BSplineOpt::runBSplineOpt(const InsatAction *act, const VecDf &q0, const VecDf &qF, VecDf &dq0, VecDf &dqF,
                              int order, int num_ctrl_pt, double T, int thread_id) {
        OptType opt(insat_params_.lowD_dims_,
                    num_ctrl_pt,
                    order,
                    T);
        drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

        addDurationAndPathCost(opt);
        addStateSpaceBounds(opt);
        addDurationConstraint(opt, T, T);

        /// Terminal position constraint
        opt.AddPathPositionConstraint(q0, q0, 0); // Linear constraint
        opt.AddPathPositionConstraint(qF, qF, 1); // Linear constraint

        /// Terminal velocity constraints if needed
        if (dq0.size()>0)
        {
            opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
        }
        if (dqF.size()>0)
        {
            opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint
        }


        /// Cost
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   q0,opt.control_points().leftCols(1));
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   qF, opt.control_points().rightCols(1));

        /// Solve
        BSplineTraj traj;
        traj.result_ = drake::solvers::Solve(prog);

        bool is_feasible = false;
        if (traj.result_.is_success())
        {
            traj.traj_ = opt.ReconstructTrajectory(traj.result_);

            auto disc_traj = sampleTrajectory(traj);
            if (act->isFeasible(disc_traj, thread_id))
            {
                traj.disc_traj_ = disc_traj;
                is_feasible = true;
            }
            else
            {
                std::vector<BSplineTraj::TrajInstanceType> traj_trace = optimizeWithCallback(opt, prog);
                for (int i=traj_trace.size()-1; i>=0; --i)
                {
                    auto samp_traj = sampleTrajectory(traj_trace[i]);
                    if (act->isFeasible(samp_traj, thread_id))
                    {
                        traj.traj_ = traj_trace[i];
                        traj.disc_traj_ = samp_traj;
                        is_feasible = true;
                        break;
                    }
                }
            }
        }

        if (!is_feasible)
        {
            traj.result_ = TrajType::OptResultType();
            traj.traj_ = TrajType::TrajInstanceType();
            assert(traj.disc_traj_.size() == 0);
        }

        return traj;
    }

    BSplineTraj BSplineOpt::runBSplineOptWithInitGuess(const InsatAction *act, const BSplineTraj &t1, BSplineTraj &t2,
                                                       const VecDf &q0, const VecDf &qF, VecDf &dq0, VecDf &dqF,
                                                       int order, int c1, int c2, double T, int thread_id) {
        OptType opt(insat_params_.lowD_dims_,
                    c1+c2,
                    order,
                    T);
        drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

        addDurationAndPathCost(opt);
        addStateSpaceBounds(opt);
        addDurationConstraint(opt, T, T);

        /// Terminal position constraint
        opt.AddPathPositionConstraint(q0, q0, 0); // Linear constraint
        opt.AddPathPositionConstraint(qF, qF, 1); // Linear constraint

        /// Terminal velocity constraints if needed
        if (dq0.size()>0)
        {
            opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
        }
        if (dqF.size()>0)
        {
            opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint
        }

        /// Control vector blending
        int nc1 = t1.traj_.num_control_points();
        int nc2 = t2.traj_.num_control_points();
        assert(c1 <= nc1);
        assert(c2 <= nc2);
        VecDi ctrl_idx1 = VecDf::LinSpaced(c1, 0, nc1-1).cast<int>();
        VecDi ctrl_idx2 = VecDf::LinSpaced(c2, 0, nc2-1).cast<int>();

        std::vector<MatDf> blend_ctrl_pt;
        for (int i=0; i<ctrl_idx1.size(); ++i)
        {
            blend_ctrl_pt.push_back(t1.traj_.control_points()[ctrl_idx1(i)]);
        }
        for (int i=0; i<ctrl_idx2.size(); ++i)
        {
            blend_ctrl_pt.push_back(t2.traj_.control_points()[ctrl_idx2(i)]);
        }

        /// number of control points should be equal to the number of basis
        auto bspline_basis = drake::math::BsplineBasis<double>(order,
                                                               blend_ctrl_pt.size(),
                                                               drake::math::KnotVectorType::kClampedUniform,
                                                               0, T);
        auto init_guess = BSplineTraj::TrajInstanceType(bspline_basis, blend_ctrl_pt);

        /// Now set the init guess
        opt.SetInitialGuess(init_guess);

        /// Cost
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   q0,opt.control_points().leftCols(1));
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   qF, opt.control_points().rightCols(1));

        /// Solve
        BSplineTraj traj;
        traj.result_ = drake::solvers::Solve(prog);

        bool is_feasible = false;
        if (traj.result_.is_success())
        {
            traj.traj_ = opt.ReconstructTrajectory(traj.result_);

            auto disc_traj = sampleTrajectory(traj);
            if (act->isFeasible(disc_traj, thread_id))
            {
                traj.disc_traj_ = disc_traj;
                is_feasible = true;
            }
            else
            {
                std::vector<BSplineTraj::TrajInstanceType> traj_trace = optimizeWithCallback(opt, prog);
                for (int i=traj_trace.size()-1; i>=0; --i)
                {
                    auto samp_traj = sampleTrajectory(traj_trace[i]);
                    if (act->isFeasible(samp_traj, thread_id))
                    {
                        traj.traj_ = traj_trace[i];
                        traj.disc_traj_ = samp_traj;
                        is_feasible = true;
                        break;
                    }
                }
            }
        }

        if (!is_feasible)
        {
            traj.result_ = TrajType::OptResultType();
            traj.traj_ = TrajType::TrajInstanceType();
            assert(traj.disc_traj_.size() == 0);
        }

        return traj;
    }

    BSplineTraj BSplineOpt::optimize(const InsatAction *act, const BSplineTraj &incoming_traj, const VecDf &curr_state,
                                     const VecDf &succ_state, int thread_id) {

        bool is_start=false, is_goal=false;
        VecDf dq0, dqF;
        if (curr_state.isApprox(opt_params_.global_start_, 5e-2))
        {
            is_start=true;
            dq0.resize(insat_params_.aux_dims_);
            dq0.setZero();
        }
        if (isGoal(succ_state))
        {
            is_goal=true;
            dqF.resize(insat_params_.aux_dims_);
            dqF.setZero();
        }

        double To = (succ_state-curr_state).norm()/opt_params_.start_goal_dist_;
        int nc = std::max(opt_params_.min_ctrl_points_,
                          static_cast<int>(To*opt_params_.max_ctrl_points_));
        To = std::max(To, opt_params_.min_duration_);
        To = std::min(To, opt_params_.max_duration_);

        auto inc_traj = runBSplineOpt(act,
                                      curr_state, succ_state,
                                      dq0, dqF,
                                      opt_params_.spline_order_, nc, To,
                                      thread_id);

        BSplineTraj full_traj;
        if (!incoming_traj.isValid())
        {
            full_traj = inc_traj;
            return full_traj;
        }

        if (inc_traj.isValid() && incoming_traj.isValid())
        {
            int c1 = incoming_traj.size()>0? incoming_traj.traj_.num_control_points() : 0;
            int c2 = nc;

            double Two = (succ_state-opt_params_.global_start_).norm()/opt_params_.start_goal_dist_;
            int tc = static_cast<int>(Two*opt_params_.max_ctrl_points_);
            tc = std::max(tc, opt_params_.min_ctrl_points_);
            int nc1 = static_cast<int>((static_cast<double>(c1)/(c1+c2))*tc);
            int nc2 = tc - nc1;
            Two = std::max(Two, opt_params_.min_duration_);
            Two = std::min(Two, opt_params_.max_duration_);

            const auto blend_q0 = incoming_traj.traj_.InitialValue();
            const auto blend_qF = inc_traj.traj_.FinalValue();
            full_traj = runBSplineOptWithInitGuess(act,
                                                   incoming_traj, inc_traj,
                                                   blend_q0, blend_qF,
                                                   dq0, dqF,
                                                   opt_params_.spline_order_,
                                                   nc1, nc2, Two,
                                                   thread_id);
        }

        return full_traj;
    }

    BSplineTraj BSplineOpt::optimize(const InsatAction *act,
                                     const BSplineTraj &incoming_traj,
                                     const std::vector<StateVarsType> &ancestors,
                                     const StateVarsType &successor,
                                     int thread_id) {

        MatDf path(insat_params_.lowD_dims_, ancestors.size());
        for (int i=0; i<ancestors.size(); ++i)
        {
            for (int j=0; j<ancestors[i].size(); ++j)
            {
                path(j, i) = ancestors[i][j];
            }
        }

        if (!path.leftCols(1).isApprox(opt_params_.global_start_, 5e-2))
        {
            path.rowwise().reverseInPlace();
        }

        path.conservativeResize(insat_params_.lowD_dims_, path.cols()+1);
        for (int i=0; i<successor.size(); ++i)
        {
            path.rightCols(1)(i) = successor[i];
        }
        assert(path.cols()>=2);

        BSplineTraj traj;
        if (opt_params_.constraint_mode_ == BSplineOptParams::ConstraintMode::WAYPT)
        {
            traj = fitBestWaypointBSpline(act, path, incoming_traj, thread_id);
        }
        else if (opt_params_.constraint_mode_ == BSplineOptParams::ConstraintMode::CONTROLPT)
        {
            traj = fitBestControlPointBSpline(act, path, incoming_traj, thread_id);
        }
        return traj;
    }

    BSplineTraj BSplineOpt::optimizeWithWaypointConstraint(VecDf& st, VecDf& go, MatDf& wp, VecDf& s_wp) const {

        OptType opt(insat_params_.lowD_dims_,
                    opt_params_.max_ctrl_points_,
                    opt_params_.spline_order_,
                    opt_params_.max_duration_);

        drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

        addDurationAndPathCost(opt);
        addStateSpaceBounds(opt);
        addDurationConstraint(opt, opt_params_.min_duration_, opt_params_.max_duration_);

        VecDf dq0(insat_params_.aux_dims_), dqF(insat_params_.aux_dims_);
        dq0.setZero(); dqF.setZero();

        /// Start constraint
        opt.AddPathPositionConstraint(st, st, 0); // Linear constraint
        opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
        /// Goal constraint
        opt.AddPathPositionConstraint(go, go, 1); // Linear constraint
        opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint

        /// Add waypoint constraint
        assert(wp.cols() == s_wp.size());
        for (int i=0; i<wp.cols(); ++i)
        {
            opt.AddPathPositionConstraint(wp.col(i), wp.col(i), s_wp(i));
        }

        /// Cost
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   st,opt.control_points().leftCols(1));
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   go, opt.control_points().rightCols(1));


        /// Solve
        BSplineTraj traj;
        traj.result_ = drake::solvers::Solve(prog);

        bool is_feasible = false;
        if (traj.result_.is_success())
        {
            traj.traj_ = opt.ReconstructTrajectory(traj.result_);
        }
        else
        {
            traj.result_ = TrajType::OptResultType();
            traj.traj_ = TrajType::TrajInstanceType();
            assert(traj.disc_traj_.size() == 0);
        }

        return traj;
    }

    BSplineTraj BSplineOpt::optimizeWithWaypointConstraintAndInit(const InsatAction *act,
                                                                  VecDf &st, VecDf &go,
                                                                  MatDf &wp, VecDf &s_wp,
                                                                  const BSplineTraj &init_traj,
                                                                  int thread_id) const {
        OptType opt(insat_params_.lowD_dims_,
                    opt_params_.max_ctrl_points_,
                    opt_params_.spline_order_,
                    opt_params_.max_duration_);

        drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

        addDurationAndPathCost(opt);
        addStateSpaceBounds(opt);
        addDurationConstraint(opt, opt_params_.min_duration_, opt_params_.max_duration_);

        VecDf dq0(insat_params_.aux_dims_);
        dq0.setZero();

        assert(st.isApprox(opt_params_.global_start_, 5e-2));

        /// Start constraint
        opt.AddPathPositionConstraint(st, st, 0); // Linear constraint
        opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
        /// Goal constraint
        opt.AddPathPositionConstraint(go, go, 1); // Linear constraint
        if (isGoal(go))
        {
            VecDf dqF(insat_params_.aux_dims_);
            dqF.setZero();
            opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint
        }

        /// Add waypoint constraint
        assert(wp.cols() == s_wp.size());
        for (int i=0; i<wp.cols(); ++i)
        {
//            opt.AddPathPositionConstraint(wp.col(i), wp.col(i), s_wp(i));
//            opt.AddPathVelocityConstraint(robot_params_.min_dq_, robot_params_.max_dq_, s_wp(i));
        }

        /// Cost
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   st,opt.control_points().leftCols(1));
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   go, opt.control_points().rightCols(1));

        if (init_traj.result_.is_success())
        {
            opt.SetInitialGuess(init_traj.traj_);
        }

        /// Solve
        BSplineTraj traj;
        traj.result_ = drake::solvers::Solve(prog);

        bool is_feasible = false;
        if (traj.result_.is_success())
        {
            traj.traj_ = opt.ReconstructTrajectory(traj.result_);

            auto disc_traj = sampleTrajectory(traj);
            if (act->isFeasible(disc_traj, thread_id))
            {
                traj.disc_traj_ = disc_traj;
                is_feasible = true;
            }
        }

        if (!is_feasible)
        {
            traj.result_ = TrajType::OptResultType();
            traj.traj_ = TrajType::TrajInstanceType();
            assert(traj.disc_traj_.size() == 0);
        }

        return traj;
    }


    BSplineTraj
    BSplineOpt::optimizeWithWaypointConstraintAndCallback(const InsatAction *act,
                                                          VecDf &st, VecDf &go, MatDf &wp,
                                                          VecDf &s_wp, int thread_id) const {
        OptType opt(insat_params_.lowD_dims_,
                    opt_params_.max_ctrl_points_,
                    opt_params_.spline_order_,
                    opt_params_.max_duration_);

        drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

        addDurationAndPathCost(opt);
        addStateSpaceBounds(opt);
        addDurationConstraint(opt, opt_params_.min_duration_, opt_params_.max_duration_);

        VecDf dq0(insat_params_.aux_dims_);
        dq0.setZero();

        /// Start constraint
        opt.AddPathPositionConstraint(st, st, 0); // Linear constraint
        opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
        /// Goal constraint
        opt.AddPathPositionConstraint(go, go, 1); // Linear constraint
        if (isGoal(go))
        {
            VecDf dqF(insat_params_.aux_dims_);
            dqF.setZero();
            opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint
        }

        /// Add waypoint constraint
        assert(wp.cols() == s_wp.size());
        for (int i=0; i<wp.cols(); ++i)
        {
            opt.AddPathPositionConstraint(wp.col(i), wp.col(i), s_wp(i));
        }

        /// Cost
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   st,opt.control_points().leftCols(1));
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   go, opt.control_points().rightCols(1));

        /// Solve
        BSplineTraj traj;
        traj.result_ = drake::solvers::Solve(prog);

        bool is_feasible = false;
        if (traj.result_.is_success())
        {
            traj.traj_ = opt.ReconstructTrajectory(traj.result_);

            auto disc_traj = sampleTrajectory(traj);
            if (act->isFeasible(disc_traj, thread_id))
            {
                traj.disc_traj_ = disc_traj;
                is_feasible = true;
            }
            else
            {
                /// re-solve with callback
                std::vector<BSplineTraj::TrajInstanceType> traj_trace = optimizeWithCallback(opt, prog);
                for (int i=traj_trace.size()-1; i>=0; --i)
                {
                    auto samp_traj = sampleTrajectory(traj_trace[i]);
                    if (act->isFeasible(samp_traj, thread_id))
                    {
                        traj.traj_ = traj_trace[i];
                        traj.disc_traj_ = samp_traj;
                        is_feasible = true;
                        break;
                    }
                }
            }
        }

        if (!is_feasible)
        {
            traj.result_ = TrajType::OptResultType();
            traj.traj_ = TrajType::TrajInstanceType();
            assert(traj.disc_traj_.size() == 0);
        }

        return traj;
    }

    BSplineTraj BSplineOpt::fitBestWaypointBSpline(const InsatAction *act,
                                                   MatDf &path, const BSplineTraj& init_traj,
                                                   int thread_id) const {

        BSplineTraj traj;

        VecDf rs(path.cols());
        rs(0) = 0.0;
        for (int i=0; i < path.cols() - 1; ++i)
        {
            rs(i+1) = rs(i) + (path.col(i + 1) - path.col(i)).norm();
        }
        rs /= rs(rs.size() - 1);

        for (int i=2; i <= path.cols(); ++i)
        {
            VecDi ctrl_idx = VecDf::LinSpaced(i, 0, path.cols() - 1).cast<int>();
            MatDf wp(path.rows(), ctrl_idx.size() - 2);
            VecDf s_wp(ctrl_idx.size()-2);

            VecDf st = path.col(ctrl_idx(0));
            VecDf go = path.col(ctrl_idx(ctrl_idx.size() - 1));
            for (int j=1, k=0; j<ctrl_idx.size()-1; ++j, ++k)
            {
                wp.col(k) = path.col(ctrl_idx(j));
                s_wp(k) = rs(ctrl_idx(j));
            }

            traj = optimizeWithWaypointConstraintAndInit(act, st, go, wp, s_wp,
                                                         init_traj, thread_id);

            if (traj.disc_traj_.cols() > 0)
            {
                break;
            }
        }

        return traj;
    }

    BSplineTraj BSplineOpt::fitBestControlPointBSpline(const InsatAction *act,
                                                       MatDf &path, const BSplineTraj& init_traj,
                                                       int thread_id) const {
        BSplineTraj traj;
        VecDf st = path.leftCols(1);
        VecDf go = path.rightCols(1);


        if (path.cols() < opt_params_.min_ctrl_points_)
        {
            /// Do direct optimization
            /// This shouldn't be failing (at least the assumption is)
            traj = directOptimize(act, path.leftCols(1), path.rightCols(1), thread_id);
            return traj;
        }

        /// For min to max control points
        for (int ctrl=opt_params_.min_ctrl_points_; ctrl<opt_params_.max_ctrl_points_; ++ctrl)
        {
            /// set up the basis functions
            auto basis = drake::math::BsplineBasis<double>(opt_params_.spline_order_, ctrl,
                                                   drake::math::KnotVectorType::kClampedUniform);

            BSplineTraj::TrajInstanceType init_guess;
            if (ctrl == init_traj.traj_.num_control_points())
            {
                /// Use the incoming trajectory as initial guess
                init_guess = init_traj.traj_;
            }
            else
            {
                /// Construct control points
                VecDi ctrl_idx = VecDf::LinSpaced(ctrl, 0, path.cols() - 1).cast<int>();

                std::vector<MatDf> control_points;
                for (int j=0; j<ctrl_idx.size(); ++j)
                {
                    control_points.emplace_back(path.col(ctrl_idx(j)));
                }
                init_guess = BSplineTraj::TrajInstanceType(basis, control_points);
            }
            traj =optimizeWithInitAndCallback(act, path.leftCols(1), path.rightCols(1), init_guess, thread_id);
            if (traj.disc_traj_.size() > 0)
            {
                break;
            }
        }
        return traj;
    }


    BSplineTraj
    BSplineOpt::directOptimize(const InsatAction *act,
                               const VecDf &q0, const VecDf &qF,
                               int thread_id) const {
        OptType opt(insat_params_.lowD_dims_,
                    opt_params_.max_ctrl_points_,
                    opt_params_.spline_order_,
                    opt_params_.max_duration_);

        drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

        addDurationAndPathCost(opt);
        addStateSpaceBounds(opt);
        addDurationConstraint(opt, opt_params_.min_duration_, opt_params_.max_duration_);

        VecDf dq0(insat_params_.aux_dims_);
        dq0.setZero();

        /// Start constraint
        opt.AddPathPositionConstraint(q0, q0, 0); // Linear constraint
        opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
        /// Goal constraint
        opt.AddPathPositionConstraint(qF, qF, 1); // Linear constraint
        if (isGoal(qF))
        {
            VecDf dqF(insat_params_.aux_dims_);
            dqF.setZero();
            opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint
        }

        /// Cost
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   q0,opt.control_points().leftCols(1));
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   qF, opt.control_points().rightCols(1));

        /// Solve
        BSplineTraj traj;
        traj.result_ = drake::solvers::Solve(prog);

        bool is_feasible = false;
        if (traj.result_.is_success())
        {
            traj.traj_ = opt.ReconstructTrajectory(traj.result_);

            auto disc_traj = sampleTrajectory(traj);
            if (act->isFeasible(disc_traj, thread_id))
            {
                traj.disc_traj_ = disc_traj;
                is_feasible = true;
            }
            else
            {
                /// re-solve with callback
                std::vector<BSplineTraj::TrajInstanceType> traj_trace = optimizeWithCallback(opt, prog);
                for (int i=traj_trace.size()-1; i>=0; --i)
                {
                    auto samp_traj = sampleTrajectory(traj_trace[i]);
                    if (act->isFeasible(samp_traj, thread_id))
                    {
                        traj.traj_ = traj_trace[i];
                        traj.disc_traj_ = samp_traj;
                        is_feasible = true;
                        break;
                    }
                }
            }
        }

        if (!is_feasible)
        {
            traj.result_ = TrajType::OptResultType();
            traj.traj_ = TrajType::TrajInstanceType();
            assert(traj.disc_traj_.size() == 0);
        }

        return traj;

    }

    BSplineTraj BSplineOpt::optimizeWithInitAndCallback(const InsatAction *act,
                                                        const VecDf &q0, const VecDf &qF,
                                                        BSplineTraj::TrajInstanceType &init_traj,
                                                        int thread_id) const {

        OptType opt(init_traj);

        drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

        addDurationAndPathCost(opt);
        addStateSpaceBounds(opt);
        addDurationConstraint(opt, opt_params_.min_duration_, opt_params_.max_duration_);

        VecDf dq0(insat_params_.aux_dims_);
        dq0.setZero();

        assert(q0.isApprox(opt_params_.global_start_, 5e-2));

        /// Start constraint
        opt.AddPathPositionConstraint(q0, q0, 0); // Linear constraint
        opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
        /// Goal constraint
        opt.AddPathPositionConstraint(qF, qF, 1); // Linear constraint
        if (isGoal(qF))
        {
            VecDf dqF(insat_params_.aux_dims_);
            dqF.setZero();
            opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint
        }

        /// Cost
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   q0,opt.control_points().leftCols(1));
        prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                   qF, opt.control_points().rightCols(1));

        /// Solve
        BSplineTraj traj;
        traj.result_ = drake::solvers::Solve(prog);

        bool is_feasible = false;
        if (traj.result_.is_success())
        {
            traj.traj_ = opt.ReconstructTrajectory(traj.result_);

            auto disc_traj = sampleTrajectory(traj);
            if (act->isFeasible(disc_traj, thread_id))
            {
                traj.disc_traj_ = disc_traj;
                is_feasible = true;
            }
            else
            {
                /// re-solve with callback
                std::vector<BSplineTraj::TrajInstanceType> traj_trace = optimizeWithCallback(opt, prog);
                for (int i=traj_trace.size()-1; i>=0; --i)
                {
                    auto samp_traj = sampleTrajectory(traj_trace[i]);
                    if (act->isFeasible(samp_traj, thread_id))
                    {
                        traj.traj_ = traj_trace[i];
                        traj.disc_traj_ = samp_traj;
                        is_feasible = true;
                        break;
                    }
                }
            }
        }

        if (!is_feasible)
        {
            traj.result_ = TrajType::OptResultType();
            traj.traj_ = TrajType::TrajInstanceType();
            assert(traj.disc_traj_.size() == 0);
        }

        return traj;
    }

    MatDf BSplineOpt::postProcess(std::vector<PlanElement>& path, double& cost, double time_limit, const InsatAction* act) const {
        MatDf disc_traj;

        /// Return already if the time length is negative (means time limit already elapsed)
        if (time_limit < 0.0)
        {
            cost = 0.0;
            return disc_traj;
        }

        typedef std::chrono::high_resolution_clock Clock;
        using namespace std::chrono;
        auto start_time = Clock::now();

        MatDf eig_path(insat_params_.lowD_dims_, path.size());
        for (int i=0; i<path.size(); ++i)
        {
            for (int j=0; j<path[i].state_.size(); ++j)
            {
                eig_path(j, i) = path[i].state_[j];
            }
        }
        assert(eig_path.cols()>=2);

        VecDf rs(eig_path.cols());
        rs(0) = 0.0;
        for (int i=0; i<eig_path.cols()-1; ++i)
        {
            rs(i+1) = rs(i) + (eig_path.col(i + 1) - eig_path.col(i)).norm();
        }
        rs /= rs(rs.size() - 1);

        cost = 0.0;
        bool success = false;
        for (int i=2; i<=eig_path.cols(); ++i)
        {
            VecDi ctrl_idx = VecDf::LinSpaced(i, 0, eig_path.cols()-1).cast<int>();
            MatDf wp(eig_path.rows(), ctrl_idx.size()-2);
            VecDf s_wp(ctrl_idx.size()-2);

            VecDf st = eig_path.col(ctrl_idx(0));
            VecDf go = eig_path.col(ctrl_idx(ctrl_idx.size()-1));
            for (int j=1, k=0; j<ctrl_idx.size()-1; ++j, ++k)
            {
                wp.col(k) = eig_path.col(ctrl_idx(j));
                s_wp(k) = rs(ctrl_idx(j));
            }

//            auto traj = optimizeWithWaypointConstraint(st, go, wp, s_wp);
            auto traj = optimizeWithWaypointConstraintAndCallback(act, st, go, wp, s_wp, 0);

            if (traj.result_.is_success())
            {
                auto samp_traj = sampleTrajectory(traj);
                if (act->isFeasible(samp_traj, 0))
                {
                    success = true;
                    disc_traj = samp_traj;
                    cost = calculateCost(disc_traj);

                    auto time_traj = sampleTrajectory(traj, 5e-3);
                    path.clear();
                    for (int i=0; i<time_traj.cols(); ++i)
                    {
                        StateVarsType pt;
                        pt.resize(insat_params_.lowD_dims_);
                        VecDf::Map(&pt[0], time_traj.col(i).size()) = time_traj.col(i);
                        path.emplace_back(pt, nullptr, cost);
                    }
                    break;
                }
                else
                {
                    path.clear();
                    cost = 0.0;
                }
            }
            auto end_time = Clock::now();
            double time_elapsed = duration_cast<duration<double> >(end_time - start_time).count();
            if (time_elapsed > time_limit)
            {
                path.clear();
                cost = 0.0;
                break;
            }
        }

        return disc_traj;
    }

    double BSplineOpt::calculateCost(const TrajType &traj) const {
        return calculateCost(traj.disc_traj_);
    }

    double BSplineOpt::calculateCost(const MatDf &disc_traj) const {
        double cost = 0;
        for (int i=0; i<disc_traj.cols()-1; ++i)
        {
            cost += (disc_traj.col(i+1)-disc_traj.col(i)).norm();
        }
        return cost;
    }

}