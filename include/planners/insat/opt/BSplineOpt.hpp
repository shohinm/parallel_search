//
// Created by Ramkumar  on 2/17/23.
//

#ifndef BSPLINEOPT_HPP
#define BSPLINEOPT_HPP

// PS
#include <common/Types.hpp>
#include <iostream>
#include <common/robots/Abb.hpp>

namespace ps
{

    class BSplineOpt
    {
    public:
        typedef drake::planning::trajectory_optimization::KinematicTrajectoryOptimization OptType;
        typedef drake::solvers::Binding<drake::solvers::Cost> CostType;
        typedef drake::solvers::Binding<drake::solvers::Constraint> ConstraintType;
        typedef drake::solvers::MathematicalProgramResult OptResult;

        // Robot
        typedef IRB1600 RobotParamsType;

        struct BSplineOptParams
        {
            BSplineOptParams() : num_positions_(6),
                                 num_control_points_(10),
                                 spline_order_(4),
                                 duration_(1.0) {}

            BSplineOptParams(int num_positions, int num_control_points,
                             int spline_order, double duration=1.0) : num_positions_(num_positions),
                                                                      num_control_points_(num_control_points),
                                                                      spline_order_(spline_order),
                                                                      duration_(duration) {}

            int num_positions_;
            int num_control_points_;
            int spline_order_;
            double duration_;

            /// Adaptive BSpline optimization
            int min_ctrl_points_;
            int max_ctrl_points_;
            VecDf global_start_; /// For now assuming higher derivatives = 0
            VecDf global_goal_; /// For now assuming higher derivatives = 0
            double total_duration_;


        };


        BSplineOpt(const InsatParams& insat_params,
                   const RobotParamsType& robot_params,
                   const BSplineOptParams& opt_params) : insat_params_(insat_params),
                                                         robot_params_(robot_params),
                                                         opt_params_(opt_params)
        {
        }

        void SetGoalChecker(std::function<bool(const StateVarsType&)> callback)
        {
            goal_checker_ = callback;
        }

        bool isGoal(const VecDf& state)
        {
            StateVarsType state_var;
            state_var.resize(insat_params_.lowD_dims_);
            VecDf::Map(&state_var[0], state.size()) = state;

            return goal_checker_(state_var);
        }

        MatDf sampleTrajectory(const BSplineTraj& traj, double dt=1e-1) const
        {
            return sampleTrajectory(traj.traj_);
        }

        MatDf sampleTrajectory(const BSplineTraj::TrajInstanceType& traj, double dt=1e-1) const
        {
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

        std::vector<BSplineTraj::TrajInstanceType> optimizeWithCallback(const OptType& opt,
                                                                        drake::solvers::MathematicalProgram& prog)
        {
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

        BSplineTraj optimize(const InsatAction* act, const VecDf& s1, const VecDf& s2, int thread_id)
        {
            MatDf dummy_traj(insat_params_.lowD_dims_, 2);
            dummy_traj << s1, s2;
            BSplineTraj traj;
            traj.disc_traj_ = dummy_traj;
            return traj;
        }

        BSplineTraj warmOptimize(const InsatAction* act, const TrajType& traj1, const TrajType & traj2, int thread_id)
        {
            int N = traj1.disc_traj_.cols()+traj2.disc_traj_.cols();
            MatDf init_traj(traj1.disc_traj_.rows(), N);

            init_traj << traj1.disc_traj_, traj2.disc_traj_;

            const VecDf& q0 = init_traj.leftCols(1);
            const VecDf& qF = init_traj.rightCols(1);
            VecDf dq0(insat_params_.aux_dims_);
            dq0.setZero();

            OptType opt(opt_params_.num_positions_,
                        opt_params_.num_control_points_,
                        opt_params_.spline_order_,
                        opt_params_.duration_);
            drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

            opt.AddDurationCost(1.0);
            opt.AddPathLengthCost(0.1);

            opt.AddPositionBounds(robot_params_.min_q_, robot_params_.max_q_);
            opt.AddVelocityBounds(robot_params_.min_dq_, robot_params_.max_dq_);
//            opt.AddAccelerationBounds(robot_params_.min_ddq_, robot_params_.max_ddq_);

            opt.AddDurationConstraint(opt_params_.duration_, opt_params_.duration_);

            /// Start constraint
            opt.AddPathPositionConstraint(q0, q0, 0); // Linear constraint
            opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
            /// Goal constraint
            opt.AddPathPositionConstraint(qF, qF, 1); // Linear constraint
            if (isGoal(qF))
            {
                VecDf dqF(insat_params_.aux_dims_);
                dqF.setZero();
                opt.AddPathVelocityConstraint(qF, qF, 1); // Linear constraint
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

            if (traj.result_.is_success())
            {
                traj.traj_ = opt.ReconstructTrajectory(traj.result_);

                auto disc_traj = sampleTrajectory(traj);
                if (act->isFeasible(disc_traj, thread_id))
                {
                    traj.disc_traj_ = disc_traj;
                }
                else
                {
                    opt.SetInitialGuess(traj1.traj_);
                    std::vector<BSplineTraj::TrajInstanceType> traj_trace = optimizeWithCallback(opt, prog);
                    for (int i=traj_trace.size()-1; i>=0; --i)
                    {
                        auto samp_traj = sampleTrajectory(traj_trace[i]);
                        if (act->isFeasible(samp_traj, thread_id))
                        {
                            traj.traj_ = traj_trace[i];
                            traj.disc_traj_ = samp_traj;
                            break;
                        }
                    }
                }
            }

//            std::cout << "Generated trajectory sampled at length: " << traj.disc_traj_.size() << std::endl;

            return traj;
        }

        BSplineTraj warmOptimize(const InsatAction* act, const TrajType& traj, int thread_id)
        {
            assert(traj.disc_traj_.cols() >= 2);

            TrajType t1, t2;
            t1.disc_traj_ = traj.disc_traj_.leftCols(1);
            t2.disc_traj_ = traj.disc_traj_.rightCols(1);

            return warmOptimize(act, t1, t2, thread_id);
        }


        BSplineTraj optimize(const InsatAction* act,
                             BSplineTraj incoming_traj,
                             VecDf& curr_state,
                             VecDf& succ_state)
        {



            OptType opt(opt_params_.num_positions_,
                        opt_params_.num_control_points_,
                        opt_params_.spline_order_,
                        opt_params_.duration_);
            drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());


        }

        virtual double calculateCost(const TrajType& traj)
        {
            auto& disc_traj = traj.disc_traj_;
            double cost = 0;
            for (int i=0; i<disc_traj.cols()-1; ++i)
            {
                cost += (disc_traj.col(i+1)-disc_traj.col(i)).norm();
            }
            return cost;
        }

//        int clearCosts()
//        {
//        }

//        int clearConstraints()
//        {
//            auto lin_constraints = prog_.linear_constraints();
//            for (auto& cn : lin_constraints)
//            {
//                prog_.RemoveConstraint(cn);
//            }
//        }

        /// Params
        InsatParams insat_params_;
        IRB1600 robot_params_;
        BSplineOptParams opt_params_;

        /// Adaptive BSplineOpt
        std::function<double(const StateVarsType&)> goal_checker_;

        /// @TODO (@ram): Rewrite Drake optimizer and make it highly reusable.
        /// Optimizer
//        std::shared_ptr<OptType> opt_;
//        drake::solvers::MathematicalProgram& prog_;
//        std::vector<CostType> costs_;
//        std::vector<ConstraintType> constraints_;

    };

}



#endif //BSPLINEOPT_HPP
