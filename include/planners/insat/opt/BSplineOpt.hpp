//
// Created by Ramkumar  on 2/17/23.
//

#ifndef BSPLINEOPT_HPP
#define BSPLINEOPT_HPP

// PS
#include <common/Types.hpp>


namespace ps
{


    struct ABBParams
    {
        ABBParams() : min_q_(6),
                      max_q_(6),
                      min_dq_(6),
                      max_dq_(6)
        {
            min_q_ << -3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813;
            max_q_ << 3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813;
            min_dq_ << -2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854;
            max_dq_ << 2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854;
        }

        VecDf min_q_, max_q_, min_dq_, max_dq_;
    };

    class BSplineOpt
    {
    public:
        typedef drake::planning::trajectory_optimization::KinematicTrajectoryOptimization OptType;
        typedef drake::solvers::Binding<drake::solvers::Cost> CostType;
        typedef drake::solvers::Binding<drake::solvers::Constraint> ConstraintType;
        typedef drake::solvers::MathematicalProgramResult OptResult;

        // Robot
        typedef ABBParams RobotParamsType;

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
        };


        BSplineOpt(const InsatParams& insat_params,
                   const RobotParamsType& robot_params,
                   const BSplineOptParams& opt_params) : insat_params_(insat_params),
                                                         robot_params_(robot_params),
                                                         opt_params_(opt_params)
        {
        }

        BSplineTraj optimizeWithCallback(const VecDf& s1, const VecDf& s2)
        {
            OptType opt(opt_params_.num_positions_,
                        opt_params_.num_control_points_,
                        opt_params_.spline_order_,
                        opt_params_.duration_);
            drake::solvers::MathematicalProgram& prog(opt.get_mutable_prog());

            opt.AddDurationCost(1.0);
            opt.AddPathLengthCost(1.0);

            opt.AddPositionBounds(robot_params_.min_q_, robot_params_.max_q_);
            opt.AddVelocityBounds(robot_params_.min_dq_, robot_params_.max_dq_);

            opt.AddDurationConstraint(2, 2);

            const VecDf& q0 = s1.topRows(insat_params_.lowD_dims_);
            const VecDf& qF = s2.topRows(insat_params_.lowD_dims_);
            const VecDf& dq0 = s1.bottomRows(insat_params_.aux_dims_);
            const VecDf& dqF = s2.bottomRows(insat_params_.aux_dims_);

            /// Start constraint
            opt.AddPathPositionConstraint(q0, q0, 0); // Linear constraint
            opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
            /// Goal constraint
            opt.AddPathPositionConstraint(qF, qF, 1); // Linear constraint
            opt.AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint

            /// Cost
            auto c1 = prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                                 q0,opt.control_points().leftCols(1));
            auto c2 = prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                                 qF, opt.control_points().rightCols(1));

            /// Solve
            BSplineTraj traj;
            traj.result_ = drake::solvers::Solve(prog);

            return traj;

        }

        BSplineTraj optimize(const InsatAction* act, const VecDf& s1, const VecDf& s2)
        {
            MatDf dummy_traj(insat_params_.fullD_dims_, 2);
            dummy_traj << s1, s2;
            BSplineTraj traj;
            traj.disc_traj_ = dummy_traj;
            return traj;
        }

        MatDf sampleTrajectory(const BSplineTraj& traj, double dt=1e-1)
        {
            MatDf sampled_traj;
            int i=0;
            for (double t=0.0; t<=traj.traj_.end_time(); t+=dt)
            {
                sampled_traj.conservativeResize(insat_params_.lowD_dims_, sampled_traj.cols()+1);
                sampled_traj.col(i) = traj.traj_.value(t);
                ++i;
            }
        }

        BSplineTraj warmOptimize(const InsatAction* act, const TrajType& traj1, const TrajType & traj2)
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
            opt.AddPathLengthCost(1.0);

            opt.AddPositionBounds(robot_params_.min_q_, robot_params_.max_q_);
            opt.AddVelocityBounds(robot_params_.min_dq_, robot_params_.max_dq_);

            opt.AddDurationConstraint(opt_params_.duration_, opt_params_.duration_);

            /// Start constraint
            opt.AddPathPositionConstraint(q0, q0, 0); // Linear constraint
            opt.AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
            /// Goal constraint
            opt.AddPathPositionConstraint(qF, qF, 1); // Linear constraint

            /// Cost
            auto c1 = prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                                 q0,opt.control_points().leftCols(1));
            auto c2 = prog.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                                 qF, opt.control_points().rightCols(1));

            if (traj1.result_.is_success())
            {
                opt.SetInitialGuess(opt.ReconstructTrajectory(traj1.result_));
            }

            /// Solve
            BSplineTraj traj;
            traj.result_ = drake::solvers::Solve(prog);
            traj.traj_ = opt.ReconstructTrajectory(traj1.result_);

            auto disc_traj = sampleTrajectory(traj);
            if (act->isFeasible(disc_traj))
            {

            }


            return traj;

        }


        int clearCosts()
        {
        }

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
        ABBParams robot_params_;
        BSplineOptParams opt_params_;

        /// @TODO (@ram): Rewrite Drake optimizer and make it highly reusable.
        /// Optimizer
//        std::shared_ptr<OptType> opt_;
//        drake::solvers::MathematicalProgram& prog_;
//        std::vector<CostType> costs_;
//        std::vector<ConstraintType> constraints_;

    };

}



#endif //BSPLINEOPT_HPP
