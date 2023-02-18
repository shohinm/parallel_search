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
                             int spline_order, double duration) : num_positions_(num_positions),
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

        BSplineTraj optimize(const InsatAction* act, const VecDf& s1, const VecDf& s2)
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

        void optimize(const VecDf& s1, const VecDf& s2, int N)
        {
        }

//        void warmOptimize(const InsatAction* act, const TrajType& traj1, const TrajType & traj2)
//        {
//            int N = traj1.cols()+traj2.cols();
//            TrajType init_traj(traj1.rows(), N);
//            TrajType opt_traj(traj1.rows(), N);
//            TrajType soln_traj(traj1.rows(), N);
//
//            init_traj << traj1, traj2;
//            opt_traj = linInterp(traj1.leftCols(1), traj2.rightCols(1), N);
//
//            for (double i=0.0; i<=1.0; i+=1.0/conv_delta_)
//            {
//                soln_traj = (1-i)*init_traj + i*opt_traj;
//                if (!act->isFeasible(soln_traj))
//                {
//                    break;
//                }
//            }
//            return soln_traj;
//        }


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
