// PS
#include <common/Types.hpp>

// Drake
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/solvers/solve.h>

namespace ps_drake
{
    struct InsatParams
    {
        int lowD_dims_ = 6;
        int fullD_dims_ = 12;
        int aux_dims_ = 6;
    };

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

    /// Highly reusable Drake Kinematic Trajectory Optimizer
    using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;
    class DrakeOpt : public KinematicTrajectoryOptimization
    {
        typedef drake::solvers::Binding<drake::solvers::Cost> CostType;
        typedef std::vector<CostType> CostVecType;
        typedef drake::solvers::Binding<drake::solvers::Constraint> ConstraintType;
        typedef std::vector<ConstraintType> ConstraintVecType;

        DrakeOpt(int num_positions, int num_control_points,
                 int spline_order = 4, double duration = 1.0) : KinematicTrajectoryOptimization(num_positions,
                                                                                                num_control_points,
                                                                                                spline_order,
                                                                                                duration),
                                                                                                prog_(this->get_mutable_prog())
        {}

        CostType AddDurationCost(double weight=1.0)
        {
            return prog_.AddLinearCost(weight * this->duration());
        }

        ConstraintType AddDurationConstraint(std::optional<double> lb, std::optional<double> ub)
        {
            return prog_.AddBoundingBoxConstraint(
                    lb.value_or(0), ub.value_or(std::numeric_limits<double>::infinity()),
                    this->duration());
        }


//        ConstraintVecType AddPositionBounds(
//                const Eigen::Ref<const VecDf>& lb,
//                const Eigen::Ref<const VecDf>& ub) {
//            DRAKE_DEMAND(lb.size() == num_positions());
//            DRAKE_DEMAND(ub.size() == num_positions());
//            // This leverages the convex hull property of the B-splines: if all of the
//            // control points satisfy these convex constraints and the curve is inside
//            // the convex hull of these constraints, then the curve satisfies the
//            // constraints for all s (and therefore all t).
//            ConstraintVecType constraints;
//            for (int i = 0; i < num_control_points(); ++i) {
//                auto cn = prog_.AddBoundingBoxConstraint(lb, ub, this->control_points().col(i));
//                constraints.emplace_back(cn);
//            }
//            return constraints;
//        }
//
//        ConstraintVecType AddVelocityBounds(
//                const Eigen::Ref<const VecDf>& lb,
//                const Eigen::Ref<const VecDf>& ub) {
//            DRAKE_DEMAND(lb.size() == num_positions());
//            DRAKE_DEMAND(ub.size() == num_positions());
//
//            // We have q̇(t) = drds * dsdt = ṙ(s) / duration, and duration >= 0, so we
//            // use duration * lb <= ṙ(s) <= duration * ub.
//            //
//            // This also leverages the convex hull property of the B-splines: if all of
//            // the control points satisfy these convex constraints and the curve is
//            // inside the convex hull of these constraints, then the curve satisfies the
//            // constraints for all t.
//            for (int i = 0; i < sym_rdot_->num_control_points(); ++i) {
//                prog_.AddLinearConstraint(sym_rdot_->control_points()[i] >=
//                                          duration_ * lb &&
//                                          sym_rdot_->control_points()[i] <= duration_ * ub);
//            }
//        }


        drake::solvers::MathematicalProgram& prog_;

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

        struct Result
        {

        };

        BSplineOpt(int num_positions, int num_control_points,
                   int spline_order = 4, double duration = 1.0) : opt_(std::make_shared<OptType>(num_positions,
                                                                                                 num_control_points,
                                                                                                 spline_order,
                                                                                                 duration)),
                                                                                                 prog_(opt_->get_mutable_prog())
        {
            opt_->AddDurationCost(1.0);
            opt_->AddPathLengthCost(1.0);

            opt_->AddPositionBounds(robot_params_.min_q_, robot_params_.max_q_);
            opt_->AddVelocityBounds(robot_params_.min_dq_, robot_params_.max_dq_);

            opt_->AddDurationConstraint(2, 2);
        }

        OptResult optimize(const VecDf& s1, const VecDf& s2)
        {
            const VecDf& q0 = s1.topRows(insat_params_.lowD_dims_);
            const VecDf& qF = s2.topRows(insat_params_.lowD_dims_);
            const VecDf& dq0 = s1.bottomRows(insat_params_.aux_dims_);
            const VecDf& dqF = s2.bottomRows(insat_params_.aux_dims_);

            /// Start constraint
            opt_->AddPathPositionConstraint(q0, q0, 0); // Linear constraint
            opt_->AddPathVelocityConstraint(dq0, dq0, 0); // Linear constraint
            /// Goal constraint
            opt_->AddPathPositionConstraint(qF, qF, 1); // Linear constraint
            opt_->AddPathVelocityConstraint(dqF, dqF, 1); // Linear constraint

            /// Cost
            auto c1 = prog_.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                        q0,opt_->control_points().leftCols(1));
            auto c2 = prog_.AddQuadraticErrorCost(MatDf::Identity(insat_params_.lowD_dims_, insat_params_.lowD_dims_),
                                        qF, opt_->control_points().rightCols(1));

            costs_.emplace_back(c1);
            costs_.emplace_back(c2);


            /// Solve
            return drake::solvers::Solve(prog_);
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

        int clearConstraints()
        {
            auto lin_constraints = prog_.linear_constraints();
            for (auto& cn : lin_constraints)
            {
                prog_.RemoveConstraint(cn);
            }
        }

        /// Optimizer
        std::shared_ptr<OptType> opt_;
        drake::solvers::MathematicalProgram& prog_;

        InsatParams insat_params_;
        ABBParams robot_params_;

        std::vector<CostType> costs_;
        std::vector<ConstraintType> constraints_;
    };

}

#include <random>
#include <chrono>
#include <numeric>
#include <iostream>

VecDf genRandomVector(VecDf& low, VecDf& high, int size)
{
    VecDf range = high-low;
    VecDf randvec = VecDf::Random(size);
    randvec += VecDf::Constant(size, 1, 1.0);
    randvec /= 2.0;
    randvec = randvec.cwiseProduct(range);
    randvec += low;

    return randvec;
}

void printTraj(drake::trajectories::BsplineTrajectory<double> traj, double dt)
{
    for (double t=0.0; t<=traj.end_time(); t+=dt)
    {
        std::cout << "t: " << t << "\ts: " << traj.value(t).transpose() << std::endl;
    }

    std::cout << "num control pts: " << traj.control_points().size() << std::endl;
    std::cout << "control pt: " << traj.control_points()[0].transpose() << std::endl;
    std::cout << "num_basis_functions: " << traj.basis().num_basis_functions() << std::endl;
}

void runBVPTest(std::vector<double>& time_log, int test_size=1)
{
    typedef std::chrono::high_resolution_clock Clock;
    using namespace std::chrono;

    ps_drake::InsatParams insat_params;
    ps_drake::ABBParams robot_params;

    for (int i=0; i<test_size; ++i)
    {
        ps_drake::BSplineOpt opt(insat_params.lowD_dims_, 7);
        VecDf st(insat_params.fullD_dims_), go(insat_params.fullD_dims_);
        st.setZero();
        go.setZero();
        VecDf r1 = genRandomVector(robot_params.min_q_, robot_params.max_q_, insat_params.lowD_dims_);
        VecDf r2 = genRandomVector(robot_params.min_q_, robot_params.max_q_, insat_params.lowD_dims_);
        st.topRows(insat_params.lowD_dims_) = r1;
        go.topRows(insat_params.lowD_dims_) = r2;

        std::cout << "---- TRIAL " << i+1 << "------" << "\nst: " << r1.transpose() << "\ngo: " << r2.transpose() << std::endl;

        auto start_time = Clock::now();
        auto result = opt.optimize(st, go);
        auto end_time = Clock::now();
        if (result.is_success())
        {
            std::cout << "SUCCESS" << std::endl;
            double time_elapsed = duration_cast<duration<double> >(end_time - start_time).count();
            time_log.emplace_back(time_elapsed);

            printTraj(opt.opt_->ReconstructTrajectory(result), 1e-1);
//            opt.opt_->SetInitialGuess(opt.opt_->ReconstructTrajectory())
        }
    }
}

void runBVPwithWPTest(std::vector<double>& time_log, int test_size=1)
{
    typedef std::chrono::high_resolution_clock Clock;
    using namespace std::chrono;

    ps_drake::InsatParams insat_params;
    ps_drake::ABBParams robot_params;

    for (int i=0; i<test_size; ++i)
    {
        ps_drake::BSplineOpt opt(insat_params.lowD_dims_, 10);
        VecDf st(insat_params.fullD_dims_), go(insat_params.fullD_dims_), wp(insat_params.fullD_dims_);
        st.setZero();
        wp.setZero();
        go.setZero();
        VecDf r1 = genRandomVector(robot_params.min_q_, robot_params.max_q_, insat_params.lowD_dims_);
        VecDf r2 = genRandomVector(robot_params.min_q_, robot_params.max_q_, insat_params.lowD_dims_);
        VecDf r3 = genRandomVector(robot_params.min_q_, robot_params.max_q_, insat_params.lowD_dims_);
        st.topRows(insat_params.lowD_dims_) = r1;
        wp.topRows(insat_params.lowD_dims_) = r2;
        go.topRows(insat_params.lowD_dims_) = r3;

        std::cout << "---- TRIAL " << i+1 << "------"
        << "\nst: " << r1.transpose()
        << "\nwp: " << r2.transpose()
        << "\ngo: " << r3.transpose() << std::endl;

        auto start_time = Clock::now();
        auto result = opt.optimize(st, go);
        auto end_time = Clock::now();
        if (result.is_success())
        {
            std::cout << "SUCCESS" << std::endl;
            double time_elapsed = duration_cast<duration<double> >(end_time - start_time).count();
            time_log.emplace_back(time_elapsed);

            printTraj(opt.opt_->ReconstructTrajectory(result), 1e-1);
//            opt.opt_->SetInitialGuess(opt.opt_->ReconstructTrajectory())
        }
    }
}

int main()
{
    typedef std::chrono::high_resolution_clock Clock;
    using namespace std::chrono;
    using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;

    ps_drake::InsatParams insat_params;
    ps_drake::ABBParams robot_params;

    int test_size = 1;

    std::vector<double> time_log;
    std::vector<double> init_time_log;

    runBVPTest(time_log, test_size);

    double mean = accumulate( time_log.begin(), time_log.end(), 0.0)/time_log.size();
    for (auto i: time_log)
        std::cout << i << ' ';
    std::cout << std::endl;
    std::cout << "Average: " << mean << std::endl;


    return 0;
}

