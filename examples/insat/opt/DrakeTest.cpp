// PS
#include <common/Types.hpp>

// Drake
#include <drake/systems/trajectory_optimization/kinematic_trajectory_optimization.h>

namespace ps
{
    struct ABBParams
    {
        ABBParams()
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
        typedef drake::systems::trajectory_optimization::KinematicTrajectoryOptimization OptType;
        typedef drake::solvers::Binding<drake::solvers::Cost> CostType;
        typedef drake::solvers::Binding<drake::solvers::Constraint> ConstraintType;

        BSplineOpt(int num_positions, int num_control_points,
                   int spline_order = 4, double duration = 1.0)
        {
            opt_ = std::make_shared<OptType>(num_positions, num_control_points, spline_order, duration);

        }

        int clearCosts()
        {

        }

        int clearConstraints()
        {

        }

        /// Optimizer
        std::shared_ptr<OptType> opt_;
        drake::solvers::MathematicalProgram prog_;

        std::vector<CostType> costs_;
        std::vector<ConstraintType> constraints_;
    };

}


int main()
{
    using namespace drake::systems::trajectory_optimization;

    KinematicTrajectoryOptimization trajopt(2, 10);
    // add costs and constraints
    trajopt.SetInitialGuess(...);
    auto result = drake::solvers::Solve(trajopt.prog());
    auto traj = trajopt.ReconstructTrajectory(result);

    return 0;
}

