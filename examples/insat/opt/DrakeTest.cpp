#include <drake/systems/trajectory_optimization/kinematic_trajectory_optimization.h>

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

