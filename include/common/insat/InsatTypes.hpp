//
// Created by gaussian on 2/17/23.
//

#ifndef INSATTYPES_HPP
#define INSATTYPES_HPP

// Drake
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/solvers/solve.h>

#include <common/EigenTypes.h>

namespace ps
{

    struct InsatParams
    {
        int lowD_dims_ = 6;
        int fullD_dims_ = 12;
        int aux_dims_ = 6;
    };

    struct BSplineTraj : public MatDf
    {
        typedef drake::solvers::MathematicalProgramResult OptResultType;

        inline long size() {return disc_traj_.size();}

        OptResultType result_;
        drake::trajectories::BsplineTrajectory<double> traj_;
        MatDf disc_traj_;
    };

}


#endif //INSATTYPES_HPP
