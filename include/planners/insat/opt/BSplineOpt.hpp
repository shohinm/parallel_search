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
            BSplineOptParams();

            BSplineOptParams(int num_positions, int num_control_points,
                             int spline_order, double min_duration, double max_duration);

            BSplineOptParams(int min_ctrl_points,
                             int max_ctrl_points,
                             VecDf& start,
                             VecDf& goal,
                             double total_duration);

            void setAdaptiveParams(int min_ctrl_points,
                                   int max_ctrl_points);

            int num_positions_;
            int num_control_points_;
            int spline_order_;
            double min_duration_;
            double max_duration_;

            double duration_cost_w_ = 1.0;
            double length_cost_w_ = 0.1;

            /// Adaptive BSpline optimization
            int min_ctrl_points_;
            int max_ctrl_points_;
            VecDf global_start_; /// For now assuming higher derivatives = 0
            VecDf global_goal_; /// For now assuming higher derivatives = 0
            double start_goal_dist_;


        };


        BSplineOpt(const InsatParams& insat_params,
                   const RobotParamsType& robot_params,
                   const BSplineOptParams& opt_params);

        void SetGoalChecker(std::function<bool(const StateVarsType&)> callback);

        void updateStartAndGoal(StateVarsType& start, StateVarsType& goal);

        bool isGoal(const VecDf& state) const;

        /// trajectory samplers with fixed time
        MatDf sampleTrajectory(const BSplineTraj::TrajInstanceType& traj, double dt) const;

        MatDf sampleTrajectory(const BSplineTraj& traj, double dt) const;

        /// trajectory samplers with adaptive time
        MatDf sampleTrajectory(const BSplineTraj::TrajInstanceType& traj) const;

        MatDf sampleTrajectory(const BSplineTraj& traj) const;

        void addDurationAndPathCost(OptType& opt) const;

        void addStateSpaceBounds(OptType& opt) const;

        void addDurationConstraint(OptType& opt) const;

        std::vector<BSplineTraj::TrajInstanceType> optimizeWithCallback(const OptType& opt,
                                                                        drake::solvers::MathematicalProgram& prog) const;

        BSplineTraj optimize(const InsatAction* act, const VecDf& s1, const VecDf& s2, int thread_id);

        BSplineTraj warmOptimize(const InsatAction* act, const TrajType& traj1, const TrajType & traj2, int thread_id);

        BSplineTraj warmOptimize(const InsatAction* act, const TrajType& traj, int thread_id);


        BSplineTraj runBSplineOpt(const InsatAction* act,
                                  const VecDf& q0, const VecDf& qF,
                                  VecDf& dq0, VecDf& dqF,
                                  int order, int num_ctrl_pt, double T,
                                  int thread_id);

        /// The assumption is both t1 and t2 exists and this function is only used for blending
        BSplineTraj runBSplineOptWithInitGuess(const InsatAction* act,
                                               const BSplineTraj& t1, BSplineTraj& t2,
                                               const VecDf& q0, const VecDf& qF,
                                               VecDf& dq0, VecDf& dqF,
                                               int order,
                                               int c1, int c2,
                                               double T,
                                               int thread_id);


        BSplineTraj optimize(const InsatAction* act,
                             const BSplineTraj& incoming_traj,
                             const VecDf& curr_state,
                             const VecDf& succ_state,
                             int thread_id);

        BSplineTraj optimize(const InsatAction* act,
                             const BSplineTraj& incoming_traj,
                             const std::vector<StateVarsType> &ancestors,
                             const StateVarsType& successor,
                             int thread_id);

        BSplineTraj optimizeWithWaypointConstraint(VecDf& st, VecDf& go, MatDf& wp, VecDf& s_wp) const;

        BSplineTraj optimizeWithWaypointConstraintAndCallback(const InsatAction *act,
                                                              VecDf& st, VecDf& go,
                                                              MatDf& wp, VecDf& s_wp,
                                                              const BSplineTraj& init_traj,
                                                              int thread_id) const;

        BSplineTraj fitBestBSpline(const InsatAction *act,
                                   MatDf &path, const BSplineTraj& init_traj,
                                   int thread_id) const;

        /// Post processing
        MatDf postProcess(std::vector<PlanElement>& path, double& cost, const InsatAction* act) const;

        virtual double calculateCost(const TrajType& traj) const;

        virtual double calculateCost(const MatDf& traj) const;

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
