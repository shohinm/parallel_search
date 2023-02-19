#ifndef RRT_PLANNER_HPP
#define RRT_PLANNER_HPP

#include <future>
#include <random>
#include <planners/Planner.hpp>

namespace ps
{

class RrtPlanner : public Planner
{
    public:
        RrtPlanner(ParamsType planner_params);
        ~RrtPlanner();
        void SetGoalState(const StateVarsType& state_vars);
        bool Plan();

    protected:
        void initialize();
        void rrtThread(int thread_id);
        double getRandomNumberBetween(double min, double max);
        StateVarsType sampleSateUniform();
        StateVarsType sampleState();
        double wrapAngle(double angle);
        double angleDifference(double angle1, double angle2);
        double calculateDistance(const StateVarsType& state_1, const StateVarsType& state_2);
        StatePtrType getNearestNeighbor(const StateVarsType& sampled_state);
        bool isValidConfiguration(const StateVarsType& state_vars, int thread_id);
        StatePtrType extend(const StatePtrType& nearest_neighbor, const StateVarsType& sampled_state, 
            bool& is_collision, int thread_id);
        void exit();

        int num_threads_;
        std::random_device random_device_;
        mutable LockType lock_;
        std::atomic<bool> terminate_;
        std::vector<std::future<void>> rrt_futures_;
};

}


#endif
