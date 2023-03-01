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
        virtual void SetGoalState(const StateVarsType& state_vars);
        virtual bool Plan();

    protected:
        void initialize();
        double getCost(const StateVarsType& current_state, const StateVarsType& successor_state, int thread_id);
        StatePtrType constructState(const StateVarsType& state, StatePtrMapType& state_map);
        EdgePtrType addEdge(StatePtrType parent_state, StatePtrType child_state, EdgePtrMapType& edge_map);
        virtual void rrtThread(int thread_id);
        double getRandomNumberBetween(double min, double max);
        StateVarsType sampleFeasibleState(int thread_id);
        StateVarsType sampleState(StatePtrType goal_state_ptr, int thread_id, double goal_prob);
        double wrapAngle(double angle);
        double angleDifference(double angle1, double angle2);
        double calculateDistance(const StateVarsType& state_1, const StateVarsType& state_2);
        StatePtrType getNearestNeighbor(const StateVarsType& sampled_state, const StatePtrMapType& state_map);
        bool isValidConfiguration(const StateVarsType& state_vars, int thread_id);
        StateVarsType collisionFree(const StateVarsType& state_vars_start,
            const StateVarsType& state_vars_end, const StatePtrMapType& state_map, 
            bool& is_collision, int thread_id);
        StatePtrType extend(const StatePtrType& nearest_neighbor, const StateVarsType& sampled_state, 
            bool& is_collision, StatePtrMapType& state_map, int thread_id);
        void exitThreads();
        void exit();

        void PrintStateVars(const StateVarsType& state_vars, const std::string& prefix="")
        {
            if (prefix.size())
                std::cout << prefix << " ";

            for (auto& s : state_vars)
                std::cout << s << " ";
            std::cout << std::endl;
        }

        std::random_device random_device_;
        mutable LockType lock_;
        size_t state_key_ = 0;
        StateVarsType goal_state_vars_;
        std::atomic<bool> terminate_;
        std::atomic<bool> plan_found_;
        std::vector<std::future<void>> rrt_futures_;

        StatePtrType start_rrt_state_;
        StatePtrType goal_rrt_state_;

};

}


#endif
