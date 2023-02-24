#ifndef PLANNER_HPP
#define PLANNER_HPP

#define VERBOSE 0

#include <functional>
#include <future>
#include <common/Types.hpp>
#include <common/Edge.hpp>

namespace ps
{

class Planner
{
    public:

        // Typedefs
        typedef std::unordered_map<size_t, StatePtrType> StatePtrMapType; 
        // Lower priority states will be in the front
        typedef smpl::intrusive_heap<State, IsLesserState> StateQueueMinType;
        typedef std::unordered_map<size_t, EdgePtrType> EdgePtrMapType;
        // Higher priority edge will be in the front
        typedef smpl::intrusive_heap<Edge, IsGreaterEdge> EdgeQueueMaxType;
        // Lower priority edge will be in the front
        typedef smpl::intrusive_heap<Edge, IsLesserEdge> EdgeQueueMinType;

        Planner()=delete;
        Planner(ParamsType planner_params);
        virtual ~Planner();
        
        virtual bool Plan() = 0;
        std::vector<PlanElement> GetPlan() const;
        PlannerStats GetStats() const;

        void SetActions(std::vector<std::shared_ptr<Action>> actions_ptrs);
        virtual void SetStartState(const StateVarsType& state_vars);
        virtual void SetGoalState(const StateVarsType& state_vars);
        void SetGoalChecker(std::function<bool(const StateVarsType&)> callback);

        void SetStateMapKeyGenerator(std::function<std::size_t(const StateVarsType&)> callback);
        void SetEdgeKeyGenerator(std::function<std::size_t(const EdgePtrType&)> callback);
        void SetHeuristicGenerator(std::function<double(const StateVarsType&)> callback);
        void SetStateToStateHeuristicGenerator(std::function<double(const StateVarsType&, const StateVarsType&)> callback);


    protected:
        
        virtual void initialize();
        void startTimer();
        bool checkTimeout();
        virtual void resetStates();
        StatePtrType constructState(const StateVarsType& state);
        size_t getEdgeKey(const EdgePtrType& edge_ptr);
        double computeHeuristic(const StatePtrType& state_ptr);
        double computeHeuristic(const StatePtrType& state_ptr_1, const StatePtrType& state_ptr_2);
        bool isGoalState(const StatePtrType& state_ptr);
        void constructPlan(StatePtrType& state_ptr);

        // Utilities
        template<typename T> bool isFutureReady(T& future){return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;};
        double roundOff(double value, int prec=3);
        void cleanUp();
        virtual void exit();

        std::unordered_map<std::string, double> planner_params_;
        std::vector<std::shared_ptr<Action>> actions_ptrs_;

        StatePtrMapType state_map_;
        EdgePtrMapType edge_map_;
        StatePtrType start_state_ptr_;
        StatePtrType goal_state_ptr_;
        double heuristic_w_;

        std::function<std::size_t(const StateVarsType&)> state_key_generator_;
        std::function<std::size_t(const EdgePtrType&)> edge_key_generator_;
        std::function<double(const StateVarsType&)> unary_heuristic_generator_;
        std::function<double(const StateVarsType&, const StateVarsType&)> binary_heuristic_generator_;
        std::function<double(const StateVarsType&)> goal_checker_;

        // Statistics
        std::vector<PlanElement> plan_;
        PlannerStats planner_stats_;
        double h_val_min_;

        std::chrono::time_point<std::chrono::steady_clock> t_start_;
};

}

#endif
