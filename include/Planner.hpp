#include <functional>
#include <future>
#include "Types.hpp"
#include "Edge.hpp"

namespace epase
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

        Planner(ParamsType planner_params);
        virtual ~Planner();
        
        virtual bool Plan(int exp_idx = 1) = 0;
        std::vector<PlanElement> GetPlan() const;
        virtual bool PrintStats(int exp_idx);

        void SetActions(std::vector<std::shared_ptr<Action>> actions_ptrs);
        void SetStartState(const StateVarsType& state_vars);
        void SetGoalChecker(std::function<double(const StatePtrType)> callback);

        void SetStateMapKeyGenerator(std::function<std::size_t(const StateVarsType&)> callback);
        void SetEdgeKeyGenerator(std::function<std::size_t(const EdgePtrType&)> callback);
        void SetHeuristicGenerator(std::function<double(const StatePtrType)> callback);
        void SetStateToStateHeuristicGenerator(std::function<double(const StatePtrType, const StatePtrType)> callback);


    protected:
        virtual void initialize();
        void resetStates();
        StatePtrType constructState(const StateVarsType& state);
        size_t getEdgeKey(const EdgePtrType& edge_ptr);
        double computeHeuristic(const StatePtrType state_ptr);
        double computeHeuristic(const StatePtrType state_ptr_1, const StatePtrType state_ptr_2);
        bool isGoalState(StatePtrType state);
        void constructPlan(StatePtrType state);

        // Utilities
        template<typename T> bool isFutureReady(T& future){return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;};
        double roundOff(double value, int prec=3);
        void cleanUp();

        std::vector<std::shared_ptr<Action>> actions_ptrs_;

        StatePtrMapType state_map_;
        EdgePtrMapType edge_map_;
        StatePtrType start_state_ptr_;
        StatePtrType goal_state_ptr_;
        double heuristic_w_;

        std::function<std::size_t(const StateVarsType&)> state_key_generator_;
        std::function<std::size_t(const EdgePtrType&)> edge_key_generator_;
        std::function<double(const StatePtrType)> unary_heuristic_generator_;
        std::function<double(const StatePtrType, const StatePtrType)> binary_heuristic_generator_;

        // Statistics
        std::vector<PlanElement> plan_;
        int num_evaluated_edges_;
        int num_state_expansions_;   
        double total_time_;
        double solution_cost_;
};

}