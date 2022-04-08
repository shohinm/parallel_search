#include <functional>
#include "Types.hpp"
#include "Edge.hpp"

namespace epase
{

class Planner
{
    public:

        typedef std::unordered_map<size_t, StatePtrType> StatePtrMapType; 
        // Lower priority states will be in the front
        typedef smpl::intrusive_heap<State, IsLesserState> StateQueueMinType;
        typedef std::unordered_map<size_t, EdgePtrType> EdgePtrMapType;
        // Higher priority edge will be in the front
        typedef smpl::intrusive_heap<Edge, IsGreaterEdge> EdgeQueueMaxType;
        // Lower priority edge will be in the front
        typedef smpl::intrusive_heap<Edge, IsLesserEdge> EdgeQueueMinType;

        Planner();
        virtual ~Planner();
        
        virtual bool Plan() = 0;

        void SetStartState(const StateVarsType& state_vars);
        void SetGoalChecker(std::function<double(const StatePtrType)> callback);

        void SetStateMapKeyGenerator(std::function<std::size_t(const StateVarsType&)> callback);
        void SetEdgeKeyGenerator(std::function<std::size_t(const EdgePtrType)> callback);
        void SetHeuristicGenerator(std::function<double(const StatePtrType)> callback);
        void SetStateToStateHeuristicGenerator(std::function<double(const StatePtrType, const StatePtrType)> callback);


    protected:
        virtual void initialize();
        StatePtrType constructState(const StateVarsType& state);
        size_t getEdgeKey(const EdgePtrType& edge_ptr);
        double computeHeuristic(const StatePtrType state_ptr);
        double computeHeuristic(const StatePtrType state_ptr_1, const StatePtrType state_ptr_2);
        double roundOff(double value, int prec=3);
        void cleanUp();

        StatePtrMapType state_map_;
        EdgePtrMapType edge_map_;
        StatePtrType start_state_ptr_;
        double heuristic_w_;

        std::function<std::size_t(const StateVarsType&)> state_key_generator_;
        std::function<std::size_t(const EdgePtrType)> edge_key_generator_;
        std::function<double(const StatePtrType)> unary_heuristic_generator_;
        std::function<double(const StatePtrType, const StatePtrType)> binary_heuristic_generator_;


 
};

}