#ifndef EDGE_HPP
#define EDGE_HPP

#include <thread>
#include <mutex>
#include <atomic>
#include <common/Types.hpp>
#include <common/Action.hpp>

namespace ps
{

class Edge : public smpl::heap_element
{
    public:
        // Edge(){};
        Edge(StatePtrType parent_ptr, ActionPtrType action_ptr, StatePtrType child_ptr = NULL): 
        parent_state_ptr_(parent_ptr), child_state_ptr_(child_ptr), action_ptr_(action_ptr),
        is_closed_(false), is_eval_(false), is_invalid_(false),
        expansion_priority_(-1), evaluation_priority_(-1),
        cost_(-1)
        {edge_id_ = id_counter_++;};
        // Edge(StatePtrType parent_ptr, ActionPtrType action_ptr): 
        // parent_state_ptr_(parent_ptr), child_state_ptr_(NULL), action_ptr_(action_ptr),
        // is_closed_(false), is_eval_(false), is_invalid_(false),
        // expansion_priority_(-1), evaluation_priority_(-1),
        // cost_(-1)
        // {edge_id_ = id_counter_++;};
        Edge(const Edge& other_edge);
        virtual Edge& operator=(const Edge& other_edge);
        virtual bool operator==(const Edge& other_edge) const;
        
        void SetCost(double cost);
        double GetCost() const;

        void Print(std::string str="");
        static void ResetStateIDCounter(){id_counter_=0;};

        static std::size_t id_counter_;
        std::size_t edge_id_;

        StatePtrType parent_state_ptr_;
        StatePtrType child_state_ptr_;
        ActionPtrType action_ptr_;

        double expansion_priority_;
        double evaluation_priority_;

        std::atomic<bool> is_closed_;
        std::atomic<bool> is_eval_;
        std::atomic<bool> is_invalid_;
 
    private:
        double cost_;
        mutable std::mutex lock_; 
        double roundOff(double value, int prec=3);
};

class IsLesserEdge
{
    public:
        bool operator() (const Edge& lhs, const Edge& rhs);
};

class IsGreaterEdge
{
    public:
        bool operator() (const Edge& lhs, const Edge& rhs);
};

}

#endif
