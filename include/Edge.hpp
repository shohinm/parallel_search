#ifndef EDGE_HPP
#define EDGE_HPP

#include <thread>
#include <mutex>
#include <atomic>
#include "Types.hpp"
#include "Action.hpp"

namespace epase
{

class Edge : public smpl::heap_element
{
    public:
        Edge(){};
        Edge(StatePtrType parent, StatePtrType child, Action action): 
        parent_(parent), child_(child), action_(action),
        is_closed_(false), is_eval_(false), is_invalid_(false)
        {edge_id_ = id_counter_++;};
        Edge(StatePtrType parent, Action action): 
        parent_(parent), action_(action),
        is_closed_(false), is_eval_(false), is_invalid_(false), child_(NULL)
        {edge_id_ = id_counter_++;};
        Edge(const Edge& other_edge);
        Edge& operator=(const Edge& other_edge);
        bool operator==(const Edge& other_edge) const;
        
        void SetCost(double cost);
        double GetCost() const;

        void Print(std::string str="");
        static void ResetStateIDCounter(){id_counter_=0;};

        static std::size_t id_counter_;
        std::size_t edge_id_;

        StatePtrType parent_;
        StatePtrType child_;
        Action action_;

        double expansion_priority_;
        std::atomic<bool> is_closed_;
        std::atomic<bool> is_eval_;
        std::atomic<bool> is_invalid_;
 
    private:
        double cost_;
        mutable std::mutex lock_; 

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