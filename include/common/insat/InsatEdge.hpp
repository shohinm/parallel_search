#ifndef INSAT_EDGE_HPP
#define INSAT_EDGE_HPP

#include <common/Edge.hpp>

namespace ps
{
    class InsatEdge : public Edge
    {
    public:

        typedef MatDf TrajType;

        InsatEdge(InsatStatePtrType parent_ptr, InsatStatePtrType child_ptr, ActionPtrType action_ptr):
                parent_state_ptr_(parent_ptr), child_state_ptr_(child_ptr),
                Edge(parent_ptr, child_ptr, action_ptr)
                {edge_id_ = id_counter_++;};

        InsatEdge(InsatStatePtrType parent_ptr, ActionPtrType action_ptr):
                parent_state_ptr_(parent_ptr),
                Edge(parent_ptr, action_ptr)
                {edge_id_ = id_counter_++;};
        InsatEdge(const InsatEdge& other_edge);
        InsatEdge& operator=(const InsatEdge& other_edge);
        bool operator==(const InsatEdge& other_edge) const;

        void SetTraj(TrajType& traj) {traj_ = traj;};
        TrajType GetTraj() { return traj_;};


        // INSAT edge
        InsatStatePtrType parent_state_ptr_;
        InsatStatePtrType child_state_ptr_;

        // INSAT edge
        TrajType traj_;
    };
}

#endif