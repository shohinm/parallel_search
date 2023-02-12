#ifndef INSAT_EDGE_HPP
#define INSAT_EDGE_HPP

#include <common/Edge.hpp>
#include <common/insat/InsatAction.hpp>

namespace ps
{
    class InsatEdge : public Edge
    {
    public:

        typedef MatDf TrajType;

        InsatEdge(InsatStatePtrType lowD_parent_ptr, InsatActionPtrType action_ptr, InsatStatePtrType fullD_parent_ptr = NULL,  InsatStatePtrType child_ptr = NULL):
                lowD_parent_state_ptr_(lowD_parent_ptr), lowD_child_state_ptr_(child_ptr),
                fullD_parent_state_ptr_(fullD_parent_ptr), fullD_child_state_ptr_(child_ptr),
                action_ptr_(action_ptr),
                Edge(lowD_parent_ptr, action_ptr, child_ptr)
                {edge_id_ = id_counter_++;};
                
        InsatEdge(const InsatEdge& other_edge);
        InsatEdge& operator=(const InsatEdge& other_edge);
        bool operator==(const InsatEdge& other_edge) const;

        void SetTraj(TrajType& traj) {traj_ = traj;};
        TrajType GetTraj() { return traj_;};
        void SetTrajCost(double traj_cost) {traj_cost_ = traj_cost;};
        double GetTrajCost() { return traj_cost_;};

        InsatStatePtrType lowD_parent_state_ptr_;
        InsatStatePtrType lowD_child_state_ptr_;
        InsatStatePtrType fullD_parent_state_ptr_;
        InsatStatePtrType fullD_child_state_ptr_;
        InsatActionPtrType action_ptr_;

        // Dynamic trajectory
        TrajType traj_;
        double traj_cost_;
    };
}

#endif