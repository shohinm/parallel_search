#ifndef INSAT_EDGE_HPP
#define INSAT_EDGE_HPP

#include <common/Edge.hpp>
#include <common/insat/InsatAction.hpp>

namespace ps
{
    class InsatEdge : public Edge
    {
    public:

        InsatEdge(InsatStatePtrType lowD_parent_ptr, InsatActionPtrType action_ptr, InsatStatePtrType fullD_parent_ptr = NULL,  InsatStatePtrType child_ptr = NULL):
                lowD_parent_state_ptr_(lowD_parent_ptr), lowD_child_state_ptr_(child_ptr),
                fullD_parent_state_ptr_(fullD_parent_ptr), fullD_child_state_ptr_(child_ptr),
                action_ptr_(action_ptr),
                Edge(lowD_parent_ptr, action_ptr, child_ptr)
                {edge_id_ = id_counter_++;};
                
        InsatEdge(const InsatEdge& other_edge);
        InsatEdge& operator=(const InsatEdge& other_edge);
        bool operator==(const InsatEdge& other_edge) const;

        void SetTraj(TrajType& traj) 
        {
            lock_.lock();
            traj_ = traj;
            lock_.unlock();
        };
        
        TrajType GetTraj() 
        { 
            lock_.lock();
            auto traj_local = traj_;
            lock_.unlock();
            return traj_local;
        };

        void SetTrajCost(double traj_cost) 
        {
            lock_.lock();
            traj_cost_ = traj_cost;
            lock_.unlock();
        };

        double GetTrajCost() 
        { 
            lock_.lock();
            auto traj_cost_local = traj_cost_;
            lock_.unlock();
            return traj_cost_local;
        };

        InsatStatePtrType lowD_parent_state_ptr_;
        InsatStatePtrType lowD_child_state_ptr_;
        InsatStatePtrType fullD_parent_state_ptr_;
        InsatStatePtrType fullD_child_state_ptr_;
        InsatActionPtrType action_ptr_;

    private:
        // Dynamic trajectory
        TrajType traj_;
        double traj_cost_;
    };
}

#endif