#include <common/insat/InsatEdge.hpp>

namespace ps
{
    InsatEdge::InsatEdge(const InsatEdge& other_edge) : Edge(other_edge)
    {
        parent_state_ptr_ = other_edge.parent_state_ptr_;
        child_state_ptr_ = other_edge.child_state_ptr_;
        lowD_parent_state_ptr_ = other_edge.lowD_parent_state_ptr_;
        lowD_child_state_ptr_ = other_edge.lowD_child_state_ptr_;
        fullD_parent_state_ptr_ = other_edge.fullD_parent_state_ptr_;
        fullD_child_state_ptr_ = other_edge.fullD_child_state_ptr_;
        traj_ = other_edge.traj_;
        traj_cost_ = other_edge.traj_cost_;
    }

    InsatEdge& InsatEdge::operator=(const InsatEdge& other_edge)
    {
        parent_state_ptr_ = other_edge.parent_state_ptr_;
        child_state_ptr_ = other_edge.child_state_ptr_;
        lowD_parent_state_ptr_ = other_edge.lowD_parent_state_ptr_;
        lowD_child_state_ptr_ = other_edge.lowD_child_state_ptr_;
        fullD_parent_state_ptr_ = other_edge.fullD_parent_state_ptr_;
        fullD_child_state_ptr_ = other_edge.fullD_child_state_ptr_;
        traj_ = other_edge.traj_;
        traj_cost_ = other_edge.traj_cost_;

        Edge::operator=(other_edge);
        return *this;
    }

    bool InsatEdge::operator==(const InsatEdge& other_edge) const
    {
        return ((parent_state_ptr_==other_edge.parent_state_ptr_)&&(action_ptr_==other_edge.action_ptr_));
    }

}