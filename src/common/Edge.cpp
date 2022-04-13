#include <iostream>
#include <common/Edge.hpp>

using namespace std;
using namespace ps;

size_t Edge::id_counter_ = 0;

void Edge::SetCost(double cost)
{
    lock_.lock();
    cost_ = cost;
    lock_.unlock();
}

double Edge::GetCost() const
{   
    double cost_local;
    lock_.lock();
    cost_local = cost_;
    lock_.unlock();
    return cost_local;
}

Edge::Edge(const Edge& other_edge)
{
    parent_state_ptr_ = other_edge.parent_state_ptr_;
    child_state_ptr_ = other_edge.child_state_ptr_;
    action_ptr_ = other_edge.action_ptr_;
    expansion_priority_ = other_edge.expansion_priority_;
    is_closed_ = other_edge.is_closed_.load();
    is_eval_ = other_edge.is_eval_.load();
    is_invalid_ = other_edge.is_invalid_.load();
    edge_id_ = other_edge.edge_id_;
    SetCost(other_edge.GetCost());
}

Edge& Edge::operator=(const Edge& other_edge)
{
    parent_state_ptr_ = other_edge.parent_state_ptr_;
    child_state_ptr_ = other_edge.child_state_ptr_;
    action_ptr_ = other_edge.action_ptr_;
    expansion_priority_ = other_edge.expansion_priority_;
    is_closed_ = other_edge.is_closed_.load();
    is_eval_ = other_edge.is_eval_.load();
    is_invalid_ = other_edge.is_invalid_.load();
    edge_id_ = other_edge.edge_id_;
    SetCost(other_edge.GetCost());
    return *this;
}            

bool Edge::operator==(const Edge& other_edge) const
{
    return ((parent_state_ptr_==other_edge.parent_state_ptr_)&&(action_ptr_==other_edge.action_ptr_));
}

void Edge::Print(std::string str)
{
    std::cout << "______________"<< str <<"_________________" << std::endl;

    if (action_ptr_)
        std::cout << "Edge: " << edge_id_ << " | Type: " << action_ptr_->GetType() << "| Cost: " << cost_ << "| expansion_priority: " << expansion_priority_<< std::endl;
    else
        std::cout << "Edge: " << edge_id_ << " | Type: NULL" << "| Cost: " << cost_ << "| expansion_priority: " << expansion_priority_<< std::endl;


    if (parent_state_ptr_)
        parent_state_ptr_->Print("Parent");
    if (child_state_ptr_)
        child_state_ptr_->Print("Child");
    std::cout << "_______________________________" << std::endl;
}

bool IsLesserEdge::operator() (const Edge& lhs, const Edge& rhs)
{
    // Default fifo ordering
    if (lhs.expansion_priority_ == rhs.expansion_priority_) // tie breaking
        return lhs.edge_id_ < rhs.edge_id_;
    else
        return lhs.expansion_priority_ < rhs.expansion_priority_;
}

bool IsGreaterEdge::operator() (const Edge& lhs, const Edge& rhs)
{
    // Default fifo ordering
    if (lhs.expansion_priority_ == rhs.expansion_priority_) // tie breaking
        return lhs.edge_id_ < rhs.edge_id_;
    else
        return lhs.expansion_priority_ > rhs.expansion_priority_;
}
