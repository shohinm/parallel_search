#include <Edge.hpp>

using namespace std;
using namespace epase;


void Edge::SetCost(double cost)
{
    lock_.lock();
    cost_ = cost;
    lock_.unlock();
}

double Edge::GetCost()
{   
    double cost_local;
    lock_.lock();
    cost_local = cost_;
    lock_.unlock();
    return cost_local;
}

Edge::Edge(const Edge& other_edge)
{
    parent_ = other_edge.parent_;
    child_ = other_edge.child_;
    action_ = other_edge.action_;
    expansion_priority_ = other_edge.expansion_priority_;
    is_closed_ = other_edge.is_closed_.load();
    is_eval_ = other_edge.is_eval_.load();
    is_invalid_ = other_edge.is_invalid_.load();
    edge_id_ = other_edge.edge_id_;
}

Edge& Edge::operator=(const Edge& other_edge)
{
    parent_ = other_edge.parent_;
    child_ = other_edge.child_;
    action_ = other_edge.action_;
    expansion_priority_ = other_edge.expansion_priority_;
    is_closed_ = other_edge.is_closed_.load();
    is_eval_ = other_edge.is_eval_.load();
    is_invalid_ = other_edge.is_invalid_.load();
    edge_id_ = other_edge.edge_id_;
    return *this;
}            

bool Edge::operator==(const Edge& other_edge) const
{
    return ((parent_==other_edge.parent_)&&(action_==other_edge.action_));
}

void Print(std::string str="")
{
    // std::cout << "______________"<< str <<"_________________" << std::endl;

    // if (gac_.controller_)
    //     std::cout << "Edge: " << edge_id_ << " | type: " << gac_.controller_->GetType() << "| Cost: " << GetCost() << "| Frozen cost: " << GetCostFrozen() << "| eval_priority: " << eval_priority_ << "| exp_priority: " << exp_priority_<< std::endl;
    // else
    //     std::cout << "Edge: " << edge_id_ << " | type: NULL" << "| Cost: " << GetCost() << "| Frozen cost: " << GetCostFrozen() << "| eval_priority: " << eval_priority_ << "| exp_priority: " << exp_priority_<< std::endl;


    // if (parent_)
    //     parent_->Print("Parent");
    // if (child_)
    //     child_->Print("Child");
    // std::cout << "_______________________________" << std::endl;
}
