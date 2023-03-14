#ifndef INSAT_STATE_HPP
#define INSAT_STATE_HPP

#include <common/State.hpp>

namespace ps
{
  class InsatState : public State
  {
  public:

    InsatState(const StateVarsType& vars=StateVarsType()) : State(vars), incoming_edge_ptr_(nullptr) {}
    ~InsatState() {};

    void SetIncomingInsatEdgePtr(InsatEdgePtrType& incoming_edge_ptr) { incoming_edge_ptr_ = incoming_edge_ptr;};
    InsatEdgePtrType GetIncomingInsatEdgePtr() {return incoming_edge_ptr_;};
    void ResetIncomingInsatEdgePtr() { incoming_edge_ptr_ = NULL;};
    void SetAncestors(const std::vector<InsatStatePtrType>& ancestors) {ancestors_ = ancestors;};
    std::vector<InsatStatePtrType> GetAncestors() const {return ancestors_;};

  protected:
    InsatEdgePtrType incoming_edge_ptr_;
    std::vector<InsatStatePtrType> ancestors_;

  };
}

#endif