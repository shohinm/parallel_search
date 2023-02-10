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

    void SetIncomingEdgePtr(InsatEdgePtrType& incoming_edge_ptr) {incoming_edge_ptr_ = incoming_edge_ptr;};
    InsatEdgePtrType GetIncomingEdgePtr() {return incoming_edge_ptr_;};
    void ResetIncomingEdgePtr() {incoming_edge_ptr_ = NULL;};

  protected:
    InsatEdgePtrType incoming_edge_ptr_;

  };
}

#endif