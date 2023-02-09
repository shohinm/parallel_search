#ifndef INSAT_STATE_HPP
#define INSAT_STATE_HPP

#include <common/State.hpp>

namespace ps
{
  class InsatState : public State
  {
  public:

    typedef MatDf TrajType;

    InsatState(const StateVarsType& vars=StateVarsType()) : State(vars) {}
    ~InsatState() {};

    void SetInsatEdge(TrajType& insat_edge) {insat_edge_ = insat_edge;};
    TrajType GetInsatEdge() {return insat_edge_;};


    // INSAT edge
    TrajType insat_edge_;

  };
}

#endif