#ifndef INSAT_NAV2D_ACTION_HPP
#define INSAT_NAV2D_ACTION_HPP

#include <common/Action.hpp>
#include <common/typedefs.h>
#include <planners/insat/opt/dummy_opt.hpp>
#include "RobotNav2dActions.hpp"

namespace ps
{

    class InsatNav2dAction : public RobotNav2dAction
    {

    public:

      typedef std::shared_ptr<InsatNav2dAction> Ptr;
      typedef MatDf TrajType;

      InsatNav2dAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, bool is_expensive = true)
                : RobotNav2dAction(type, params, map, is_expensive){};
      bool isFeasible(TrajType& traj);
      TrajType optimize(StateVarsType& s1, StateVarsType& s2);
      TrajType warmOptimize(TrajType& t1, TrajType& t2);
      double getCost(TrajType& traj);


    protected:

      TrajType traj_;
      DummyOpt<InsatNav2dAction> opt_;

    };


}

#endif
