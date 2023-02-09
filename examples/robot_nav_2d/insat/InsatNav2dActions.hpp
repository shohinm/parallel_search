#ifndef INSAT_NAV2D_ACTION_HPP
#define INSAT_NAV2D_ACTION_HPP

#include <common/Action.hpp>
#include <common/typedefs.h>
#include <planners/insat/opt/dummy_opt.hpp>
#include "../RobotNav2dActions.hpp"

namespace ps
{

    class Nav2D
    {

    public:

      typedef std::shared_ptr<Nav2D> Ptr;
      typedef MatDf TrajType;
      typedef DummyOpt<Nav2D> OptType;

      Nav2D(ParamsType params,
            std::vector<std::vector<int>> map);
      bool isFeasible(TrajType& traj);
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2);
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2);
      double getCost(const TrajType& traj);

        OptType opt_;

    protected:
        RobotNav2dAction nav2d_map_;
    };


}

#endif
