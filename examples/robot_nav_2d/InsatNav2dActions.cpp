#include "InsatNav2dActions.hpp"

namespace ps
{
  bool InsatNav2dAction::isFeasible(TrajType &traj)
  {
    bool feas = true;
    for (int i=0; i<=traj.rows(); ++i)
    {
      double x = traj.col(i)(0);
      double y = traj.col(i)(1);

      if (!isValidCell(x, y))
      {
        feas = false;
        break;
      }
    }
    return feas;
  }

  TrajType InsatNav2dAction::optimize(StateVarsType &s1, StateVarsType &s2)
  {
    VecDf p1 = Eigen::Map<VecDf, Eigen::Unaligned>(s1.data(), s1.size());
    VecDf p2 = Eigen::Map<VecDf, Eigen::Unaligned>(s2.data(), s2.size());

    return opt_.optimize(p1, p2);
  }

  TrajType InsatNav2dAction::warmOptimize(TrajType &t1, TrajType &t2)
  {
    return opt_.warmOptimize(t1, t2);
  }

  double InsatNav2dAction::getCost(TrajType &traj)
  {
    return opt_.calculateCost(traj);
  }

}