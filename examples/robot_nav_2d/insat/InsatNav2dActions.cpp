#include "InsatNav2dActions.hpp"

namespace ps
{
    Nav2D::Nav2D(ParamsType params,
                 std::vector<std::vector<int>> map) : opt_(OptType::InterpMode::LINEAR,
                                                   5e-1,
                                                  1e-1),
                                                           nav2d_map_("", params, map)
   {
   }

  bool Nav2D::isFeasible(TrajType &traj)
  {
    bool feas = true;
    for (int i=0; i<=traj.rows(); ++i)
    {
      double x = traj.col(i)(0);
      double y = traj.col(i)(1);

      if (!nav2d_map_.isValidCell(x, y))
      {
        feas = false;
        break;
      }
    }
    return feas;
  }

    Nav2D::TrajType Nav2D::optimize(const StateVarsType &s1, const StateVarsType &s2)
  {
    VecDf p1 = Eigen::Map<const VecDf, Eigen::Unaligned>(s1.data(), s1.size());
    VecDf p2 = Eigen::Map<const VecDf, Eigen::Unaligned>(s2.data(), s2.size());

    return opt_.optimize(p1, p2);
  }

  Nav2D::TrajType Nav2D::warmOptimize(const TrajType &t1, const TrajType &t2)
  {
    return opt_.warmOptimize(t1, t2);
  }

  double Nav2D::getCost(const TrajType &traj)
  {
    return opt_.calculateCost(traj);
  }

}