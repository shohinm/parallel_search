#include <iostream>
#include <cmath>
#include <sbpl/utils/utils.h>
#include "InsatNav2dActions.hpp"

namespace ps
{
    InsatNav2dAction::InsatNav2dAction(const std::string& type,
                                       ParamsType params,
                                       std::vector<std::vector<int>>& map,
                                       OptVecPtrType& opt,
                                       bool is_expensive) : InsatAction(type, params, is_expensive),
                                                            map_(map),
                                                            opt_(opt)
   {
   }

    bool InsatNav2dAction::CheckPreconditions(StateVarsType state)
    {
        return true;
    }

    ActionSuccessor InsatNav2dAction::GetSuccessor(StateVarsType state_vars, int thread_id)
    {
        std::vector<double> next_state_vars(2, 0);
        for (int i = 0; i < Action::params_["length"]; ++i)
        {
            next_state_vars[0] = state_vars[0] + i*move_dir_[0];
            next_state_vars[1] = state_vars[1] + i*move_dir_[1];

            int x_limit =  map_.size();
            int y_limit =  map_[0].size();
            bool in_range = inRange(next_state_vars[0], next_state_vars[1]);

            if (!in_range)
                return ActionSuccessor(false, {make_pair(StateVarsType(), -DINF)});

            bool is_collision =  false;

            auto footprint = getFootPrintRectangular(next_state_vars[0], next_state_vars[1], params_["footprint_size"]);

            for (auto& cell : footprint)
            {
                if (!isValidCell(cell.first, cell.second))
                    return ActionSuccessor(false, {make_pair(StateVarsType(), -DINF)});
            }

        }

        double cost = pow(pow((next_state_vars[0] - state_vars[0]), 2) + pow((next_state_vars[1] - state_vars[1]), 2), 0.5);;
        return ActionSuccessor(true, {make_pair(next_state_vars, cost)});

    }

    ActionSuccessor InsatNav2dAction::GetSuccessorLazy(StateVarsType state_vars, int thread_id)
    {
        std::vector<double> final_state_vars(3, 0);
        final_state_vars[0] = state_vars[0] + (params_["length"] - 1)*move_dir_[0];
        final_state_vars[1] = state_vars[1] + (params_["length"] - 1)*move_dir_[1];
        final_state_vars[2] = state_vars[2];

        int x_limit =  map_.size();
        int y_limit =  map_[0].size();
        bool in_range = inRange(final_state_vars[0], final_state_vars[1]);

        if (!in_range)
            return ActionSuccessor(false, {make_pair(StateVarsType(), -DINF)});

        bool is_collision =  false;

        auto footprint = getFootPrintRectangular(final_state_vars[0], final_state_vars[1], params_["footprint_size"]);

        for (auto& cell : footprint)
        {
            if (!isValidCell(cell.first, cell.second))
                return ActionSuccessor(false, {make_pair(StateVarsType(), -DINF)});
        }

        double cost = pow(pow((final_state_vars[0] - state_vars[0]), 2) + pow((final_state_vars[1] - state_vars[1]), 2), 0.5);;
        return ActionSuccessor(true, {make_pair(final_state_vars, cost)});

    }

    ActionSuccessor InsatNav2dAction::Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id)
    {
        return GetSuccessor(parent_state_vars, thread_id);
    }

    bool InsatNav2dAction::isValidCell(int x, int y) const
    {
        int x_limit =  map_.size();
        int y_limit =  map_[0].size();

        return  ((x >= 0) && (x < x_limit) &&
                 (y >= 0) && (y < y_limit) &&
                 (map_[round(x)][round(y)] == 0));
    }

    bool InsatNav2dAction::inRange(int x, int y)
    {
        int x_limit =  map_.size();
        int y_limit =  map_[0].size();

        return  ((x >= 0) && (x < x_limit) &&
                 (y >= 0) && (y < y_limit));
    }

    std::vector<std::pair<int, int>> InsatNav2dAction::getFootPrintRectangular(int x, int y, int footprint_size)
    {
        std::vector<std::pair<int,int>> footprint;

        if (params_["cache_footprint"])
        {

            lock_.lock();
            footprint =  footprint_;
            lock_.unlock();


            if (footprint.size() == 0)
            {
                // sbpl's footprint calculation method
                std::vector<sbpl_2Dpt_t> polygon;
                std::set<sbpl_2Dcell_t> cells;

                // sbpl's footprint function takes in (x,y) world coordinates not cells
                polygon.push_back(sbpl_2Dpt_t(footprint_size, footprint_size));
                polygon.push_back(sbpl_2Dpt_t(footprint_size, -footprint_size));
                polygon.push_back(sbpl_2Dpt_t(-footprint_size, -footprint_size));
                polygon.push_back(sbpl_2Dpt_t(-footprint_size, footprint_size));

                sbpl_xy_theta_pt_t pose(0, 0, 0);
                get_2d_footprint_cells(polygon, &cells, pose, 1);

                for (auto cell : cells)
                {
                    footprint.push_back(std::pair<int,int>(cell.x, cell.y));
                }

                lock_.lock();
                footprint_ = footprint;
                lock_.unlock();

            }

            for (auto& coord : footprint)
            {
                coord.first += x;
                coord.second += y;
            }
        }
        else
        {
            // sbpl's footprint calculation method
            std::vector<sbpl_2Dpt_t> polygon;
            std::set<sbpl_2Dcell_t> cells;

            // sbpl's footprint function takes in (x,y) world coordinates not cells
            polygon.push_back(sbpl_2Dpt_t(footprint_size, footprint_size));
            polygon.push_back(sbpl_2Dpt_t(footprint_size, -footprint_size));
            polygon.push_back(sbpl_2Dpt_t(-footprint_size, -footprint_size));
            polygon.push_back(sbpl_2Dpt_t(-footprint_size, footprint_size));

            sbpl_xy_theta_pt_t pose(x, y, 0);
            get_2d_footprint_cells(polygon, &cells, pose, 1);

            for (auto cell : cells)
            {
                footprint.push_back(std::pair<int,int>(cell.x, cell.y));
            }
        }

        return footprint;
    }

    // INSAT
    void InsatNav2dAction::setOpt(OptVecPtrType& opt)
    {
        opt_ = opt;
    }

  bool InsatNav2dAction::isFeasible(MatDf &traj, int thread_id) const
  {
    bool feas = true;
    for (int i=0; i<traj.cols(); ++i)
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

  TrajType InsatNav2dAction::optimize(const StateVarsType &s1,
                                                        const StateVarsType &s2,
                                                        int thread_id) const
  {
    Eigen::Map<const VecDf> p1(&s1[0], s1.size());
    Eigen::Map<const VecDf> p2(&s2[0], s2.size());

    return (*opt_)[thread_id].optimize(this, p1, p2, thread_id);
  }

  TrajType InsatNav2dAction::warmOptimize(const TrajType &t1,
                                                            const TrajType &t2,
                                                            int thread_id) const
  {
    return (*opt_)[thread_id].warmOptimize(this, t1, t2, thread_id);
  }

    TrajType InsatNav2dAction::warmOptimize(const TrajType& t, int thread_id) const
    {
        return (*opt_)[thread_id].warmOptimize(this, t);
    }

    double InsatNav2dAction::getCost(const TrajType &traj, int thread_id) const
  {
    return (*opt_)[thread_id].calculateCost(traj);
  }

}
