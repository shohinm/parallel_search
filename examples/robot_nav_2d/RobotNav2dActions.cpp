#include <iostream>
#include <cmath>
#include <sbpl/utils/utils.h>
#include "RobotNav2dActions.hpp"

using namespace std;
using namespace ps;


bool RobotNav2dAction::CheckPreconditions(StateVarsType state)
{
    return true;
}

ActionSuccessor RobotNav2dAction::GetSuccessor(StateVarsType state_vars, int thread_id)
{
    vector<double> next_state_vars(3, 0);
    for (int i = 0; i < params_["length"]; ++i)
    {
        next_state_vars[0] = state_vars[0] + i*move_dir_[0];
        next_state_vars[1] = state_vars[1] + i*move_dir_[1];
        next_state_vars[2] = state_vars[2];

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

ActionSuccessor RobotNav2dAction::GetSuccessorLazy(StateVarsType state_vars, int thread_id)
{
    vector<double> final_state_vars(3, 0);
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

ActionSuccessor RobotNav2dAction::Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id)
{
    return GetSuccessor(parent_state_vars, thread_id);
}

bool RobotNav2dAction::isValidCell(int x, int y)
{   
    int x_limit =  map_.size();
    int y_limit =  map_[0].size();

    return  ((x >= 0) && (x < x_limit) && 
            (y >= 0) && (y < y_limit) &&
            (map_[round(x)][round(y)] == 0));
}

bool RobotNav2dAction::inRange(int x, int y)
{   
    int x_limit =  map_.size();
    int y_limit =  map_[0].size();

    return  ((x >= 0) && (x < x_limit) && 
            (y >= 0) && (y < y_limit));
}

vector<pair<int, int>> RobotNav2dAction::getFootPrintRectangular(int x, int y, int footprint_size)
{
    vector<pair<int,int>> footprint;

    if (params_["cache_footprint"])
    {

        lock_.lock();
        footprint =  footprint_;
        lock_.unlock();


        if (footprint.size() == 0)
        {
            // sbpl's footprint calculation method
            vector<sbpl_2Dpt_t> polygon;
            set<sbpl_2Dcell_t> cells;

            // sbpl's footprint function takes in (x,y) world coordinates not cells
            polygon.push_back(sbpl_2Dpt_t(footprint_size, footprint_size));
            polygon.push_back(sbpl_2Dpt_t(footprint_size, -footprint_size));
            polygon.push_back(sbpl_2Dpt_t(-footprint_size, -footprint_size));
            polygon.push_back(sbpl_2Dpt_t(-footprint_size, footprint_size));

            sbpl_xy_theta_pt_t pose(0, 0, 0);
            get_2d_footprint_cells(polygon, &cells, pose, 1);

            for (auto cell : cells)
            {
                footprint.push_back(pair<int,int>(cell.x, cell.y));
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
        vector<sbpl_2Dpt_t> polygon;
        set<sbpl_2Dcell_t> cells;

        // sbpl's footprint function takes in (x,y) world coordinates not cells
        polygon.push_back(sbpl_2Dpt_t(footprint_size, footprint_size));
        polygon.push_back(sbpl_2Dpt_t(footprint_size, -footprint_size));
        polygon.push_back(sbpl_2Dpt_t(-footprint_size, -footprint_size));
        polygon.push_back(sbpl_2Dpt_t(-footprint_size, footprint_size));

        sbpl_xy_theta_pt_t pose(x, y, 0);
        get_2d_footprint_cells(polygon, &cells, pose, 1);

        for (auto cell : cells)
        {
            footprint.push_back(pair<int,int>(cell.x, cell.y));
        }
    }

    return footprint;
}
