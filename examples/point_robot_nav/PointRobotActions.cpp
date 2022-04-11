#include <cmath>
#include <sbpl/utils/utils.h>
#include "PointRobotActions.hpp"

using namespace std;
using namespace epase;


ActionSuccessor PointRobotAction::Apply(StateVarsType state_vars, int thread_id)
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
        {
            return ActionSuccessor(false, {make_pair(StateVarsType(), -DINF)});
        }

        bool is_collision =  false;

        auto t_start = chrono::system_clock::now();
        auto footprint = getFootPrintRectangular(next_state_vars[0], next_state_vars[1], params_["footprint_size"]);
        auto t_end = chrono::system_clock::now();
        double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
        // cout << "footprint calculation time: " << 1e-6*t_elapsed  << " ms " << endl;
        // cout << "New position: " << next_state_vars[0] << ", " << next_state_vars[1] << endl;
        // displayMap({footprint}, 100);

        for (auto& cell : footprint)
        {
            if (!isValidCell(cell.first, cell.second))
            {
	            return ActionSuccessor(false, {make_pair(StateVarsType(), -DINF)});
            }
        }

    }

    
    double cost = pow(pow((next_state_vars[0] - state_vars[0]), 2) + pow((next_state_vars[1] - state_vars[1]), 2), 0.5);;
    return ActionSuccessor(true, {make_pair(next_state_vars, cost)});

}

bool PointRobotAction::CheckPreconditions(StateVarsType state)
{
	return true;
}

bool PointRobotAction::isValidCell(int x, int y)
{   
    int x_limit =  map_.size();
    int y_limit =  map_[0].size();

    return  ((x >= 0) && (x < x_limit) && 
            (y >= 0) && (y < y_limit) &&
            (map_[round(x)][round(y)] == 0));
}

bool PointRobotAction::inRange(int x, int y)
{   
    int x_limit =  map_.size();
    int y_limit =  map_[0].size();

    return  ((x >= 0) && (x < x_limit) && 
            (y >= 0) && (y < y_limit));
}

vector<pair<int, int>> PointRobotAction::getFootPrintRectangular(int x, int y, int footprint_size)
{
    vector<pair<int,int>> footprint;

    if (params_["cache_footprint"])
    {
        if (footprint_.size() == 0)
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
                footprint_.push_back(pair<int,int>(cell.x, cell.y));
            }
        }

        footprint = footprint_;
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
