#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/functional/hash.hpp>
#include "PointRobotActions.hpp"
#include <EpasePlanner.hpp>

using namespace std;
using namespace epase;

vector<double> goal;

vector<vector<int>> loadMap(const char *fname, cv::Mat& img, int &width, int &height, int scale=1)
{
    FILE *f;
    f = fopen(fname, "r");
    vector<vector<int>> map;
    if (f)
    {
        fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
        map.resize(width, vector<int>(height));

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                char c;
                do {
                    fscanf(f, "%c", &c);
                } while (isspace(c));

                map[x][y] = (c == '.' || c == 'G' || c == 'S' || c == 'T') ? 0 : 100;
            }
        }
        fclose(f);
    }

    vector<vector<int>> scaled_map;
    int scaled_height = scale*height;
    int scaled_width = scale*width;
    scaled_map.resize(scaled_width, vector<int>(scaled_height));

    for (int y = 0; y < scaled_height; y++)
    {
        for (int x = 0; x < scaled_width; x++)
        {
            scaled_map[x][y] = map[x/scale][y/scale];
        }
    }

    img = cv::Mat(scaled_height, scaled_width, CV_8UC3);

    for (int y = 0; y < scaled_height; y++)
    {
        for (int x = 0; x < scaled_width; x++)
        {
            img.at<cv::Vec3b>(y,x) = (scaled_map[x][y] > 0) ? cv::Vec3b(0,0,0) : cv::Vec3b(255,255,255);
        }
    }

    return scaled_map;

}
double computeHeuristic(const StatePtrType state_ptr)
{
    auto state_vars = state_ptr->GetStateVars();
    double dist_to_goal_region = pow(pow((state_vars[0] - goal[0]), 2) + pow((state_vars[1] - goal[1]), 2), 0.5);

    if (dist_to_goal_region < 0)
        dist_to_goal_region = 0;
    return dist_to_goal_region;
}

double computeHeuristicStateToState(const StatePtrType state_ptr_1, const StatePtrType state_ptr_2)
{
    auto state_vars_1 = state_ptr_1->GetStateVars();
    auto state_vars_2 = state_ptr_2->GetStateVars();
    double dist = pow(pow((state_vars_1[0] - state_vars_2[0]), 2) + pow((state_vars_1[1] - state_vars_2[1]), 2), 0.5);
    if (dist < 0)
        dist = 0;
    return dist;
}

size_t StateKeyGenerator(const StateVarsType& state_vars)
{ 
    int x = round(state_vars[0]); 
    int y = round(state_vars[1]); 
    size_t seed = 0;
    boost::hash_combine(seed, x);
    boost::hash_combine(seed, y);
    return seed;
}

size_t EdgeKeyGenerator(const EdgePtrType& edge_ptr)
{
    int controller_id;
    auto action_ptr = edge_ptr->action_ptr_;

    if (action_ptr->GetType() ==  "MoveUp")
        controller_id = 0;
    else if (action_ptr->GetType() ==  "MoveUpLong")
        controller_id = 1;
    else if (action_ptr->GetType() ==  "MoveUpRight")
        controller_id = 2;
    else if (action_ptr->GetType() ==  "MoveUpRightLong")
        controller_id = 3;
    else if (action_ptr->GetType() ==  "MoveRight")
        controller_id = 4;
    else if (action_ptr->GetType() ==  "MoveRightLong")
        controller_id = 5;
    else if (action_ptr->GetType() ==  "MoveRightDown")
        controller_id = 6;
    else if (action_ptr->GetType() ==  "MoveRightDownLong")
        controller_id = 7;
    else if (action_ptr->GetType() ==  "MoveDown")
        controller_id = 8;
    else if (action_ptr->GetType() ==  "MoveDownLong")
        controller_id = 9;
    else if (action_ptr->GetType() ==  "MoveDownLeft")
        controller_id = 10;
    else if (action_ptr->GetType() ==  "MoveDownLeftLong")
        controller_id = 11;
    else if (action_ptr->GetType() ==  "MoveLeft")
        controller_id = 12;
    else if (action_ptr->GetType() ==  "MoveLeftLong")
        controller_id = 13;
    else if (action_ptr->GetType() ==  "MoveLeftUp")
        controller_id = 14;
    else if (action_ptr->GetType() ==  "MoveLeftUpLong")
        controller_id = 15;
    else
        throw runtime_error("Controller type not recognized in getEdgeKey!");

    size_t seed = 0;
    boost::hash_combine(seed, edge_ptr->parent_state_ptr_->GetStateID());
    boost::hash_combine(seed, controller_id);


    return seed;
}

void loadMap(vector<vector<int>>& map)
{

}

void constructActions(vector<shared_ptr<Action>>& action_ptrs, vector<vector<int>>& map)
{
    ParamsType action_params;

    auto move_up_controller_ptr = make_shared<MoveUpAction>("MoveUp", action_params, map);
    action_ptrs.emplace_back(move_up_controller_ptr);

    auto move_up_right_controller_ptr = make_shared<MoveUpRightAction>("MoveUpRight", action_params, map);
    action_ptrs.emplace_back(move_up_right_controller_ptr);

    auto move_right_controller_ptr = make_shared<MoveRightAction>("MoveRight", action_params, map);
    action_ptrs.emplace_back(move_right_controller_ptr);

    auto move_right_down_controller_ptr = make_shared<MoveRightDownAction>("MoveRightDown", action_params, map);
    action_ptrs.emplace_back(move_right_down_controller_ptr);

    auto move_down_controller_ptr = make_shared<MoveDownAction>("MoveDown", action_params, map);
    action_ptrs.emplace_back(move_down_controller_ptr);

    auto move_down_left_controller_ptr = make_shared<MoveDownLeftAction>("MoveDownLeft", action_params, map);
    action_ptrs.emplace_back(move_down_left_controller_ptr);

    auto move_left_controller_ptr = make_shared<MoveLeftAction>("MoveLeft", action_params, map);
    action_ptrs.emplace_back(move_left_controller_ptr);

    auto move_left_up_controller_ptr = make_shared<MoveLeftUpAction>("MoveLeftUp", action_params, map);
    action_ptrs.emplace_back(move_left_up_controller_ptr);

}

int main(int argc, char* argv[])
{

    // Define planner parameters
    ParamsType planner_params;
    planner_params["num_threads"] = 10;

    // Read map
    vector<vector<int>> map;
    loadMap(map);

    // Construct actions
    vector<shared_ptr<Action>> action_ptrs;
    constructActions(action_ptrs, map);


    // Construct planner
    shared_ptr<Planner> planner_ptr = make_shared<EpasePlanner>(planner_params);
    planner_ptr->SetActions(action_ptrs);
    planner_ptr->SetStateMapKeyGenerator(bind(StateKeyGenerator, placeholders::_1));
    planner_ptr->SetEdgeKeyGenerator(bind(EdgeKeyGenerator, placeholders::_1));
    planner_ptr->SetHeuristicGenerator(bind(computeHeuristic, placeholders::_1));
    planner_ptr->SetStateToStateHeuristicGenerator(bind(computeHeuristicStateToState, placeholders::_1, placeholders::_2));

}