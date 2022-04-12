#include <cmath>
#include <iostream>
#include <fstream>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/functional/hash.hpp>
#include "PointRobotActions.hpp"
#include <planners/EpasePlanner.hpp>

using namespace std;
using namespace epase;

vector<double> goal;

double to_degrees(double rads)
{
    return rads * 180.0 / M_PI;
}

double roundOff(double value, unsigned char prec)
{
    double pow_10 = pow(10.0, (double)prec);
    return round(value * pow_10) / pow_10;
}

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
double computeHeuristic(const StatePtrType& state_ptr)
{
    auto state_vars = state_ptr->GetStateVars();
    double dist_to_goal_region = pow(pow((state_vars[0] - goal[0]), 2) + pow((state_vars[1] - goal[1]), 2), 0.5);

    if (dist_to_goal_region < 0)
        dist_to_goal_region = 0;
    return dist_to_goal_region;
}

double computeHeuristicStateToState(const StatePtrType& state_ptr_1, const StatePtrType& state_ptr_2)
{
    auto state_vars_1 = state_ptr_1->GetStateVars();
    auto state_vars_2 = state_ptr_2->GetStateVars();
    double dist = pow(pow((state_vars_1[0] - state_vars_2[0]), 2) + pow((state_vars_1[1] - state_vars_2[1]), 2), 0.5);
    if (dist < 0)
        dist = 0;
    return dist;
}

bool isGoalState(const StatePtrType& state_ptr, double dist_thresh)
{
    return (computeHeuristic(state_ptr) < dist_thresh);
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

void constructActions(vector<shared_ptr<Action>>& action_ptrs, ParamsType& action_params, vector<vector<int>>& map)
{
    // Define action parameters
    action_params["length"] = 25;
    action_params["footprint_size"] = 16;
    action_params["cache_footprint"] = 1;
    
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

void constructPlanner(string planner_name, shared_ptr<Planner>& planner_ptr, vector<shared_ptr<Action>>& action_ptrs, ParamsType& planner_params, ParamsType& action_params)
{
    if (planner_name == "epase")
        planner_ptr = make_shared<EpasePlanner>(planner_params);
    else
        throw runtime_error("Planner type not identified!");      

    planner_ptr->SetActions(action_ptrs);
    planner_ptr->SetStateMapKeyGenerator(bind(StateKeyGenerator, placeholders::_1));
    planner_ptr->SetEdgeKeyGenerator(bind(EdgeKeyGenerator, placeholders::_1));
    planner_ptr->SetHeuristicGenerator(bind(computeHeuristic, placeholders::_1));
    planner_ptr->SetStateToStateHeuristicGenerator(bind(computeHeuristicStateToState, placeholders::_1, placeholders::_2));
    planner_ptr->SetGoalChecker(bind(isGoalState, placeholders::_1, action_params["length"]));
}

void loadStartsGoalsFromFile(vector<vector<double>>& starts, vector<vector<double>>& goals, int scale, int num_runs)
{
    ifstream starts_fin("../examples/point_robot_nav/resources/nav2d_starts.txt");
    ifstream goals_fin("../examples/point_robot_nav/resources/nav2d_goals.txt");    
   
    for (int j = 0; j < num_runs; ++j)
    {
        vector<double> start, goal;
        double val_start, val_goal;
        for (int i = 0; i < 2; ++i)
        {
            starts_fin >> val_start;
            goals_fin >> val_goal;                
            start.push_back((scale/5.0)*val_start);
            goal.push_back((scale/5.0)*val_goal);
        }
        start[2] = to_degrees(start[2]);
        goal[2] = to_degrees(goal[2]);
        starts.emplace_back(start);
        goals.emplace_back(goal);

        double cost, length;
        starts_fin >> cost;            
        starts_fin >> length;            
    }
}

int main(int argc, char* argv[])
{
    // Experiment parameters
    int num_runs = 50;
    int scale = 5;
    bool visualize_plan = false;
    bool load_starts_goals_from_file = true;

    // Define planner parameters
    ParamsType planner_params;
    string planner_name = "epase";
    planner_params["num_threads"] = 5;
    planner_params["heuristic_weight"] = 1;
    
    // Read map
    vector<vector<int>> map;
    int width, height;
    cv::Mat img;
    map = loadMap("../examples/point_robot_nav/resources/hrt201n.map", img, width, height, scale);

    // Read starts and goals from text file
    vector<vector<double>> starts, goals;

    if (load_starts_goals_from_file)
        loadStartsGoalsFromFile(starts, goals, scale, num_runs);
    else
    {
        starts = vector<vector<double>> (num_runs, {scale*10, scale*61});
        goals = vector<vector<double>> (num_runs, {scale*200, scale*170});
    }

    // Construct actions
    ParamsType action_params;
    vector<shared_ptr<Action>> action_ptrs;
    constructActions(action_ptrs, action_params, map);

    // Construct planner
    shared_ptr<Planner> planner_ptr;
    constructPlanner(planner_name, planner_ptr, action_ptrs, planner_params, action_params);

    // Run experiments
    int start_goal_idx = 0;
    vector<double> time_vec, cost_vec;
    vector<int> num_edges_vec;

    cout << "Map size: (" << map.size() << ", " << map[0].size() << ") | " 
    << " | planner: " << planner_name   
    << " | heuristic_weight: " << planner_params["heuristic_weight"]   
    << " | num_threads: " << planner_params["num_threads"]   
    << endl;
    cout <<  "---------------------------------------------------" << endl;

    for (int exp_idx = 0; exp_idx < num_runs; ++exp_idx )
    {
        cout << "Experiment: " << exp_idx;

        if (start_goal_idx >= starts.size()) 
            start_goal_idx = 0;

        // Set start state
        planner_ptr->SetStartState(starts[start_goal_idx]);
        
        // Set goal conditions
        goal.clear();
        goal.emplace_back(goals[start_goal_idx][0]);
        goal.emplace_back(goals[start_goal_idx][1]);

        double t=0, cost=0;
        int num_edges=0;

        bool plan_found = planner_ptr->Plan(exp_idx);
        if (plan_found)
        {
            auto planner_stats = planner_ptr->GetStats();
            time_vec.emplace_back(planner_stats.total_time_);
            cost_vec.emplace_back(planner_stats.path_cost_);
            num_edges_vec.emplace_back(planner_stats.num_evaluated_edges_);
            cout << " | Time (s): " << planner_stats.total_time_ << " | Cost: " << planner_stats.path_cost_ << endl;
        }
        else
            cout << " | Plan not found!" << endl;

        ++start_goal_idx;            
        // if (visualize_plan)
        // {
        //     // Display map with start and goal
        //     cv::namedWindow("Plan", cv::WINDOW_AUTOSIZE );// Create a window for display.
        //     for (auto& plan_element: mplp_ptr->GetPlan())
        //     {
        //         auto pose = plan_element.intermediate_states_costs_.front().first.GetRobotState()->GetPose2D();
        //         cv::circle(img, cv::Point(pose(0), pose(1)), cfg["controllers"]["short"]["footprint_size"].as<int>(), cv::Scalar(255, 0, 0), -1, 8);
        //     }
        //     cv::circle(img, cv::Point(starts[exp_idx][0], starts[exp_idx][1]), cfg["controllers"]["short"]["footprint_size"].as<int>(), cv::Scalar(0, 255, 0), -1, 8);
        //     cv::circle(img, cv::Point(goals[exp_idx][0], goals[exp_idx][1]), cfg["controllers"]["short"]["footprint_size"].as<int>(), cv::Scalar(0, 0, 255), -1, 8 );

        //     cv::Mat img2;
        //     cv::resize(img, img2, cv::Size(4*img.cols/scale, 4*img.rows/scale));
        //     cv::imshow("Plan", img2);

        //     cv::waitKey(0);
        // }  
    }

    cout << endl << "************************" << endl;
    cout << "Number of runs: " << num_runs << endl;
    cout << "Mean time: " << accumulate(time_vec.begin(), time_vec.end(), 0.0)/time_vec.size() << endl;
    cout << "Mean cost: " << accumulate(cost_vec.begin(), cost_vec.end(), 0.0)/cost_vec.size() << endl;    
    cout << "Mean evaluated edges: " << roundOff(accumulate(num_edges_vec.begin(), num_edges_vec.end(), 0.0)/double(num_edges_vec.size()), 2) << endl;
    cout << "************************" << endl;




}