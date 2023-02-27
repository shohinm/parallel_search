#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/functional/hash.hpp>
#include "RobotNav2dActions.hpp"
#include <planners/WastarPlanner.hpp>
#include <planners/PwastarPlanner.hpp>
#include <planners/ArastarPlanner.hpp>
#include <planners/PasePlanner.hpp>
#include <planners/EpasePlanner.hpp>
#include <planners/GepasePlanner.hpp>
#include <planners/AgepasePlanner.hpp>
#include <planners/MplpPlanner.hpp>

using namespace std;
using namespace ps;

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
    vector<vector<int>> map;
    FILE *f;
    f = fopen(fname, "r");
    
    if (f)
    {
        if (fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width))
        {
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
double computeHeuristic(const StateVarsType& state_vars, double dist_thresh)
{
    double dist_to_goal_region = pow(pow((state_vars[0] - goal[0]), 2) + pow((state_vars[1] - goal[1]), 2), 0.5);

    if (dist_to_goal_region < dist_thresh)
        dist_to_goal_region = 0;
    return dist_to_goal_region;
}

double computeHeuristicStateToState(const StateVarsType& state_vars_1, const StateVarsType& state_vars_2)
{
    double dist = pow(pow((state_vars_1[0] - state_vars_2[0]), 2) + pow((state_vars_1[1] - state_vars_2[1]), 2), 0.5);
    if (dist < 0)
        dist = 0;
    return dist;
}

bool isGoalState(const StateVarsType& state_vars, double dist_thresh)
{
    return (computeHeuristic(state_vars, dist_thresh) <= 0);
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

    ParamsType expensive_action_params = action_params;
    expensive_action_params["cache_footprint"] = 1;
    
    auto move_up_controller_ptr = make_shared<MoveUpAction>("MoveUp", action_params, map);
    action_ptrs.emplace_back(move_up_controller_ptr);

    auto move_up_right_controller_ptr = make_shared<MoveUpRightAction>("MoveUpRight", expensive_action_params, map);
    action_ptrs.emplace_back(move_up_right_controller_ptr);

    auto move_right_controller_ptr = make_shared<MoveRightAction>("MoveRight", action_params, map);
    action_ptrs.emplace_back(move_right_controller_ptr);

    auto move_right_down_controller_ptr = make_shared<MoveRightDownAction>("MoveRightDown", expensive_action_params, map);
    action_ptrs.emplace_back(move_right_down_controller_ptr);

    auto move_down_controller_ptr = make_shared<MoveDownAction>("MoveDown", action_params, map);
    action_ptrs.emplace_back(move_down_controller_ptr);

    auto move_down_left_controller_ptr = make_shared<MoveDownLeftAction>("MoveDownLeft", expensive_action_params, map);
    action_ptrs.emplace_back(move_down_left_controller_ptr);

    auto move_left_controller_ptr = make_shared<MoveLeftAction>("MoveLeft", action_params, map);
    action_ptrs.emplace_back(move_left_controller_ptr);

    auto move_left_up_controller_ptr = make_shared<MoveLeftUpAction>("MoveLeftUp", expensive_action_params, map);
    action_ptrs.emplace_back(move_left_up_controller_ptr);

}

void constructPlanner(string planner_name, shared_ptr<Planner>& planner_ptr, vector<shared_ptr<Action>>& action_ptrs, ParamsType& planner_params, ParamsType& action_params)
{
    if (planner_name == "wastar")
        planner_ptr = make_shared<WastarPlanner>(planner_params);
    else if (planner_name == "pwastar")
        planner_ptr = make_shared<PwastarPlanner>(planner_params);
    else if (planner_name == "arastar")
        planner_ptr = make_shared<ArastarPlanner>(planner_params);
    else if (planner_name == "pase")
        planner_ptr = make_shared<PasePlanner>(planner_params);
    else if (planner_name == "epase")
        planner_ptr = make_shared<EpasePlanner>(planner_params); 
    else if (planner_name == "gepase")
        planner_ptr = make_shared<GepasePlanner>(planner_params); 
    else if (planner_name == "agepase")
        planner_ptr = make_shared<AgepasePlanner>(planner_params);
    else if (planner_name == "mplp")
        planner_ptr = make_shared<MplpPlanner>(planner_params); 
    else
        throw runtime_error("Planner type not identified!");      

    planner_ptr->SetActions(action_ptrs);
    planner_ptr->SetStateMapKeyGenerator(bind(StateKeyGenerator, placeholders::_1));
    planner_ptr->SetEdgeKeyGenerator(bind(EdgeKeyGenerator, placeholders::_1));
    planner_ptr->SetHeuristicGenerator(bind(computeHeuristic, placeholders::_1, action_params["length"]));
    planner_ptr->SetStateToStateHeuristicGenerator(bind(computeHeuristicStateToState, placeholders::_1, placeholders::_2));
    planner_ptr->SetGoalChecker(bind(isGoalState, placeholders::_1, action_params["length"]));
}

void loadStartsGoalsFromFile(vector<vector<double>>& starts, vector<vector<double>>& goals, int scale, int num_runs, const string& path)
{
    ifstream starts_fin(path + "nav2d_starts.txt");
    ifstream goals_fin(path + "nav2d_goals.txt");    
   
    for (int j = 0; j < num_runs; ++j)
    {
        vector<double> start, goal;
        double val_start, val_goal;
        for (int i = 0; i < 2; ++i)
        {
            starts_fin >> val_start;
            goals_fin >> val_goal;                
            start.push_back(scale*val_start);
            goal.push_back(scale*val_goal);
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
    int num_threads;
    double time_budget = 0;

    if (!strcmp(argv[1], "wastar"))
    {
        if (argc != 2) throw runtime_error("Format: run_robot_nav_2d wastar");
        num_threads = 1;
    }
    else if (!strcmp(argv[1], "mplp"))
    {
        if (argc != 3) throw runtime_error("Format: run_robot_nav_2d [planner_name] [num_threads]");
        if (atoi(argv[2]) < 4) throw runtime_error("mplp requires a minimum of 4 threads");
        num_threads = atoi(argv[2]);
    }
    else if (!strcmp(argv[1], "arastar"))
    {
        if (argc != 3) throw runtime_error("Format: run_robot_nav_2d arastar [time_budget]");
        num_threads = 1;
        time_budget = atof(argv[2]);
    }
    else if (!strcmp(argv[1], "agepase"))
    {
        if (argc != 4) throw runtime_error("Format: run_robot_nav_2d agepase [num_threads] [time_budget]");
        num_threads = atoi(argv[2]);
        time_budget = atof(argv[3]);
    }
    else
    {
        if (argc != 3) throw runtime_error("Format: run_robot_nav_2d [planner_name] [num_threads]");
        num_threads = atoi(argv[2]);
    }
    

    // Experiment parameters
    int num_runs = 50;
    vector<int> scale_vec = {5, 5, 5, 10, 5};
    bool visualize_plan = true;
    bool load_starts_goals_from_file = true;

    // Define planner parameters
    ParamsType planner_params;
    string planner_name = argv[1];
    planner_params["num_threads"] = num_threads;
    planner_params["heuristic_weight"] = 50;
    // planner_params["heuristic_weight"] = 3;
    if (time_budget)
    {
        planner_params["timeout"] = time_budget;
    }
    else
    {
        planner_params["timeout"] = 5;
    }
    
    // Read map
    int width, height;
    cv::Mat img;
    
    vector<vector<vector<int>>> map_vec;
    vector<cv::Mat> img_vec;

    map_vec.emplace_back(loadMap("../examples/robot_nav_2d/resources/hrt201n/hrt201n.map", img, width, height, scale_vec[0]));
    img_vec.emplace_back(img.clone());
    map_vec.emplace_back(loadMap("../examples/robot_nav_2d/resources/den501d/den501d.map", img, width, height, scale_vec[1]));
    img_vec.emplace_back(img.clone());
    map_vec.emplace_back(loadMap("../examples/robot_nav_2d/resources/den520d/den520d.map", img, width, height, scale_vec[2]));
    img_vec.emplace_back(img.clone());
    map_vec.emplace_back(loadMap("../examples/robot_nav_2d/resources/ht_chantry/ht_chantry.map", img, width, height, scale_vec[3]));
    img_vec.emplace_back(img.clone());
    map_vec.emplace_back(loadMap("../examples/robot_nav_2d/resources/brc203d/brc203d.map", img, width, height, scale_vec[4]));
    img_vec.emplace_back(img.clone());


    vector<string> starts_goals_path = {"../examples/robot_nav_2d/resources/hrt201n/", 
    "../examples/robot_nav_2d/resources/den501d/", 
    "../examples/robot_nav_2d/resources/den520d/",
    "../examples/robot_nav_2d/resources/ht_chantry/",
    "../examples/robot_nav_2d/resources/brc203d/",
    };

    vector<double> all_maps_time_vec, all_maps_cost_vec;
    vector<int> all_maps_num_edges_vec;
    unordered_map<string, vector<double>> all_action_eval_times;

    for (int m_idx = 0; m_idx < map_vec.size(); ++m_idx)
    // for (int m_idx = 2; m_idx < 3; ++m_idx) // Fail case for optimality (thread=10)
    {
        auto map = map_vec[m_idx];
        auto img = img_vec[m_idx];
        auto scale = scale_vec[m_idx];

        // Construct actions
        ParamsType action_params;
        vector<shared_ptr<Action>> action_ptrs;
        constructActions(action_ptrs, action_params, map);

        // Construct planner
        shared_ptr<Planner> planner_ptr;
        constructPlanner(planner_name, planner_ptr, action_ptrs, planner_params, action_params);

        // Read starts and goals from text file
        vector<vector<double>> starts, goals;

        if (load_starts_goals_from_file)
            loadStartsGoalsFromFile(starts, goals, scale, num_runs, starts_goals_path[m_idx]);
        else
        {
            starts = vector<vector<double>> (num_runs, {scale*10.0, scale*61.0});
            goals = vector<vector<double>> (num_runs, {scale*200.0, scale*170.0});
        }

        // Run experiments
        int start_goal_idx = 0;
        vector<double> time_vec, cost_vec;
        vector<int> num_edges_vec, threads_used_vec;
        vector<int> jobs_per_thread(planner_params["num_threads"], 0);
        unordered_map<string, vector<double>> action_eval_times;

        cout << "Map size: (" << map.size() << ", " << map[0].size() << ") | " 
        << " | Planner: " << planner_name   
        << " | Heuristic weight: " << planner_params["heuristic_weight"]   
        << " | Number of threads: " << planner_params["num_threads"]   
        << " | Number of runs: " << num_runs
        << endl;
        cout <<  "---------------------------------------------------" << endl;

        if (visualize_plan) cv::namedWindow("Plan", cv::WINDOW_AUTOSIZE );// Create a window for display.
        
        int num_success = 0;
        // for (int exp_idx = 45; exp_idx < 46; ++exp_idx )
        for (int exp_idx = 0; exp_idx < num_runs; ++exp_idx)
        {
            cout << "Experiment: " << exp_idx;

            if (start_goal_idx >= starts.size()) 
                start_goal_idx = 0;

            // start_goal_idx = 45; // Fail case to find optimal (thread=10, h_weight=1)

            // Set start state
            planner_ptr->SetStartState(starts[start_goal_idx]);
            
            // Set goal conditions
            goal.clear();
            goal.emplace_back(goals[start_goal_idx][0]);
            goal.emplace_back(goals[start_goal_idx][1]);

            double t=0, cost=0;
            int num_edges=0;

            bool plan_found = planner_ptr->Plan();
            
            if (plan_found)
            {
                auto planner_stats = planner_ptr->GetStats();
                
                time_vec.emplace_back(planner_stats.total_time_);
                all_maps_time_vec.emplace_back(planner_stats.total_time_);
                cost_vec.emplace_back(planner_stats.path_cost_);
                all_maps_cost_vec.emplace_back(planner_stats.path_cost_);
                num_edges_vec.emplace_back(planner_stats.num_evaluated_edges_);
                all_maps_num_edges_vec.emplace_back(planner_stats.num_evaluated_edges_);

                for (auto& [action, times] : planner_stats.action_eval_times_)
                { 
                    action_eval_times[action].insert(action_eval_times[action].end(), times.begin(), times.end());
                    all_action_eval_times[action].insert(all_action_eval_times[action].end(), times.begin(), times.end());
                }

                threads_used_vec.emplace_back(planner_stats.num_threads_spawned_);
                cout << " | Time (s): " << planner_stats.total_time_ 
                << " | Cost: " << planner_stats.path_cost_ 
                << " | Length: " << planner_stats.path_length_
                << " | State expansions: " << planner_stats.num_state_expansions_
                << " | Threads used: " << planner_stats.num_threads_spawned_ << "/" << planner_params["num_threads"]
                << " | Lock time: " <<  planner_stats.lock_time_
                << " | Expand time: " << planner_stats.cumulative_expansions_time_
                << " | Threads: " << planner_stats.num_threads_spawned_ << "/" << planner_params["num_threads"] << endl;
               
                // cout << endl << "------------- Jobs per thread -------------" << endl;
                // for (int tidx = 0; tidx < planner_params["num_threads"]; ++tidx)
                    // cout << "thread: " << tidx << " jobs: " << planner_stats.num_jobs_per_thread_[tidx] << endl;
                for (int tidx = 0; tidx < planner_params["num_threads"]; ++tidx)
                    jobs_per_thread[tidx] += planner_stats.num_jobs_per_thread_[tidx];        
                
                num_success++;
            }
            else
                cout << " | Plan not found!" << endl;

            ++start_goal_idx;   

            if (visualize_plan)
            {
                cv::Mat img2 = img.clone();
                
                // Display map with start and goal
                for (auto& plan_element: planner_ptr->GetPlan())
                {
                    auto c1 = cv::Point(plan_element.state_[0]-action_params["footprint_size"], plan_element.state_[1]+action_params["footprint_size"]);
                    auto c2 = cv::Point(plan_element.state_[0]+action_params["footprint_size"], plan_element.state_[1]-action_params["footprint_size"]);
                    cv::rectangle(img2, c1, c2, cv::Scalar(255, 0, 0), -1, 8);
                }

                auto c1 = cv::Point(starts[exp_idx][0]-action_params["footprint_size"], starts[exp_idx][1]+action_params["footprint_size"]);
                auto c2 = cv::Point(starts[exp_idx][0]+action_params["footprint_size"], starts[exp_idx][1]-action_params["footprint_size"]);
                cv::rectangle(img2, c1, c2, cv::Scalar(0, 255, 0), -1, 8);

                c1 = cv::Point(goals[exp_idx][0]-action_params["footprint_size"], goals[exp_idx][1]+action_params["footprint_size"]);
                c2 = cv::Point(goals[exp_idx][0]+action_params["footprint_size"], goals[exp_idx][1]-action_params["footprint_size"]);
                cv::rectangle(img2, c1, c2, cv::Scalar(0, 0, 255), -1, 8);

                cv::resize(img2, img2, cv::Size(4*img.cols/scale, 4*img.rows/scale));
                cv::imshow("Plan", img2);
                cv::waitKey(500);

                img2.setTo(cv::Scalar(0,0,0));
                cv::imshow("Plan", img2);

            }  
        }

        cout << endl << "************************" << endl;
        cout << "Number of runs: " << num_runs << endl;
        cout << "Mean time: " << accumulate(time_vec.begin(), time_vec.end(), 0.0)/time_vec.size() << endl;
        cout << "Mean cost: " << accumulate(cost_vec.begin(), cost_vec.end(), 0.0)/cost_vec.size() << endl;    
        cout << "Mean threads used: " << accumulate(threads_used_vec.begin(), threads_used_vec.end(), 0.0)/threads_used_vec.size() << "/" << planner_params["num_threads"] << endl;
        cout << "Mean evaluated edges: " << roundOff(accumulate(num_edges_vec.begin(), num_edges_vec.end(), 0.0)/double(num_edges_vec.size()), 2) << endl;
        cout << endl << "------------- Mean jobs per thread -------------" << endl;
        for (int tidx = 0; tidx < planner_params["num_threads"]; ++tidx)
        {
            cout << "thread: " << tidx << " jobs: " << jobs_per_thread[tidx]/num_success << endl;
        }
        cout << "************************" << endl;
    
        cout << endl << "------------- Mean action eval times -------------" << endl;
        for (auto [action, times] : action_eval_times)
        {
            cout << action << ": " << accumulate(times.begin(), times.end(), 0.0)/times.size() << endl; 
        }
        cout << "************************" << endl;
    }

    cout << endl << "************ Global Stats ************" << endl;
    cout << "Mean time: " << accumulate(all_maps_time_vec.begin(), all_maps_time_vec.end(), 0.0)/all_maps_time_vec.size() << endl;
    cout << "Mean cost: " << accumulate(all_maps_cost_vec.begin(), all_maps_cost_vec.end(), 0.0)/all_maps_cost_vec.size() << endl;    
    cout << "Mean evaluated edges: " << roundOff(accumulate(all_maps_num_edges_vec.begin(), all_maps_num_edges_vec.end(), 0.0)/double(all_maps_num_edges_vec.size()), 2) << endl;
    cout << endl << "************************" << endl;

    cout << endl << "------------- Mean action eval times -------------" << endl;
    for (auto [action, times] : all_action_eval_times)
    {
        cout << action << ": " << accumulate(times.begin(), times.end(), 0.0)/times.size() << endl; 
    }
    cout << "************************" << endl;



}
