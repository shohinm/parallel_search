#include <iostream>
#include <cmath>
#include <planners/RrtPlanner.hpp>

using namespace std;
using namespace ps;

RrtPlanner::RrtPlanner(ParamsType planner_params):
Planner(planner_params)
{    

}

RrtPlanner::~RrtPlanner()
{
    
}

void RrtPlanner::SetGoalState(const StateVarsType& state_vars)
{
    goal_state_ptr_ = constructState(state_vars);
}

bool RrtPlanner::Plan()
{
    initialize();

    auto t_start = chrono::steady_clock::now();

    if (num_threads_ == 1)
    {
        rrtThread(0);
    }
    else
    {
        for (int thread_id = 0; thread_id < num_threads_-1; ++thread_id)
        {
            if (VERBOSE) cout << "Spawining state expansion thread " << thread_id << endl;
            rrt_futures_.emplace_back(async(launch::async, &RrtPlanner::rrtThread, this, thread_id));
        }        
        
        planner_stats_.num_threads_spawned_ += rrt_futures_.size();
    }

    // Spin till termination, should be replaced by conditional variable
    while(!terminate_){}

    auto t_end = chrono::steady_clock::now();
    double t_elapsed = chrono::duration_cast<chrono::nanoseconds>(t_end-t_start).count();
    planner_stats_.total_time_ = 1e-9*t_elapsed;
    exit();
}

void RrtPlanner::initialize()
{
    // Initialize planner stats
    planner_stats_ = PlannerStats();
    planner_stats_.num_jobs_per_thread_.resize(num_threads_, 0);
    planner_stats_.num_threads_spawned_ = 2;
}

void RrtPlanner::rrtThread(int thread_id)
{
    while (!terminate_)
    {
        auto sampled_state = sampleState();
        auto nearest_neighbor = getNearestNeighbor(sampled_state);
        bool is_collision;
        auto state_ptr = extend(nearest_neighbor, sampled_state, is_collision, thread_id);

        if (isGoalState(state_ptr) && (!terminate_))
        {
            goal_state_ptr_ = state_ptr;
            
            // Reconstruct and return path
            constructPlan(state_ptr);   
            terminate_ = true;
            lock_.unlock();
            return;
        }        
    }
}

double RrtPlanner::getRandomNumberBetween(double min, double max)
{
    mt19937 gen(random_device_());
    uniform_real_distribution<double> distr(min, max);
    return distr(gen);
}

StateVarsType RrtPlanner::sampleSateUniform()
{

}

StateVarsType RrtPlanner::sampleState()
{
    double r = getRandomNumberBetween(0,1);

    StateVarsType sampled_state;
    if (r < planner_params_["goal_bias_probability"])
    {
        sampled_state = goal_state_ptr_->GetStateVars();
    }
    else
    {           
        sampled_state = sampleSateUniform();
    }

    return sampled_state;

}

double RrtPlanner::wrapAngle(double angle)
{
    // normalize to [-2*pi, 2*pi] range
    if (std::fabs(angle) > 2.0 * M_PI) {
        angle = std::fmod(angle, 2.0 * M_PI);
    }

    if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    return angle;
}

double RrtPlanner::angleDifference(double angle1, double angle2)
{
    return wrapAngle(angle1-angle2);
}

double RrtPlanner::calculateDistance(const StateVarsType& state_1, const StateVarsType& state_2)
{
    double dist=0;
    for (int i = 0; i < state_1.size(); ++i)
    {
        dist += pow(angleDifference(state_1[i],state_2[i]),2);
    }
    dist = sqrt(dist);
    return dist;
}

StatePtrType RrtPlanner::getNearestNeighbor(const StateVarsType& sampled_state)
{
    lock_.lock();
    
    StatePtrType nearest_state;
    auto min_d = DINF;
    for (auto& [state_id, state_ptr] : state_map_)
    {
        auto d = calculateDistance(state_ptr->GetStateVars(), sampled_state);
        if (d < min_d)
        {
            min_d = d;
            nearest_state = state_ptr;
        }
    }
    
    lock_.unlock();

    return nearest_state;
}

bool RrtPlanner::isValidConfiguration(const StateVarsType& state_vars, int thread_id)
{
    // return actions_ptrs_->isCollisionFree(state_vars, thread_id);
}

StatePtrType RrtPlanner::extend(const StatePtrType& nearest_neighbor, const StateVarsType& sampled_state, 
    bool& is_collision, int thread_id)
{
    int ndof = nearest_neighbor->GetStateVars().size();
    auto nearest_state_vars = nearest_neighbor->GetStateVars();
    // if sampledNode is closer than m_eps, return that 
    if (calculateDistance(sampled_state, nearest_state_vars) < planner_params_["eps"])
    {
        return constructState(sampled_state);
    }

    double angle_diff_norm = 0;
    for (int i = 0; i < ndof; ++i)
    {
        double diff = angleDifference(sampled_state[i], nearest_state_vars[i]);
        angle_diff_norm += pow(diff,2);
    }
    angle_diff_norm = sqrt(angle_diff_norm);

    StateVarsType final_state(ndof, 0);
    for (int i = 0; i < ndof; ++i)
    {
        final_state[i] = nearest_state_vars[i] + 
        planner_params_["eps"]*(angleDifference(sampled_state[i],nearest_state_vars[i])/angle_diff_norm); 
    }

    StateVarsType curr_state = nearest_state_vars;
    StateVarsType final_valid_state = nearest_state_vars;

    int num_samples = planner_params_["num_samples"];

    for (int i = 0; i < num_samples; ++i)
    {
        for (int j = 0; j < ndof; ++j)
        {
            curr_state[j] = nearest_state_vars[j] + ((double)(i)/(num_samples-1))*angleDifference(final_state[j],nearest_state_vars[j]);/*/angleDiffNorm);*/
        }

        if(!isValidConfiguration(curr_state, thread_id))
        {
            is_collision = true;
            break;
        }

        final_valid_state = curr_state;
    }
    
    return constructState(final_valid_state); 
}

void RrtPlanner::exit()
{
    bool all_rrt_threads_terminated = false;
    while (!all_rrt_threads_terminated)
    {
        all_rrt_threads_terminated = true;
        for (auto& fut : rrt_futures_)
        {
            if (!isFutureReady(fut))
            {
                all_rrt_threads_terminated = false;
                break;
            }
        }
    }
    rrt_futures_.clear();
    
    Planner::exit();
}
