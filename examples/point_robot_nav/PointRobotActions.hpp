#ifndef POINT_ROBOT_ACTION_HPP
#define POINT_ROBOT_ACTION_HPP

#include <Action.hpp>

namespace epase
{

class PointRobotAction : public Action
{

public:
    PointRobotAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): Action(type, params), map_(map) {};
    ActionSuccessor Apply(StateVarsType state_vars, int thread_id); 
    bool CheckPreconditions(StateVarsType state); 
protected:
    bool isValidCell(int x, int y);
    bool inRange(int x, int y);
    std::vector<std::pair<int, int>> getFootPrintRectangular(int x, int y, int footprint_size);
    std::vector<double> move_dir_;
    std::vector<std::vector<int>> map_;
    std::vector<std::pair<int, int>> footprint_;
};

class MoveUpAction : public PointRobotAction
{

public:
    MoveUpAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): PointRobotAction(type, params, map)
    {
        move_dir_ = {0, 1};
    };
};

class MoveUpRightAction : public PointRobotAction
{

public:
    MoveUpRightAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): PointRobotAction(type, params, map)
    {
        move_dir_ = {1, 1};
    };
};

class MoveRightAction : public PointRobotAction
{

public:
    MoveRightAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): PointRobotAction(type, params, map)
    {
        move_dir_ = {1, 0};
    };
};

class MoveRightDownAction : public PointRobotAction
{

public:
    MoveRightDownAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): PointRobotAction(type, params, map)
    {
        move_dir_ = {1, -1};
    };
};


class MoveDownAction : public PointRobotAction
{

public:
    MoveDownAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): PointRobotAction(type, params, map)
    {
        move_dir_ = {0, -1};
    };
};

class MoveDownLeftAction : public PointRobotAction
{

public:
    MoveDownLeftAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): PointRobotAction(type, params, map)
    {
        move_dir_ = {-1, -1};
    };
};


class MoveLeftAction : public PointRobotAction
{

public:
    MoveLeftAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): PointRobotAction(type, params, map)
    {
        move_dir_ = {-1, 0};
    };
};


class MoveLeftUpAction : public PointRobotAction
{

public:
    MoveLeftUpAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map): PointRobotAction(type, params, map)
    {
        move_dir_ = {-1, 1};
    };
};

}

#endif
