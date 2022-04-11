#ifndef POINT_ROBOT_ACTION_HPP
#define POINT_ROBOT_ACTION_HPP

#include <Action.hpp>

namespace epase
{

class PointRobotAction : public Action
{

public:
    PointRobotAction(const std::string& type, ParamsType params): Action(type, params) {};
    ActionSuccessor Apply(StateVarsType state); 
    bool CheckPreconditions(StateVarsType state); 
protected:
    std::vector<double> move_dir_;

};

class MoveUpAction : public PointRobotAction
{

public:
    MoveUpAction(const std::string& type, ParamsType params): PointRobotAction(type, params)
    {
        move_dir_ = {0, 1};
    };
};

class MoveUpRightAction : public PointRobotAction
{

public:
    MoveUpRightAction(const std::string& type, ParamsType params): PointRobotAction(type, params)
    {
        move_dir_ = {1, 1};
    };
};

class MoveRightAction : public PointRobotAction
{

public:
    MoveRightAction(const std::string& type, ParamsType params): PointRobotAction(type, params)
    {
        move_dir_ = {1, 0};
    };
};

class MoveRightDownAction : public PointRobotAction
{

public:
    MoveRightDownAction(const std::string& type, ParamsType params): PointRobotAction(type, params)
    {
        move_dir_ = {1, -1};
    };
};


class MoveDownAction : public PointRobotAction
{

public:
    MoveDownAction(const std::string& type, ParamsType params): PointRobotAction(type, params)
    {
        move_dir_ = {0, -1};
    };
};

class MoveDownLeftAction : public PointRobotAction
{

public:
    MoveDownLeftAction(const std::string& type, ParamsType params): PointRobotAction(type, params)
    {
        move_dir_ = {-1, -1};
    };
};


class MoveLeftAction : public PointRobotAction
{

public:
    MoveLeftAction(const std::string& type, ParamsType params): PointRobotAction(type, params)
    {
        move_dir_ = {-1, 0};
    };
};


class MoveLeftUpAction : public PointRobotAction
{

public:
    MoveLeftUpAction(const std::string& type, ParamsType params): PointRobotAction(type, params)
    {
        move_dir_ = {-1, 1};
    };
};

}

#endif
