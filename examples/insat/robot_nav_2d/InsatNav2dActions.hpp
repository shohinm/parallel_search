#ifndef INSAT_NAV2D_ACTION_HPP
#define INSAT_NAV2D_ACTION_HPP

#include <common/Types.hpp>
#include <common/insat/InsatAction.hpp>
#include "planners/insat/opt/dummy_opt.hpp"

namespace ps
{

    class InsatNav2dAction : public InsatAction
    {

    public:

        typedef std::shared_ptr<InsatNav2dAction> Ptr;
        typedef DummyOpt OptType;
        typedef std::shared_ptr<OptType> OptPtrType;
        typedef std::vector<OptPtrType> OptPtrVecType;

        InsatNav2dAction(const std::string& type,
                         ParamsType params,
                         std::vector<std::vector<int>>& map,
                         OptPtrVecType& opt,
                         bool is_expensive = true);

        virtual bool CheckPreconditions(StateVarsType state);
        ActionSuccessor GetSuccessor(StateVarsType state_vars, int thread_id);
        ActionSuccessor GetSuccessorLazy(StateVarsType state_vars, int thread_id);
        ActionSuccessor Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id=0);
        bool isValidCell(int x, int y);
        bool inRange(int x, int y);
        std::vector<std::pair<int, int>> getFootPrintRectangular(int x, int y, int footprint_size);

        // INSAT
        void setOpt(OptPtrVecType& opt);
        bool isFeasible(TrajType& traj);
        TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0);
        TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0);
        double getCost(const TrajType& traj, int thread_id=0);

    protected:

        std::vector<double> move_dir_;
        std::vector<std::vector<int>> map_;
        std::vector<std::pair<int, int>> footprint_;
        LockType lock_;
        OptPtrVecType opt_;

    };

    class IMoveUpAction : public InsatNav2dAction
    {

    public:
        IMoveUpAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptPtrVecType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {0, 1};
        };
    };

    class IMoveUpRightAction : public InsatNav2dAction
    {

    public:
        IMoveUpRightAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptPtrVecType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {1, 1};
        };
    };

    class IMoveRightAction : public InsatNav2dAction
    {

    public:
        IMoveRightAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptPtrVecType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {1, 0};
        };
    };

    class IMoveRightDownAction : public InsatNav2dAction
    {

    public:
        IMoveRightDownAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptPtrVecType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {1, -1};
        };
    };


    class IMoveDownAction : public InsatNav2dAction
    {

    public:
        IMoveDownAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptPtrVecType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {0, -1};
        };
    };

    class IMoveDownLeftAction : public InsatNav2dAction
    {

    public:
        IMoveDownLeftAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptPtrVecType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {-1, -1};
        };
    };


    class IMoveLeftAction : public InsatNav2dAction
    {

    public:
        IMoveLeftAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptPtrVecType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {-1, 0};
        };
    };


    class IMoveLeftUpAction : public InsatNav2dAction
    {

    public:
        IMoveLeftUpAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptPtrVecType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {-1, 1};
        };
    };

}

#endif
