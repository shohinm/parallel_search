#ifndef INSAT_NAV2D_ACTION_HPP
#define INSAT_NAV2D_ACTION_HPP

#include <common/Types.hpp>
#include <common/insat/InsatAction.hpp>
#include "planners/insat/opt/DummyOpt.hpp"

namespace ps
{

    class InsatNav2dAction : public InsatAction
    {

    public:

        typedef std::shared_ptr<InsatNav2dAction> Ptr;
        typedef DummyOpt OptType;
        typedef std::vector<OptType> OptVecType;
        typedef std::shared_ptr<OptVecType> OptVecPtrType;

        friend class DummyOpt;

        InsatNav2dAction(const std::string& type,
                         ParamsType params,
                         std::vector<std::vector<int>>& map,
                         OptVecPtrType& opt,
                         bool is_expensive = true);

        virtual bool CheckPreconditions(StateVarsType state);
        ActionSuccessor GetSuccessor(StateVarsType state_vars, int thread_id);
        ActionSuccessor GetSuccessorLazy(StateVarsType state_vars, int thread_id);
        ActionSuccessor Evaluate(StateVarsType parent_state_vars, StateVarsType child_state_vars, int thread_id=0);
        bool isValidCell(int x, int y) const;
        bool inRange(int x, int y);
        std::vector<std::pair<int, int>> getFootPrintRectangular(int x, int y, int footprint_size);

        // INSAT
        void setOpt(OptVecPtrType& opt);
        bool isFeasible(MatDf& traj) const;
        TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const;
        TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const;
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const;
        double getCost(const TrajType& traj, int thread_id=0) const;

    protected:

        std::vector<double> move_dir_;
        std::vector<std::vector<int>> map_;
        std::vector<std::pair<int, int>> footprint_;
        LockType lock_;
        OptVecPtrType opt_;

    };

    class IMoveUpAction : public InsatNav2dAction
    {

    public:
        IMoveUpAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptVecPtrType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {0, 1};
        };

      void setOpt(OptVecPtrType& opt) {InsatNav2dAction::setOpt(opt);};
      bool isFeasible(MatDf& traj) const {return InsatNav2dAction::isFeasible(traj);};
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
      {return InsatNav2dAction::optimize(s1, s2, thread_id);}
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
      {return InsatNav2dAction::warmOptimize(t1, t2, thread_id);}
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const
        {return InsatNav2dAction::warmOptimize(t, thread_id);}
      double getCost(const TrajType& traj, int thread_id=0) const
      {return InsatNav2dAction::getCost(traj, thread_id);}
    };

    class IMoveUpRightAction : public InsatNav2dAction
    {

    public:
        IMoveUpRightAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptVecPtrType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {1, 1};
        };

      void setOpt(OptVecPtrType& opt) {InsatNav2dAction::setOpt(opt);};
      bool isFeasible(MatDf& traj) const {return InsatNav2dAction::isFeasible(traj);};
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
      {return InsatNav2dAction::optimize(s1, s2, thread_id);}
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
      {return InsatNav2dAction::warmOptimize(t1, t2, thread_id);}
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const
        {return InsatNav2dAction::warmOptimize(t, thread_id);}
      double getCost(const TrajType& traj, int thread_id=0) const
      {return InsatNav2dAction::getCost(traj, thread_id);}
    };

    class IMoveRightAction : public InsatNav2dAction
    {

    public:
        IMoveRightAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptVecPtrType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {1, 0};
        };

      void setOpt(OptVecPtrType& opt) {InsatNav2dAction::setOpt(opt);};
      bool isFeasible(MatDf& traj) const {return InsatNav2dAction::isFeasible(traj);};
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
      {return InsatNav2dAction::optimize(s1, s2, thread_id);}
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
      {return InsatNav2dAction::warmOptimize(t1, t2, thread_id);}
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const
        {return InsatNav2dAction::warmOptimize(t, thread_id);}
      double getCost(const TrajType& traj, int thread_id=0) const
      {return InsatNav2dAction::getCost(traj, thread_id);}
    };

    class IMoveRightDownAction : public InsatNav2dAction
    {

    public:
        IMoveRightDownAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptVecPtrType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {1, -1};
        };

      void setOpt(OptVecPtrType& opt) {InsatNav2dAction::setOpt(opt);};
      bool isFeasible(MatDf& traj) const {return InsatNav2dAction::isFeasible(traj);};
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
      {return InsatNav2dAction::optimize(s1, s2, thread_id);}
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
      {return InsatNav2dAction::warmOptimize(t1, t2, thread_id);}
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const
        {return InsatNav2dAction::warmOptimize(t, thread_id);}
      double getCost(const TrajType& traj, int thread_id=0) const
      {return InsatNav2dAction::getCost(traj, thread_id);}
    };


    class IMoveDownAction : public InsatNav2dAction
    {

    public:
        IMoveDownAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptVecPtrType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {0, -1};
        };

      void setOpt(OptVecPtrType& opt) {InsatNav2dAction::setOpt(opt);};
      bool isFeasible(MatDf& traj) const {return InsatNav2dAction::isFeasible(traj);};
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
      {return InsatNav2dAction::optimize(s1, s2, thread_id);}
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
      {return InsatNav2dAction::warmOptimize(t1, t2, thread_id);}
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const
        {return InsatNav2dAction::warmOptimize(t, thread_id);}
      double getCost(const TrajType& traj, int thread_id=0) const
      {return InsatNav2dAction::getCost(traj, thread_id);}
    };

    class IMoveDownLeftAction : public InsatNav2dAction
    {

    public:
        IMoveDownLeftAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptVecPtrType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {-1, -1};
        };

      void setOpt(OptVecPtrType& opt) {InsatNav2dAction::setOpt(opt);};
      bool isFeasible(MatDf& traj) const {return InsatNav2dAction::isFeasible(traj);};
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
      {return InsatNav2dAction::optimize(s1, s2, thread_id);}
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
      {return InsatNav2dAction::warmOptimize(t1, t2, thread_id);}
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const
        {return InsatNav2dAction::warmOptimize(t, thread_id);}
      double getCost(const TrajType& traj, int thread_id=0) const
      {return InsatNav2dAction::getCost(traj, thread_id);}
    };


    class IMoveLeftAction : public InsatNav2dAction
    {

    public:
        IMoveLeftAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptVecPtrType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {-1, 0};
        };

      void setOpt(OptVecPtrType& opt) {InsatNav2dAction::setOpt(opt);};
      bool isFeasible(MatDf& traj) const {return InsatNav2dAction::isFeasible(traj);};
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
      {return InsatNav2dAction::optimize(s1, s2, thread_id);}
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
      {return InsatNav2dAction::warmOptimize(t1, t2, thread_id);}
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const
        {return InsatNav2dAction::warmOptimize(t, thread_id);}
      double getCost(const TrajType& traj, int thread_id=0) const
      {return InsatNav2dAction::getCost(traj, thread_id);}
    };


    class IMoveLeftUpAction : public InsatNav2dAction
    {

    public:
        IMoveLeftUpAction(const std::string& type, ParamsType params, std::vector<std::vector<int>> map, OptVecPtrType opt, bool is_expensive = true):
                InsatNav2dAction(type, params, map, opt, is_expensive)
        {
            move_dir_ = {-1, 1};
        };

      void setOpt(OptVecPtrType& opt) {InsatNav2dAction::setOpt(opt);};
      bool isFeasible(MatDf& traj) const {return InsatNav2dAction::isFeasible(traj);};
      TrajType optimize(const StateVarsType& s1, const StateVarsType& s2, int thread_id=0) const
      {return InsatNav2dAction::optimize(s1, s2, thread_id);}
      TrajType warmOptimize(const TrajType& t1, const TrajType& t2, int thread_id=0) const
      {return InsatNav2dAction::warmOptimize(t1, t2, thread_id);}
        TrajType warmOptimize(const TrajType& t, int thread_id=0) const
        {return InsatNav2dAction::warmOptimize(t, thread_id);}
      double getCost(const TrajType& traj, int thread_id=0) const
      {return InsatNav2dAction::getCost(traj, thread_id);}
    };

}

#endif
