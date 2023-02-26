#ifndef DUMMY_OPT_HPP
#define DUMMY_OPT_HPP

#include <common/Types.hpp>
#include <common/insat/InsatAction.hpp>

namespace ps
{

    class DummyOpt
    {
        public:

        typedef std::shared_ptr<DummyOpt> Ptr;

        enum InterpMode
        {
            LINEAR = 0,
            SPLINE
        };

        DummyOpt(InterpMode intp = InterpMode::LINEAR,
                 double wp_delta = 1e-1,
                 double conv_delta= 1e-1);

        virtual ~DummyOpt() {}

        virtual TrajType optimize(const InsatAction* act, const VecDf& s1, const VecDf& s2, int thread_id);

        virtual TrajType optimize(const InsatAction* act, const VecDf& s1, const VecDf& s2, int N, int thread_id);

        virtual TrajType warmOptimize(const InsatAction* act, const TrajType& traj1, const TrajType & traj2, int thread_id);

        virtual TrajType warmOptimize(const InsatAction* act, const TrajType& traj, int thread_id);

        virtual double calculateCost(const TrajType& traj);

        protected:

        MatDf linInterp(const VecDf& p1, const VecDf& p2, int N);

        std::vector<std::shared_ptr<InsatAction>> env_;

        InterpMode intp_;

        double wp_delta_;
        double conv_delta_;
    };

}

#endif
