#ifndef DUMMY_OPT_HPP
#define DUMMY_OPT_HPP

#include <common/typedefs.h>

namespace ps
{

    template<typename EnvType>
    class DummyOpt
    {
        typedef typename EnvType::Ptr EnvPtrType;
        typedef typename EnvType::TrajType TrajType;

        enum InterpMode
        {
            LINEAR = 0,
            SPLINE
        };

        public:

        DummyOpt(EnvType& env,
                 InterpMode intp = InterpMode::LINEAR,
                 double wp_delta = 1e-1,
                 double conv_delta= 1e-1) : env_(env),
                                            intp_(intp),
                                            wp_delta_(wp_delta),
                                            conv_delta_(conv_delta)
        {}

        virtual ~DummyOpt() {}

        virtual TrajType optimize(VecDf& s1, VecDf& s2)
        {
            double dist = (s2-s1).norm();
            int N = ceil(dist/wp_delta_)+1;
            return optimize(s1, s2, N);
        }

        virtual TrajType optimize(VecDf& s1, VecDf& s2, int N)
        {
            assert(N>=2);

            if (intp_ == InterpMode::LINEAR)
            {
                TrajType soln_traj = linInterp(s1, s2, N);
                if (env_->isFeasible(soln_traj))
                {
                    return soln_traj;
                }
                else
                {
                    TrajType empty_traj;
                    return empty_traj;
                }
            }
        }

        virtual TrajType warmOptimize(TrajType& traj1, TrajType & traj2)
        {
            int N = traj1.cols()+traj2.cols();
            TrajType init_traj(traj1.rows(), N);
            TrajType opt_traj(traj1.rows(), N);
            TrajType soln_traj(traj1.rows(), N);

            init_traj << traj1, traj2;
            opt_traj = linInterp(traj1.leftCols(1), traj2.rightCols(1));

            for (double i=0.0; i<=1.0; i+=1.0/conv_delta_)
            {
              soln_traj = (1-i)*init_traj + i*opt_traj;
              if (!env_.isFeasible(soln_traj))
              {
                break;
              }
            }
            return soln_traj;
        }

        virtual double calculateCost(TrajType& traj)
        {
            double cost = 0;
            for (int i=0; i<traj.cols()-1; ++i)
            {
                cost += (traj.col(i+1)-traj.col(i)).norm();
            }
            return cost;
        }

        protected:

        TrajType linInterp(VecDf& p1, VecDf& p2, int N)
        {
            TrajType traj(p1.size(), N);

            for (double i=0.0; i<=1.0; i+=1.0/N)
            {
                traj.col(i) = p1*(1-i) + p2*i;
            }
            traj.rightCols(1) = p2;

            return traj;
        }

        EnvPtrType env_;

        InterpMode intp_;

        double wp_delta_;
        double conv_delta_;
    };

}

#endif
