#ifndef DUMMY_OPT_HPP
#define DUMMY_OPT_HPP

#include <common/typedefs.h>

namespace ps
{

    template<typename EnvType>
    class DummyOpt
    {
        typedef MatDf TrajType;

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
                if (env_.isFeasible(soln_traj))
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
            TrajType soln_traj;
            double len1 = (traj1.rightCols(1)-traj1.leftCols(1)).norm();
            double len2 = (traj2.rightCols(1)-traj2.leftCols(1)).norm();

            double alpha = len1/(len1+len2);

            VecDf opt_point = alpha*traj1.leftCols(1) + (1-alpha)*traj2.rightCols(1);

            int N = ceil((opt_point - traj1.rightCols(1)).norm()/conv_delta_);
            for (double i=0.0; i<=1.0; i+=1.0/N)
            {
                VecDf next_point = (1-i)*traj1.rightCols(1) + i*opt_point;
                TrajType new_traj1 = optimize(traj1.leftCols(1), next_point);
                TrajType new_traj2 = optimize(next_point, traj2.rightCols(1));

                if (env_.isFeasible(new_traj1) && env_.isFeasible(new_traj2))
                {
                    soln_traj.resize(new_traj1.rows(), new_traj1.cols()+new_traj2.cols());
                    soln_traj << new_traj1, new_traj2;
                }
                else
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

        EnvType env_;

        InterpMode intp_;

        double wp_delta_;
        double conv_delta_;
    };

}

#endif
