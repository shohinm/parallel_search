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
                 double conv_delta= 1e-1) : intp_(intp),
                                            wp_delta_(wp_delta),
                                            conv_delta_(conv_delta)
        {}

        virtual ~DummyOpt() {}

        void setEnv(std::vector<std::shared_ptr<Action>>& env)
        {
            for (auto& e : env)
            {
                env_.emplace_back(std::dynamic_pointer_cast<InsatAction>(e));
            }
        }

        virtual TrajType optimize(const VecDf& s1, const VecDf& s2)
        {
            double dist = (s2-s1).norm();
            int N = ceil(dist/wp_delta_)+1;
            return optimize(s1, s2, N);
        }

        virtual TrajType optimize(const VecDf& s1, const VecDf& s2, int N, int thread_id=0)
        {
            assert(N>=2);

            if (intp_ == InterpMode::LINEAR)
            {
                TrajType soln_traj = linInterp(s1, s2, N);
                if (env_[thread_id]->isFeasible(soln_traj))
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

        virtual TrajType warmOptimize(const TrajType& traj1, const TrajType & traj2, int thread_id=0)
        {
            int N = traj1.cols()+traj2.cols();
            TrajType init_traj(traj1.rows(), N);
            TrajType opt_traj(traj1.rows(), N);
            TrajType soln_traj(traj1.rows(), N);

            init_traj << traj1, traj2;
            opt_traj = linInterp(traj1.leftCols(1), traj2.rightCols(1), N);

            for (double i=0.0; i<=1.0; i+=1.0/conv_delta_)
            {
              soln_traj = (1-i)*init_traj + i*opt_traj;
              if (!env_[thread_id]->isFeasible(soln_traj))
              {
                break;
              }
            }
            return soln_traj;
        }

        virtual double calculateCost(const TrajType& traj)
        {
            double cost = 0;
            for (int i=0; i<traj.cols()-1; ++i)
            {
                cost += (traj.col(i+1)-traj.col(i)).norm();
            }
            return cost;
        }

        protected:

        TrajType linInterp(const VecDf& p1, const VecDf& p2, int N)
        {
            TrajType traj(p1.size(), N);

            for (int i=0.0; i<N; ++i)
            {
                double j = i/static_cast<double>(N);
                traj.col(i) = p1*(1-j) + p2*j;
            }
            traj.rightCols(1) = p2;

            return traj;
        }

        std::vector<std::shared_ptr<InsatAction>> env_;

        InterpMode intp_;

        double wp_delta_;
        double conv_delta_;
    };

}

#endif
