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

        virtual TrajType optimize(const InsatAction* act, const VecDf& s1, const VecDf& s2, int thread_id)
        {
            double dist = (s2-s1).norm();
            int N = ceil(dist/wp_delta_)+1;
            return optimize(act, s1, s2, N, thread_id);
        }

        virtual TrajType optimize(const InsatAction* act, const VecDf& s1, const VecDf& s2, int N, int thread_id)
        {
            assert(N>=2);

            TrajType traj;
            if (intp_ == InterpMode::LINEAR)
            {
                traj.disc_traj_ = linInterp(s1, s2, N);
                if (act->isFeasible(traj.disc_traj_, thread_id))
                {
                    return traj;
                }
                else
                {
                    TrajType empty_traj;
                    return empty_traj;
                }
            }
        }

        virtual TrajType warmOptimize(const InsatAction* act, const TrajType& traj1, const TrajType & traj2, int thread_id)
        {
            int N = traj1.disc_traj_.cols()+traj2.disc_traj_.cols();
            MatDf init_traj(traj1.disc_traj_.rows(), N);
            MatDf opt_traj(traj1.disc_traj_.rows(), N);
            MatDf soln_traj(traj1.disc_traj_.rows(), N);

            init_traj << traj1.disc_traj_, traj2.disc_traj_;
            opt_traj = linInterp(traj1.disc_traj_.leftCols(1), traj2.disc_traj_.rightCols(1), N);

            for (double i=0.0; i<=1.0; i+=1.0/conv_delta_)
            {
              soln_traj = (1-i)*init_traj + i*opt_traj;
              if (!act->isFeasible(soln_traj, thread_id))
              {
                break;
              }
            }
            TrajType traj;
            traj.disc_traj_ = soln_traj;
            return traj;
        }

        virtual TrajType warmOptimize(const InsatAction* act, const TrajType& traj, int thread_id)
        {
            assert(traj.disc_traj_.cols() >= 2);

            TrajType t1, t2;
            t1.disc_traj_ = traj.disc_traj_.col(1);
            t2.disc_traj_ = traj.disc_traj_.rightCols(1);

            return warmOptimize(act, t1, t2, thread_id);
        }

        virtual double calculateCost(const TrajType& traj)
        {
            auto& disc_traj = traj.disc_traj_;
            double cost = 0;
            for (int i=0; i<disc_traj.cols()-1; ++i)
            {
                cost += (disc_traj.col(i+1)-disc_traj.col(i)).norm();
            }
            return cost;
        }

        protected:

        MatDf linInterp(const VecDf& p1, const VecDf& p2, int N)
        {
            MatDf traj(p1.size(), N);

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
