/*
 * Copyright (c) 2023, Ramkumar Natarajan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   DummmyOpt.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   2/26/23
 */

#include <planners/insat/opt/DummyOpt.hpp>

namespace ps
{

    DummyOpt::DummyOpt(DummyOpt::InterpMode intp, double wp_delta, double conv_delta) : intp_(intp),
                                                                                        wp_delta_(wp_delta),
                                                                                        conv_delta_(conv_delta)
    {}

    TrajType DummyOpt::optimize(const InsatAction *act, const VecDf &s1, const VecDf &s2, int thread_id) {
        double dist = (s2-s1).norm();
        int N = ceil(dist/wp_delta_)+1;
        return optimize(act, s1, s2, N, thread_id);
    }

    TrajType DummyOpt::optimize(const InsatAction *act, const VecDf &s1, const VecDf &s2, int N, int thread_id) {
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

    TrajType DummyOpt::warmOptimize(const InsatAction *act, const TrajType &traj1, const TrajType &traj2, int thread_id) {
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

    TrajType DummyOpt::warmOptimize(const InsatAction *act, const TrajType &traj, int thread_id) {
        assert(traj.disc_traj_.cols() >= 2);

        TrajType t1, t2;
        t1.disc_traj_ = traj.disc_traj_.col(1);
        t2.disc_traj_ = traj.disc_traj_.rightCols(1);

        return warmOptimize(act, t1, t2, thread_id);
    }

    double DummyOpt::calculateCost(const TrajType &traj) {
        auto& disc_traj = traj.disc_traj_;
        double cost = 0;
        for (int i=0; i<disc_traj.cols()-1; ++i)
        {
            cost += (disc_traj.col(i+1)-disc_traj.col(i)).norm();
        }
        return cost;
    }

    MatDf DummyOpt::linInterp(const VecDf &p1, const VecDf &p2, int N) {
        MatDf traj(p1.size(), N);

        for (int i=0.0; i<N; ++i)
        {
            double j = i/static_cast<double>(N);
            traj.col(i) = p1*(1-j) + p2*j;
        }
        traj.rightCols(1) = p2;

        return traj;
    }
}