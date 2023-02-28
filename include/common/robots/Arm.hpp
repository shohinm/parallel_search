//
// Created by Ramkumar Natarajan on 2/23/23.
//

#ifndef ARM_HPP
#define ARM_HPP

#include <common/Types.hpp>

namespace ps
{
    struct Arm
    {
        VecDf min_q_, max_q_;
        VecDf min_dq_, max_dq_;
        VecDf min_ddq_, max_ddq_;
        VecDf min_dddq_, max_dddq_;

        double collision_delta_;
    };
}

#endif //ARM_HPP
