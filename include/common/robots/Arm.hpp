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
        VecDf min_q_, max_q_, min_dq_, max_dq_;
    };
}

#endif //ARM_HPP
