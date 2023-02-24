//
// Created by Ramkumar Natarajan on 2/23/23.
//

#ifndef ABB_HPP
#define ABB_HPP

#include <common/Types.hpp>
#include <common/robots/Arm.hpp>

namespace ps
{
    struct IRB1600 : public Arm
    {
        IRB1600()
        {
            min_q_.resize(6);
            max_q_.resize(6);
            min_dq_.resize(6);
            max_dq_.resize(6);
            min_ddq_.resize(6);
            max_ddq_.resize(6);

            min_q_ << -3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813;
            max_q_ << 3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813;
            min_dq_ << -2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854;
            max_dq_ << 2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854;
            min_ddq_ = -100*VecDf::Ones(6);
            max_ddq_ = 100*VecDf::Ones(6);
        }
    };
}

#endif //ABB_HPP
