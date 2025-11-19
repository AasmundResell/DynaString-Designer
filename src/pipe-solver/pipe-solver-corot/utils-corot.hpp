#pragma once
#include "misc/config.hpp"
#include "misc/includes.hpp"

using Vec6 = Eigen::Vector<scalar, 6>;
using Vec7 = Eigen::Vector<scalar, 7>;
using Mat7 = Eigen::Matrix<scalar, 7, 7>;
using Vec12 = Eigen::Vector<scalar, 12>;
using Mat12 = Eigen::Matrix<scalar, 12, 12>;
using Mat3_12 = Eigen::Matrix<scalar, 3, 12>;
using Mat12_3 = Eigen::Matrix<scalar, 12, 3>;
using Mat6_12 = Eigen::Matrix<scalar, 6, 12>;
using Mat7_12 = Eigen::Matrix<scalar, 7, 12>;

namespace corot {

#define for_each_node_cpu(i) for (i = top_node; i < N; i++)

} // namespace corot