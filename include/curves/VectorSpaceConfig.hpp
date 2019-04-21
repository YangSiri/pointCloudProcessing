//
// Created by cyz on 19-4-15.
//

#ifndef POINTCLOUDPROCESSING_VECTORSPACECONFIG_HPP
#define POINTCLOUDPROCESSING_VECTORSPACECONFIG_HPP

/*
 * ScalarCurveConfig.hpp
 *
 *  Created on: Mar 5, 2015
 *      Author: Paul Furgale, Renaud Dube, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <Eigen/Core>

namespace curves {

    template <int N>
    struct VectorSpaceConfig {
        typedef Eigen::Matrix<double,N,1> ValueType;
        typedef Eigen::Matrix<double,N,1> DerivativeType;
    };

} // namespace
#endif //POINTCLOUDPROCESSING_VECTORSPACECONFIG_HPP
