//
// Created by cyz on 19-4-20.
//

#ifndef POINTCLOUDPROCESSING_SCALARCURVECONFIG_HPP
#define POINTCLOUDPROCESSING_SCALARCURVECONFIG_HPP

/*
 * ScalarCurveConfig.hpp
 *
 *  Created on: Mar 5, 2015
 *      Author: Paul Furgale, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

namespace curves {

    struct ScalarCurveConfig {
        typedef double ValueType;
        typedef double DerivativeType;
    };

} // namespace

#endif //POINTCLOUDPROCESSING_SCALARCURVECONFIG_HPP
