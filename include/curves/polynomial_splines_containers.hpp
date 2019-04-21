//
// Created by cyz on 19-4-15.
//

#ifndef POINTCLOUDPROCESSING_POLYNOMIAL_SPLINES_CONTAINERS_HPP
#define POINTCLOUDPROCESSING_POLYNOMIAL_SPLINES_CONTAINERS_HPP
/*
 * polynomial_splines_containers.hpp
 *
 *  Created on: Mar 7, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// curves
#include "curves/PolynomialSplineContainer.hpp"

namespace curves {

    using PolynomialSplineContainerLinear    = PolynomialSplineContainer<1>;
    using PolynomialSplineContainerQuadratic = PolynomialSplineContainer<2>;
    using PolynomialSplineContainerCubic     = PolynomialSplineContainer<3>;
    using PolynomialSplineContainerQuartic   = PolynomialSplineContainer<4>;
    using PolynomialSplineContainerQuintic   = PolynomialSplineContainer<5>;

}
#endif //POINTCLOUDPROCESSING_POLYNOMIAL_SPLINES_CONTAINERS_HPP
