//
// Created by cyz on 19-4-15.
//

#ifndef POINTCLOUDPROCESSING_POLYNOMIAL_SPLINES_HPP
#define POINTCLOUDPROCESSING_POLYNOMIAL_SPLINES_HPP
/*
*
* polynomial_splines.hpp
        *
        *  Created on: Mar 7, 2017
*      Author: Dario Bellicoso
*/

#pragma once

// curves
#include "curves/PolynomialSpline.hpp"

namespace curves {

    using PolynomialSplineQLinear   = PolynomialSpline<1>;
    using PolynomialSplineQuadratic = PolynomialSpline<2>;
    using PolynomialSplineCubic     = PolynomialSpline<3>;
    using PolynomialSplineQuartic   = PolynomialSpline<4>;
    using PolynomialSplineQuintic   = PolynomialSpline<5>;

}

#endif //POINTCLOUDPROCESSING_POLYNOMIAL_SPLINES_HPP
