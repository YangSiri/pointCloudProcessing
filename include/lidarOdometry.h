//
// Created by cyz on 18-10-27.
//

#ifndef POINTCLOUDPROCESSING_LIDARODOMETRY_H
#define POINTCLOUDPROCESSING_LIDARODOMETRY_H

#include "smoothing.h"

namespace lidarOdometry
{
//    std::vector<Eigen::MatrixXf> rangeImgs;

    class lidarOdometryClass
    {

    public:
        bool getRangeAndptIdImage(pcXYZIptr inputCloud, Eigen::MatrixXf &rangeImg, Eigen::MatrixXf &ptIdImg);

        bool calculateSmoothness(Eigen::MatrixXf rangeImg, Eigen::MatrixXf &ptTypeImg);

        bool calculateCurvature(Eigen::MatrixXf ptIDimg);
    };

}
#endif //POINTCLOUDPROCESSING_LIDARODOMETRY_H
