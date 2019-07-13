//
// Created by cyz on 18-10-27.
//

#ifndef POINTCLOUDPROCESSING_LIDARODOMETRY_H
#define POINTCLOUDPROCESSING_LIDARODOMETRY_H

#include "smoothing.h"

namespace lidarOdometry
{

    struct timeStamp
    {
        timeStamp(long int sec_, long int usec_):sec(sec_), usec(usec_){}
        long int sec;
        long int usec;

        long int operator - (const timeStamp& rhs)
        {
            return (sec-rhs.sec)*1000000 + usec-rhs.usec;
        }
    };


    class lidarOdometryClass
    {

    public:
        bool getRangeAndptIdImage(pcXYZIptr inputCloud, Eigen::MatrixXf &rangeImg, Eigen::MatrixXf &ptIdImg);

        bool calculateSmoothness(Eigen::MatrixXf rangeImg, Eigen::MatrixXf &ptTypeImg);

        bool calculateCurvature(Eigen::MatrixXf ptIDimg);

        bool readposefile(std::string posfile, std::vector<timeStamp> &postimeStamps,
                          std::vector<Eigen::Vector3d> &translation,std::vector<Eigen::Quaterniond> &rotations);

        bool vector2pointcloudXYZ(std::vector<Eigen::Vector3d> vecPoints, pcXYZptr pcPoints);

        bool linefitting(pcXYZ inputCloud, std::vector<std::vector<int>> &linesIndices);
    };

}
#endif //POINTCLOUDPROCESSING_LIDARODOMETRY_H
