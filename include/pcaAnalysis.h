//
// Created by cyz on 18-10-1.
//

#ifndef POINTCLOUDPROCESSING_PCAANALYSIS_H
#define POINTCLOUDPROCESSING_PCAANALYSIS_H

#include "smoothing.h"

#include <math.h>
#include <Eigen/Dense>


class pcaAnalysist
{
    //特征值
    struct lamda
    {
        float la1;
        float la2;
        float la3;
    };

    //特征向量
    struct vec
    {
        Eigen::Vector3f v1 ;
        Eigen::Vector3f v2 ;
        Eigen::Vector3f v3 ;

    };

public:
    struct pcaFeature
    {
        lamda lamds;
        vec vecs;
        int ptID;
        int ptNum;///num of neighbor pts

        //structure
        float lineStruc;
        float planeStruc;
        float scatterStruc;
    };

    bool calculatePCAofPointCloud(pcXYZIptr inputCloud, float radius,
                                  std::vector<pcaFeature> &pcaFeaofCloud);

    bool calculatePCAofPoint(pcXYZIptr inputCloud, int i,
                             std::vector<int> neighborIndices,
                             pcaFeature &ptFeature);

    bool calculatePCAofPoint(pcXYZIptr inputCloud, pcl::PointXYZI centro,
                                           std::vector<int> neighborIndices,
                                           pcaAnalysist::pcaFeature &ptFeature);

    bool getMiniBoundingBox(pcXYZIptr inputCloud);

};

#endif //POINTCLOUDPROCESSING_PCAANALYSIS_H
