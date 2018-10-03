//
// Created by cyz on 18-10-1.
//

#include "pcaAnalysis.h"

#include <Eigen/Core>
#include <pcl/common/pca.h>
#include <pcl/PointIndices.h>

bool pcaAnalysist::calculatePCAofPointCloud(pcXYZIptr inputCloud, float radius,
                                            std::vector<pcaAnalysist::pcaFeature> &pcaFeaofCloud)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdTreeFla;
    kdTreeFla.setInputCloud(inputCloud);
    pcaFeature tempFea;

    for(size_t i=0 ; i<inputCloud->points.size() ; i++)
    {
        std::vector<int> neighborIndices;
        std::vector<float> neighborDists;
        std::vector<int>().swap(neighborIndices) ;
        std::vector<float>().swap(neighborDists) ;

        kdTreeFla.radiusSearch(i, radius, neighborIndices, neighborDists);
        tempFea.ptID = i;
        tempFea.ptNum = neighborIndices.size();
        calculatePCAofPoint(inputCloud, neighborIndices, tempFea);
        try
        {
            tempFea.lineStruc =( sqrt(tempFea.lamds.la1) - sqrt(tempFea.lamds.la2) ) / sqrt(tempFea.lamds.la1);
            tempFea.planeStruc =( sqrt(tempFea.lamds.la2) - sqrt(tempFea.lamds.la3) )/sqrt(tempFea.lamds.la1);
            tempFea.scatterStruc = sqrt(tempFea.lamds.la3) / sqrt(tempFea.lamds.la1);
        }
        catch (float)
        {
            std::cout<<"Invalid eigen values!"<<endl;
        }

        pcaFeaofCloud.push_back(tempFea);
    }

}

bool pcaAnalysist::calculatePCAofPoint(pcXYZIptr inputCloud, std::vector<int> neighborIndices,
                                       pcaAnalysist::pcaFeature &ptFeature)
{
    if(ptFeature.ptNum < 10)
        return false;

    pcl::PointIndices ptIndice;
    ptIndice.indices.swap(neighborIndices);
    pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices(ptIndice));
//    indicesPtr->swap(neighborIndices);
    pcl::PCA<pcl::PointXYZI> pcaFunc;///构造PCA函数
    pcaFunc.setInputCloud(inputCloud);
    pcaFunc.setIndices(indicesPtr);

    Eigen::Vector3f eigenValues = pcaFunc.getEigenValues();
    Eigen::Matrix3f eigenVecs = pcaFunc.getEigenVectors();

    ptFeature.lamds.la1 = eigenValues(0,0);
    ptFeature.lamds.la2 = eigenValues(1,0);
    ptFeature.lamds.la3 = eigenValues(2,0);

    ptFeature.vecs.v1(0,0) = eigenVecs(0,0);
    ptFeature.vecs.v1(1,0) = eigenVecs(1,0);
    ptFeature.vecs.v1(2,0) = eigenVecs(2,0);

    ptFeature.vecs.v2(0,0) = eigenVecs(0,1);
    ptFeature.vecs.v2(1,0) = eigenVecs(1,1);
    ptFeature.vecs.v2(2,0) = eigenVecs(2,1);

    ptFeature.vecs.v3(0,0) = eigenVecs(0,2);
    ptFeature.vecs.v3(1,0) = eigenVecs(1,2);
    ptFeature.vecs.v3(2,0) = eigenVecs(2,2);
}

bool pcaAnalysist::getMiniBoundingBox(pcXYZIptr inputCloud)
{
    Eigen::Vector4f pcaCentroid;///齐次坐标
    pcl::compute3DCentroid(*inputCloud, pcaCentroid);

    Eigen::Matrix3f cov;///covariance Matrix
    pcl::computeCovarianceMatrixNormalized(*inputCloud, pcaCentroid, cov);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSovr(cov, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f pcaVecs = eigenSovr.eigenvectors();
    pcaVecs.col(2) = pcaVecs.col(0).cross(pcaVecs.col(1));
//    pcaVecs.col(0) = pcaVecs.col(1).cross(pcaVecs.col(2));
//    pcaVecs.col(1) = pcaVecs.col(2).cross(pcaVecs.col(0));

    Eigen::Matrix4f transMatrix = Eigen::Matrix4f::Identity();///transformation Matrix
    transMatrix.block<3,3>(0,0) = pcaVecs.transpose();
    transMatrix.block<3,1>(0,3) = -1.f * (transMatrix.block<3,3>(0,0) * pcaCentroid.head<3>());

    pcXYZIptr transdCloud;
    pcl::transformPointCloud(*inputCloud, *transdCloud, transMatrix);

    pcl::PointXYZI maxPt, minPt;
    pcl::getMinMax3D(*transdCloud, minPt, maxPt);
    const Eigen::Vector3f meanDiag = 0.5f * (maxPt.getVector3fMap() + minPt.getVector3fMap());///型心
    
}