//
// Created by cyz on 18-10-13.
//

#include "buildingFacadeExtraction.h"

bool buildingFacadeExtractor::groundFilter(pcXYZIptr origiCloud, pcXYZIptr nonGroundCloud)
{
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> groundFilter;
    groundFilter.setInputCloud(origiCloud);
    groundFilter.setMaxWindowSize(20);
    groundFilter.setSlope(1.0f);
    groundFilter.setMaxDistance(4.0f);
    groundFilter.setInitialDistance(3.0f);
    groundFilter.extract(ground->indices);

    pcl::ExtractIndices<pcl::PointXYZI> extractor;
    extractor.setInputCloud(origiCloud);
    extractor.setIndices(ground);
    extractor.setNegative(true);
    extractor.filter(*nonGroundCloud);

    return true;
}

bool buildingFacadeExtractor::planeProjection(pcXYZIptr inputCloud, pcl::ModelCoefficients::Ptr paras,
                                              pcXYZIptr outputCloud)
{
    pcl::ProjectInliers<pcl::PointXYZI> prjector;
    prjector.setInputCloud(inputCloud);
    prjector.setModelType(pcl::SACMODEL_PLANE);
    prjector.setModelCoefficients(paras);
    prjector.filter(*outputCloud);

    return true;
}

bool buildingFacadeExtractor::meanShiftClustering(pcXYZIptr inputcloud, int iterTimes)
{
    pcXYZI vecCloud;//储存每一点的mean shift vector
    pcl::PointXYZI tmp;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdTreeFla;
    kdTreeFla.setInputCloud(inputcloud);

    std::vector<int>ptIndces;
    std::vector<float >ptDist;

    Eigen::MatrixXf meanShiftVec_of_NeighborPt;

    int times = 0;
    do {
        //for every point in the original cloud
        for (int i = 0; i < inputcloud->points.size(); i++)
        {
            std::vector<int>().swap(ptIndces);
            std::vector<float>().swap(ptDist);
            tmp.x = 0;
            tmp.y = 0;
            tmp.z = 0;

            kdTreeFla.radiusSearch(i, 2.0, ptIndces, ptDist);
            if (ptIndces.size() < 15)
            {
                vecCloud.push_back(tmp);
                continue;
            }

            meanShiftVec_of_NeighborPt.resize(3, ptIndces.size());

            //for all neighbor pts
            for (int j = 0; j < ptIndces.size(); j++)
            {
                meanShiftVec_of_NeighborPt(0, j) = inputcloud->points[ptIndces[j]].x - inputcloud->points[i].x;
                meanShiftVec_of_NeighborPt(1, j) = inputcloud->points[ptIndces[j]].y - inputcloud->points[i].y;
                meanShiftVec_of_NeighborPt(2, j) = inputcloud->points[ptIndces[j]].z - inputcloud->points[i].z;

                tmp.x += meanShiftVec_of_NeighborPt(0, j);
                tmp.y += meanShiftVec_of_NeighborPt(1, j);
                tmp.z += meanShiftVec_of_NeighborPt(2, j);

                std::cout<<"---------x__"<<tmp.x;
                std::cout<<"---------y__"<<tmp.y;
                std::cout<<"---------z__"<<tmp.z<<endl;

            }
            vecCloud.push_back(tmp);//save mean shift vector
        }

        //update every point based on mean shift vector
        for (int i=0 ; i<vecCloud.points.size() ; i++)
        {
            inputcloud->points[i].x += vecCloud.points[i].x;
            inputcloud->points[i].y += vecCloud.points[i].y;
            inputcloud->points[i].z += vecCloud.points[i].z;
        }

        times++;
        vecCloud.clear();

    }while(times < iterTimes);

    return true;

}

bool buildingFacadeExtractor::constructVoxels(pcXYZIptr inputCloud, float gridResolution,
                                              buildingFacadeExtractor::voxel* voxels_of_Cloud)
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);

    int colsX = std::ceil((maxPt.x - minPt.x) / gridResolution);
    int colsY = std::ceil((maxPt.y - minPt.y) / gridResolution);
    int colsZ = std::ceil((maxPt.z - minPt.z) / gridResolution);

    voxels_of_Cloud = new voxel[colsX*colsY*colsZ];

    for(int i=0 ; i<inputCloud->points.size() ; i++)
    {

    }

}