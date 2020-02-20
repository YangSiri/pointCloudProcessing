//
// Created by cyz on 18-10-13.
//

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include "smoothing.h"
#include "pcaAnalysis.h"
#include "buildingFacadeExtraction.h"

//progressive morphological ground filter
bool buildingFacadeExtractor::groundFilter(pcXYZIptr origiCloud, pcXYZIptr nonGroundCloud)
{
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> groundFilter;
    groundFilter.setInputCloud(origiCloud);
    groundFilter.setMaxWindowSize(20);
    groundFilter.setSlope(1.0f);
    groundFilter.setMaxDistance(6.0f);
    groundFilter.setInitialDistance(1.0f);//设置初始高度被认为是地面点
    groundFilter.extract(ground->indices);

    pcl::ExtractIndices<pcl::PointXYZI> extractor;
    extractor.setInputCloud(origiCloud);
    extractor.setIndices(ground);
    extractor.setNegative(true);
    extractor.filter(*nonGroundCloud);

    return true;
}

/**
 * project pointcloud to specific plane
 * @param inputCloud
 * @param paras
 * @param outputCloud
 * @return
 */
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

/**
 *
 * @param inputcloud
 * @param iterTimes
 * @return
 */
bool buildingFacadeExtractor::meanShiftClustering(pcXYZIptr inputcloud, int iterTimes){

    pcXYZI vecCloud;  // 储存每一点的mean shift vector
    pcl::PointXYZI tmp;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdTreeFla;
    kdTreeFla.setInputCloud(inputcloud);

    std::vector<int>ptIndces;
    std::vector<float >ptDist;

    Eigen::MatrixXf meanShiftVec_of_NeighborPt;

    int times = 0;
    do {
        ///for every point in the original cloud
        for (int i = 0; i < inputcloud->points.size(); i++)
        {
            std::vector<int>().swap(ptIndces);
            std::vector<float>().swap(ptDist);
            tmp.x = 0;
            tmp.y = 0;
            tmp.z = 0;

            kdTreeFla.radiusSearch(i, 2.0, ptIndces, ptDist);
            std::cout<<"/---neighbor points size : "<<ptIndces.size()<<endl;
            if (ptIndces.size() < 15)
            {
                vecCloud.push_back(tmp);
                continue;
            }
//            else if(ptIndces.size() > 1000)
//            {
//                std::cout<<"mean shift terminated. "<<endl;
//                break;
//            }

            meanShiftVec_of_NeighborPt.resize(3, ptIndces.size());

            ///for all neighbor pts
            for (int j = 0; j < ptIndces.size(); j++)
            {
                meanShiftVec_of_NeighborPt(0, j) = inputcloud->points[ptIndces[j]].x - inputcloud->points[i].x;
                meanShiftVec_of_NeighborPt(1, j) = inputcloud->points[ptIndces[j]].y - inputcloud->points[i].y;
                meanShiftVec_of_NeighborPt(2, j) = inputcloud->points[ptIndces[j]].z - inputcloud->points[i].z;

                tmp.x += meanShiftVec_of_NeighborPt(0, j) / ptIndces.size();
                tmp.y += meanShiftVec_of_NeighborPt(1, j) / ptIndces.size();
                tmp.z += meanShiftVec_of_NeighborPt(2, j) / ptIndces.size();

//                std::cout<<"/---dx = "<<tmp.x;
//                std::cout<<"/---dy = "<<tmp.y;
//                std::cout<<"/---dz = "<<tmp.z<<endl;

            }
            vecCloud.push_back(tmp);///save mean shift vector
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



/**
 *
 * @param inputCloud
 * @param gridResolution
 * @return
 */
bool buildingFacadeExtractor::constructVoxels(pcXYZIptr inputCloud, float gridResolution)
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);

    pcRGBptr voxelColorCloud (new pcRGB());
    pcl::copyPointCloud(*inputCloud, *voxelColorCloud);

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> pcOctree(gridResolution);
    pcOctree.setInputCloud(inputCloud);
    pcOctree.addPointsFromInputCloud();
//    pcOctree.defineBoundingBox(minPt.x, minPt.y, minPt.z, maxPt.x, maxPt.y, maxPt.z);
    std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> voxelCenterpts;
    int voxelSize = pcOctree.getOccupiedVoxelCenters(voxelCenterpts);
    int voxelNumsFilted = 0;


//    std::vector<voxel> voxels;
//    voxels.resize(voxelSize);
    voxel voxels[voxelSize] ;
//    voxel *voxelArrptr = new voxel[voxelSize];
    std::vector<int> inVoxelpt;

    for(int i=0 ; i<voxelSize ; i++){

        std::vector<int>().swap(inVoxelpt);
        pcOctree.voxelSearch(voxelCenterpts[i], inVoxelpt);

        std::cout<<"voxel points num: "<<inVoxelpt.size()<<std::endl;

        if(inVoxelpt.size() < 20)
            continue;

        voxels[i].id = i;
        voxels[i].ptNums = inVoxelpt.size();
        voxels[i].centroid = voxelCenterpts[i];
        voxels[i].ptIndices = inVoxelpt;
//        (*voxelArrptr).id = i;
//        (*voxelArrptr).ptNums = inVoxelpt.size();
//        (*voxelArrptr).centroid = voxelCenterpts[i];
//        (*voxelArrptr).ptIndices = inVoxelpt;
//        voxelArrptr++;

        voxelNumsFilted++;

        //不同体素随机赋色
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;

        for(int j=0; j < inVoxelpt.size() ; j++)
        {
            voxelColorCloud->points[inVoxelpt[j]].r = r;
            voxelColorCloud->points[inVoxelpt[j]].g = g;
            voxelColorCloud->points[inVoxelpt[j]].b = b;
        }

    }

//    voxels.resize(voxelNumsFilted);
    std::vector<voxel> superVox;
    makeSuperVoxels(voxels, voxelNumsFilted, inputCloud, superVox);

}

bool buildingFacadeExtractor::makeSuperVoxels(buildingFacadeExtractor::voxel *voxelArrptr,
                                              int voxelNums,
                                              pcXYZIptr inputCloud,
                                              std::vector<buildingFacadeExtractor::voxel> &superVoxels)
{
    pcaAnalysist pcaAnalysistor;
    for(int i=0; i<voxelNums ; i++)
    {
        pcaAnalysist::pcaFeature pcaFeatureOfVoxel;
        pcaAnalysistor.calculatePCAofPoint(inputCloud,
                                           voxelArrptr[i].centroid,
                                           voxelArrptr[i].ptIndices, pcaFeatureOfVoxel);
        if(pcaFeatureOfVoxel.planeStruc > 0.6)
            superVoxels.push_back(voxelArrptr[i]);

    }
}
