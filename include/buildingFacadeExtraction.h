//
// Created by cyz on 18-10-13.
//

#ifndef POINTCLOUDPROCESSING_BUILDINGFACADEEXTRACTION_H
#define POINTCLOUDPROCESSING_BUILDINGFACADEEXTRACTION_H

#include <math.h>
#include "smoothing.h"
#include <iostream>

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

class buildingFacadeExtractor
{
    
public:
    struct voxel
    {
        int id;
        int ptNums;
        std::vector<int> ptIndices;
        pcl::PointXYZI centroid;
    };
    
    bool groundFilter(pcXYZIptr origiCloud, pcXYZIptr nonGroundCloud);

    bool planeProjection(pcXYZIptr inputCloud,pcl::ModelCoefficients::Ptr paras,
                         pcXYZIptr outputCloud);

    bool meanShiftClustering(pcXYZIptr inputcloud, int iterTimes);

    bool constructVoxels(pcXYZIptr inputCloud, float gridResolution);

    bool makeSuperVoxels(voxel *voxelArrptr, int voxelNums, pcXYZIptr inputCloud,
                         std::vector<voxel> &superVoxels);
};

#endif //POINTCLOUDPROCESSING_BUILDINGFACADEEXTRACTION_H
