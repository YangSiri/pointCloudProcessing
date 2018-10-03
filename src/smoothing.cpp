//
// Created by cyz on 18-9-23.
//

#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>
#include "../include/smoothing.h"

#include <boost/thread/thread.hpp>

#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>


bool preprocess::txt2pc(std::string txtpath, pcXYZI &origiCloud)
{
    std::ifstream txtfile(txtpath);
    pcl::PointXYZI pt;
    std::string line;
    int count=0;

    if(!txtfile)
    {
        std::cout<<"Wrong txt file! "<<endl;
    } else
    {
        while(getline(txtfile, line))
        {
            count++;
            std::stringstream ss(line);
            ss>>pt.x;
            ss>>pt.y;
            ss>>pt.z;
            ss>>pt.intensity;

            if(pt.z<4)
                continue;
//            cout<<"INTENSITY : "<<pt.intensity<<endl;
            cout<<count<<endl;
            origiCloud.push_back(pt);

//            if(count >= 10000000)
//                break;
        }

    }

    return true;
}



bool preprocess::statisticalOutlierRemoval(pcXYZIptr cloud,
                                           pcXYZIptr cloud_new)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statisOutRemov;
    statisOutRemov.setInputCloud(cloud);
    statisOutRemov.setMeanK(60);//neighbor points
    statisOutRemov.setStddevMulThresh(1.0);

    statisOutRemov.filter(*cloud_new);

}

bool preprocess::downSample(pcXYZIptr cloud, pcXYZIptr cloud_new)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxGrd;
    voxGrd.setInputCloud(cloud);
    voxGrd.setLeafSize(0.5, 0.5, 0.5);

    voxGrd.filter(*cloud_new);
}

bool preprocess::planeSeg( pcXYZIptr inputCloud,
                          std::vector<pcXYZI> &cloud_planes)
{
    pcXYZIptr tempCloud (new pcXYZI);
    pcl::copyPointCloud(*inputCloud, *tempCloud);

    pcl::ModelCoefficients::Ptr coeffi(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    pcl::SACSegmentation<pcl::PointXYZI> sacSeg;
    sacSeg.setOptimizeCoefficients(true);
    sacSeg.setModelType(pcl::SACMODEL_PLANE);
    sacSeg.setMethodType(pcl::SAC_RANSAC);
    sacSeg.setMaxIterations(800);
    sacSeg.setDistanceThreshold(0.3);///与模型的距离阈值，小于该值则为内点

    pcl::ExtractIndices<pcl::PointXYZI> extracIndice;

    do
    {
        pcXYZI planeCloud;
        sacSeg.setInputCloud(tempCloud);
        sacSeg.segment(*inliers, *coeffi);///输出内点以及平面的参数

        if(inliers->indices.size() == 0)
        {
            cout<<"There is no large plane for the given dataset. "<<endl;
            return false;
        }

        extracIndice.setInputCloud(tempCloud);
        extracIndice.setIndices(inliers);
        extracIndice.setNegative(false);
        extracIndice.filter(planeCloud);

        if(planeCloud.points.size() < 1000)
            break;
        cloud_planes.push_back(planeCloud);

        ///剔除平面点云
        extracIndice.setNegative(true);
        extracIndice.filter(*tempCloud);

    }while(tempCloud->points.size()>0.3 * inputCloud->points.size());

}

bool preprocess::normalestimate(pcXYZIptr cloud,
                                pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimater;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);

    normalEstimater.setInputCloud(cloud);
    kdtree->setInputCloud(cloud);
    normalEstimater.setSearchMethod(kdtree);
    normalEstimater.setKSearch(20);
    normalEstimater.compute(*normals);

}

bool preprocess::regionGrow_flat(pcXYZIptr cloud,
                                 pcl::PointCloud<pcl::Normal>::Ptr normals,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr &clustersCloud)
{
    std::vector<pcl::PointIndices> cluster;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZI>) ;
    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> regnGrowr;

    kdTree->setInputCloud(cloud);
    regnGrowr.setInputCloud(cloud);
    regnGrowr.setMinClusterSize(500);
    regnGrowr.setMaxClusterSize(10000);
    regnGrowr.setSearchMethod(kdTree);
    regnGrowr.setCurvatureThreshold(0.2);
    regnGrowr.setInputNormals(normals);
    regnGrowr.setNumberOfNeighbours(30) ;
    regnGrowr.setSmoothnessThreshold( 10.0 / 180.0 * M_PI );

    regnGrowr.extract(cluster);
    std::cout<<"total "<<cluster.size()<<" clusters. "<<endl;

    clustersCloud = regnGrowr.getColoredCloud();

    return true;
}