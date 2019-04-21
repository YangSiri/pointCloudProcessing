//
// Created by cyz on 18-9-23.
//

#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>
#include "../include/smoothing.h"

#include <boost/thread/thread.hpp>

#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


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
            try {
                ss>>pt.x;
                ss>>pt.y;
                ss>>pt.z;
                ss>>pt.intensity;
            }
            catch(...)
            {
                std::cout<<"invalid data!"<<endl;
                continue;
            }


//            if(pt.z<4)
//                continue;
//            cout<<"INTENSITY : "<<pt.intensity<<endl;
            cout<<"Point num : "<<count<<endl;
            origiCloud.push_back(pt);

//            if(count >= 10000000)
//                break;
        }

    }

    return true;
}

bool preprocess::xyz2pc(std::string txtpath, pcXYZI &origiCloud)
{
    std::ifstream xyzfile(txtpath);
    pcl::PointXYZI pt;
    char line[256];
    int count = 0;
    char intens;

    if(!xyzfile)
    {
        std::cout<<"Wrong *.xyz file! "<<endl;
    } else
    {
        while(!xyzfile.eof())
        {
            xyzfile.getline(line,256);

            try {
                std::sscanf(line, "%f %f %f %s", &pt.x, &pt.y, &pt.z, &intens);
                pt.intensity = intens;
                count++;
            }
            catch(...)
            {
                std::cout<<"invalid data!"<<endl;
                continue;
            }

//            if(pt.z<4)
//                continue;
//            cout<<"INTENSITY : "<<pt.intensity<<endl;
//            cout<<"Point num : "<<count<<endl;
//            cout<<"Point intensity : "<<pt.intensity<<endl;
            origiCloud.push_back(pt);

//            if(count >= 10000000)
//                break;
        }

    }

    return true;
}

bool preprocess::readpcfileFromFolder(std::string folder, std::vector<pcXYZI> &pcScans)
{
    std::vector<std::string> filenames;
    if(!boost::filesystem::exists(folder))
    {
        std::cout<<"wrong file folder!"<<endl;
        return false;
    }

    boost::filesystem::directory_iterator endIter;
    for(boost::filesystem::directory_iterator dirIter(folder); dirIter!=endIter; dirIter++)
    {
        if(dirIter->path().extension() != ".xyz")
        {
            std::cout<<"not a xyz file"<<endl;
            continue;
        }
        filenames.push_back(dirIter->path().string());
    }
    std::cout<<"total "<<filenames.size()<<" files."<<endl;

//    std::vector<pcXYZI> pcScans;
    pcScans.resize(filenames.size());
    for(int i=0 ; i<filenames.size() ; i++)
    {
//        std::cout<<"file : "<<filenames[i]<<endl;
        xyz2pc(filenames[i],pcScans[i]);
        std::cout<<"scan size : "<<pcScans[i].points.size()<<endl;
    }
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
    voxGrd.setLeafSize(0.2f, 0.2f, 0.2f);///采样密度

    voxGrd.filter(*cloud_new);
}

//incremental large plane segmentation
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
        std::cout<<"plane parameters : "<<
                 coeffi->values[0]<<" / "<<
                 coeffi->values[1]<<" / "<<
                 coeffi->values[2]<<" / "<<
                 coeffi->values[3]<<endl;

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
        {
            cout<<"Oops, not enough points on this plane"<<endl;
            break;
        }

        cloud_planes.push_back(planeCloud);

        ///剔除平面点云
        extracIndice.setNegative(true);
        extracIndice.filter(*tempCloud);

    }while(tempCloud->points.size() > 0.3 * inputCloud->points.size());//剩余点云数量30%

}

bool preprocess::normalestimate(pcXYZIptr cloud,
                                pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimater;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);

    normalEstimater.setInputCloud(cloud);
    kdtree->setInputCloud(cloud);
    normalEstimater.setSearchMethod(kdtree);
    normalEstimater.setKSearch(15);
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
    regnGrowr.setMinClusterSize(300);
    regnGrowr.setMaxClusterSize(8000);
    regnGrowr.setSearchMethod(kdTree);
    regnGrowr.setCurvatureThreshold(0.2);
    regnGrowr.setInputNormals(normals);
    regnGrowr.setNumberOfNeighbours(50) ;
    regnGrowr.setSmoothnessThreshold( 8.0 / 180.0 * M_PI );

    regnGrowr.extract(cluster);
    std::cout<<"total "<<cluster.size()<<" clusters. "<<endl;

    ///RANSAC求取平面参数
    Eigen::VectorXf plane_paras;
    for(size_t i=0 ; i<cluster.size() ; i++)
    {
        std::cout<<"----------------------------\n";
        pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr plane_modl
                (new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(cloud,cluster[i].indices) );
        std::cout<<"cluster size : "<<cluster[i].indices.size()<<"\n";

        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(plane_modl);
        ransac.setDistanceThreshold(0.3);
        ransac.computeModel();
        ransac.getInliers(cluster[i].indices);
        ransac.getModelCoefficients(plane_paras);
//        plane_modl->computeModelCoefficients(cluster[i].indices, plane_paras);
        if(fabs(plane_paras(0,0)) > 0.98 || fabs(plane_paras(1,0)) > 0.98)
        {
            std::cout<<"plane parameters : "<<plane_paras<<"\n";
            std::cout<<"inliers : "<<cluster[i].indices.size()<<endl;

            pcl::io::savePCDFile("../data/plane_"+std::to_string(i)+".pcd", *cloud, cluster[i].indices);
        }

    }

    clustersCloud = regnGrowr.getColoredCloud();

    return true;
}