//
// Created by joe on 2019/8/29.
//

#ifndef POINTCLOUDPROCESSING_COMMONTOOLS_H
#define POINTCLOUDPROCESSING_COMMONTOOLS_H

#include <algorithm>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include "smoothing.h"

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x) (float, y, y)
                                          (float, z, z) (float, intensity, intensity)
                                          (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                          (double, time, time)
)


typedef PointXYZIRPYT  PointTypePose;
typedef pcl::PointCloud<PointTypePose>::Ptr  pcXYZIRPYTptr;
typedef pcl::PointCloud<PointTypePose>  pcXYZIRPYT;

using namespace std;

/**
    * 利用6D位姿将点云进行转换 from LeGO-LOAM （坐标轴不同于VLP16）
    */
pcXYZIptr transformPointCloud(pcXYZIptr cloudIn, PointTypePose* transformIn){

    pcXYZIptr cloudOut(new pcXYZI());

    pcl::PointXYZI *pointFrom;
    pcl::PointXYZI pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);


    for (int i = 0; i < cloudSize; ++i)
    {

        pointFrom = &cloudIn->points[i];
        float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
        float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
        float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

        pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
        pointTo.y = y2 + transformIn->y;
        pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
        pointTo.intensity = pointFrom->intensity;

        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}

int readRPYposefromfile(std::string file, pcXYZIRPYTptr pcRPYpose){

    PointTypePose ptRPY;
    char line[256] ;
    ifstream infile(file.c_str());
    if(infile.is_open())
    {
        while(!infile.eof())
        {
            infile.getline(line,256);
            sscanf(line, "%lf %f %f %f %f %f %f\n",&ptRPY.time, &ptRPY.x, &ptRPY.y, &ptRPY.z,
                   &ptRPY.roll, &ptRPY.pitch, &ptRPY.yaw);
            pcRPYpose->push_back(ptRPY);
        }
    }
    infile.close();
    int keyposeSize = pcRPYpose->points.size()-1;//最后一个位姿读了两遍?

    return keyposeSize;
}
int readQuanPosefromfile(const std::string& file, pcXYZIRPYTptr pcQuanpose){

    /**
     * row=q.x  pitch=q.y  yaw=q.z  intensity=q.z
     */
    PointTypePose ptRPY;
    char line[256] ;
    ifstream infile(file.c_str());
    if(infile.is_open())
    {
        while(!infile.eof())
        {
            infile.getline(line,256);
            sscanf(line, "%lf %f %f %f %f %f %f %f\n",&ptRPY.time, &ptRPY.x, &ptRPY.y, &ptRPY.z,
                   &ptRPY.roll, &ptRPY.pitch, &ptRPY.yaw, &ptRPY.intensity);
            pcQuanpose->push_back(ptRPY);
        }
    }
    infile.close();
    int keyposeSize = pcQuanpose->points.size()-1;//最后一个位姿读了两遍

    return keyposeSize;
}

bool getAndsaveglobalmap(string scanspath, pcl::PointCloud<PointTypePose>::Ptr pcRPYpose){

    pcXYZIptr globalmap(new pcXYZI());
    pcXYZIptr scan(new pcXYZI());
//    pcl::visualization::PCLVisualizer visualizer;
//    pcl::visualization::CloudViewer ccviewer("viewer");

    for (int k = 0; k < pcRPYpose->points.size(); ++k) {
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(scanspath+to_string(pcRPYpose->points[k].time)+".pcd", *scan) != -1){

            //legoLOAM中坐标系定义不同
            //转换到VLP坐标系下
            PointTypePose vlp_pose;
            vlp_pose = pcRPYpose->points[k];

            vlp_pose.x = pcRPYpose->points[k].z;
            vlp_pose.y = pcRPYpose->points[k].x;
            vlp_pose.z = pcRPYpose->points[k].y;
            vlp_pose.roll = pcRPYpose->points[k].yaw;
            vlp_pose.pitch = pcRPYpose->points[k].roll;
            vlp_pose.yaw = pcRPYpose->points[k].pitch;

            *globalmap += *transformPointCloud(scan, &vlp_pose);
//            ccviewer.showCloud(globalmap);
//            ccviewer.wasStwopped(100000);
            cout<<" —— scan "<<k<<" finished."<<endl;
        } else {
            cout<<"#no correspondent scan !"<<endl;
            continue;
        }
    }
    pcl::io::savePCDFile("/home/joe/workspace/testData/globalmapVLP.pcd",*globalmap);

    return true;
}

#endif //POINTCLOUDPROCESSING_COMMONTOOLS_H
