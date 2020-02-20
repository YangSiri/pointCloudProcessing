/////////////////////////////////////////////////////////
// Created by joe on 2019/9/26.
//
// Modified from LeGO-LOAM imageProjection.
// mainly aim for extracting cloudaboveground
//
//////////////////////////////////////////////////////////////

#ifndef POINTCLOUDPROCESSING_RINGPROJECTION_H
#define POINTCLOUDPROCESSING_RINGPROJECTION_H

#include "commontools.h"
#include <octomap/octomap.h>
#include <opencv2/opencv.hpp>

extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;//360\0.2
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;

extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 1.0472;
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

class RingProjector{

private:
    struct ringGrid{
        int row, col;
        std::vector<int> ptIndices;
        double height;

        ringGrid(){
            row = -1;
            col = -1;
            height = -1;
            std::vector<int>().swap(ptIndices);
        }
    };

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;  //单帧点云

    pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud;   // 距离图像对应的点云,intensity=row + (col/10000)
    pcl::PointCloud<pcl::PointXYZI>::Ptr fullInfoCloud;  //投影得到的距离图像 16*1800
    // （index为行列号，无坐标值，x为原始点云indice, intensity为range）

    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud; //地面点
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloud;  //代表较大物体的点，其中地面点抽稀过,与fullCloud点类型相同
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloudPure;  //所有代表较大物体的点（不含地面点）,intensity为标签
    pcl::PointCloud<pcl::PointXYZI>::Ptr outlierCloud;  //地面点中因抽稀而滤掉的点以及散乱点
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudaboveGround;  //位于地面上的点云，与非地面点不同

    pcl::PointXYZI nanPoint;

    cv::Mat rangeMat; //距离图像对应的距离矩阵
    cv::Mat labelMat;  //距离图像对应的点的label, -1为地面点
    cv::Mat groundMat; //距离图像对应的点是否为地面点 1则为地面点
    int labelCount;

    float startOrientation;
    float endOrientation;
    float orientationDiff;

    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;
    uint16_t *queueIndY;

    bool isOutdoor;

    double angleResolu_, distResolu_;
    int rowNum_, colNum_;
    double *maxGroundrangeOfcol;

    ringGrid *ringGrids_;

public:
    RingProjector(double angle, double dist){

        angleResolu_ = angle;
        distResolu_ = dist;
        rowNum_ = 100.0 / dist;
        colNum_ = 360.0 / angle;

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());

        fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        fullInfoCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        groundCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        segmentedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        segmentedCloudPure.reset(new pcl::PointCloud<pcl::PointXYZI>());
        outlierCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudaboveGround.reset(new pcl::PointCloud<pcl::PointXYZI>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        maxGroundrangeOfcol = new double[Horizon_SCAN];//初始化为0？
        ringGrids_ = new ringGrid[rowNum_*colNum_];
    }
    void resetParameters(){

        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();
        cloudaboveGround->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~RingProjector(){}


    //1
    void setInputCloud(const pcXYZIptr inCloud);

    //2
    void findStartEndAngle();

    //3
    void projectPointCloud();

    //4
    void groundRemoval();

    //5
    void cloudSegmentation();
    void labelComponents(int row, int col);

    //6
    void publishCloud(pcXYZIptr outCloud);

    void fullSegmentation(){

        projectPointCloud();
        groundRemoval();
        cloudSegmentation();
    }

};
#endif //POINTCLOUDPROCESSING_RINGPROJECTION_H
