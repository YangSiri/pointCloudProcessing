//
// Created by cyz on 18-10-27.
//

#include "lidarOdometry.h"
#include "smoothing.h"
#include "math.h"

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

using namespace lidarOdometry;

/**
 *
 * @param inputCloud
 * @param rangeImg
 * @param ptIdImg
 * @return
 */
bool lidarOdometryClass::getRangeAndptIdImage(pcXYZIptr inputCloud, Eigen::MatrixXf &rangeImg, Eigen::MatrixXf &ptIdImg)
{
    pcRGBptr colorCloud (new pcRGB());
    pcl::copyPointCloud(*inputCloud, *colorCloud);

//    Eigen::MatrixXf rangeImage and ptIdimage;
    rangeImg.resize(16,900);//VLP-16,horizontal angle resolution is 0.2 degree.900=180/0.2
    ptIdImg.resize(16,900);
    rangeImg = Eigen::MatrixXf::Zero(16,900);
    ptIdImg = Eigen::MatrixXf::Zero(16,900);

    double startVerticalangle = atan(inputCloud->points[0].z /
                                     sqrt(inputCloud->points[0].x*inputCloud->points[0].x
                                          + inputCloud->points[0].y*inputCloud->points[0].y)) * 180.0/M_PI;
    double startHorizontalangle = atan(inputCloud->points[0].y / inputCloud->points[0].x) * 180.0/M_PI;

    double vertiAngl, horiAngl;
    int row, col;
    double vertiAnglMax=0, horiAnglmax=0;
    double vertiAnglMin=1000, horiAnglmin=1000;

    ///求解垂直方向与水平方向角度范围
    for(int i=0 ; i<inputCloud->points.size() ; i++)
    {
        vertiAngl = atan(inputCloud->points[i].z / sqrt(inputCloud->points[i].x*inputCloud->points[i].x
                                                        + inputCloud->points[i].y*inputCloud->points[i].y)) * 180.0/M_PI ;

        horiAngl = atan(inputCloud->points[i].y / inputCloud->points[i].x) * 180.0/M_PI ;

        if(vertiAngl>vertiAnglMax)
            vertiAnglMax =vertiAngl;
        if(vertiAngl<vertiAnglMin)
            vertiAnglMin =vertiAngl;

        if(horiAngl>horiAnglmax)
            horiAnglmax =horiAngl;
        if(horiAngl<horiAnglmin)
            horiAnglmin =horiAngl;

    }
    std::cout<<vertiAnglMax<<"/// "<<vertiAnglMin<<endl;
    std::cout<<horiAnglmax<<"/// "<<horiAnglmin<<endl;

    pcl::search::KdTree<pcl::PointXYZI> kdTree;
    kdTree.setInputCloud(inputCloud);
    std::vector<int> neighborIndices;
    std::vector<float> neighborDist;
    int count = 0;
    for(int i=0 ; i<inputCloud->points.size() ; i++)
    {
        //领域点个数小于阈值
        std::vector<int>().swap(neighborIndices);
        std::vector<float>().swap(neighborDist);
        kdTree.radiusSearch(i, 3.0, neighborIndices, neighborDist );
        if(neighborIndices.size() < 6)
            continue;

        vertiAngl = atan(inputCloud->points[i].z / sqrt(inputCloud->points[i].x*inputCloud->points[i].x
                                                        + inputCloud->points[i].y*inputCloud->points[i].y)) * 180.0/M_PI - vertiAnglMin ;

        horiAngl = atan(inputCloud->points[i].y /inputCloud->points[i].x) * 180.0/M_PI - horiAnglmin;

        row = int(vertiAngl / 2.0 ) ; //vertical angle resolution =2.0 degree
        col = int(horiAngl / 0.2 ); //horizontal angle resolution =0.2 degree

//        if(row == 15)
//        {
//            colorCloud->points[i].r = rand()%255;
//            colorCloud->points[i].g = rand()%255;
//            colorCloud->points[i].b = rand()%255;
//        }

        if(rangeImg(row,col) != 0)
            count++;
        rangeImg(row,col) = sqrt(inputCloud->points[i].x*inputCloud->points[i].x
                                 + inputCloud->points[i].y*inputCloud->points[i].y
                                 + inputCloud->points[i].z*inputCloud->points[i].z);
        ptIdImg(row,col) = i;

    }
    std::cout<<"losing points num : "<<count<<endl;
//    pcl::io::savePCDFile("scan1.pcd",*colorCloud);

    count = 0;
    for(int i=0 ; i<16 ; i++)
        for(int j=0 ; j<900 ; j++)
            if(rangeImg(i,j) > 0)
                count++;
    std::cout<<"points num total :"<<inputCloud->points.size()<<endl;
    std::cout<<"points num in range image :"<<count<<endl;
    std::cout<<rangeImg<<count<<endl;
}

/**
 *
 * @param rangeImg
 * @param ptTypeImg
 * @return
 */
bool lidarOdometryClass::calculateSmoothness(Eigen::MatrixXf rangeImg, Eigen::MatrixXf &ptTypeImg)
{
    Eigen::MatrixXf smoothnessImg = Eigen::MatrixXf::Zero(16,900);
    ptTypeImg = Eigen::MatrixXf::Zero(16,900);
    int validrangeNums = 0;
    double sumRange = 0;
    double smoothness = 0;
    std::vector<double> smoothnessRow;

    for(int row=0 ; row<16 ; row++)
    {
        ///按列分5块,900/5=180
        for(int b=0 ; b<5 ; b++)
        {
            std::cout<<"\nrow/ "<<row<<"---block/ "<<(b+1)<<"---------------------"<<endl;
            for(int col=180*b+9 ; col<180*(b+1) ; col++)
            {
                //跳过近处点像素1m
                if(rangeImg(row, col) < 1)
                    continue;
                //该像素前后9邻域（同行）
                validrangeNums = 0;
                sumRange = 0;
                for(int i=col-9 ; i<col+9 ; i++)
                {
                    if(rangeImg(row,i) > 1)
                    {
                        validrangeNums++;
                        sumRange += rangeImg(row,i);
                    }
                }
                if(validrangeNums >= 9)
                {
                    smoothness = sumRange - rangeImg(row,col) * validrangeNums;
                    smoothness = fabs(smoothness / (validrangeNums*rangeImg(row,col)));
                    cout<<"smoothness of "<<row<<"//"<<col<<" : "<<smoothness<<endl;
                    smoothnessImg(row,col) = smoothness;
                    smoothnessRow.push_back(smoothness);
                }

            }//块内像素循环结束
            unsigned long smoothnessptSizeRow = smoothnessRow.size();
            std::sort(smoothnessRow.begin(),smoothnessRow.end());
            std::cout<<"valid point smoothness size : "<<smoothnessptSizeRow<<endl;
            for(int j=0 ; j<smoothnessptSizeRow ; j++)
                std::cout<<smoothnessRow[j]<<" /// ";

            if(smoothnessptSizeRow > 9)
            {

                for(int col=180*b+9 ; col<180*(b+1) ; col++)
                {
                    if(smoothnessImg(row,col) >= smoothnessRow[smoothnessptSizeRow - 3])
//                    if(smoothnessImg(row,col) >= smoothnessRow[smoothnessptSizeRow - int(smoothnessptSizeRow*0.2)])
                        ptTypeImg(row,col) = 1;
                    if(smoothnessImg(row,col) < smoothnessRow[3])
//                    if(smoothnessImg(row,col) < smoothnessRow[int(smoothnessptSizeRow*0.2)])
                        ptTypeImg(row,col) = 2;

                }
            }
            std::vector<double>().swap(smoothnessRow);
        }//块间循环




    }//行循环结束
//    std::cout<<smoothnessImg<<endl;
}

bool lidarOdometryClass::calculateCurvature(Eigen::MatrixXf ptIDimg)
{

    for(int row=0 ; row<16 ; row++)
    {
        
    }
}

/**
 *
 * @param posfile
 * @param postimeStamps
 * @param translations
 * @param rotations
 * @return
 */
bool lidarOdometryClass::readposefile(std::string posfile, std::vector<lidarOdometry::timeStamp> &postimeStamps,
                                      std::vector<Eigen::Vector3d> &translations,
                                      std::vector<Eigen::Quaterniond> &rotations) {
    std::ifstream ifs(posfile);
    char line[256];

    while(ifs.good())
    {
        ifs.getline(line,256);
        Eigen::Vector3d tmptrans;
        Eigen::Quaterniond tmprotat;

        long int sec,usec;
        sscanf(line,"%ld %ld %lf %lf %lf %lf %lf %lf %lf", &sec, &usec, &tmptrans[0], &tmptrans[1], &tmptrans[2],
        &tmprotat.coeffs()[0], &tmprotat.coeffs()[1], &tmprotat.coeffs()[2], &tmprotat.coeffs()[3]);

        lidarOdometry::timeStamp t(sec, usec);
        postimeStamps.push_back(t);
        translations.push_back(tmptrans);
        rotations.push_back(tmprotat);

    }

    return true;
}

/**
 *
 * @param vecPoints
 * @param pcPoints
 * @return
 */
bool lidarOdometryClass::vector2pointcloudXYZ(std::vector<Eigen::Vector3d> vecPoints, pcXYZptr pcPoints) {

    unsigned long ptSize = vecPoints.size();
    pcPoints->resize(ptSize);
//    pcl::PointXYZ tmpPt;

    for(int i=0 ; i<ptSize ; i++)
    {
//        tmpPt.x = vecPoints[i](0);
//        tmpPt.y = vecPoints[i](1);
//        tmpPt.z = vecPoints[i](2);
        pcPoints->points[i].x=vecPoints[i](0);
        pcPoints->points[i].y=vecPoints[i](1);
        pcPoints->points[i].z=vecPoints[i](2);
    }

    return true;
}

/**
 *
 * @param inputCloud
 * @param linesIndices
 * @return
 */
bool lidarOdometryClass::linefitting(pcXYZ inputCloud, std::vector<std::vector<int>> &linesIndices) {

    int cloudSize = inputCloud.points.size();
    pcXYZptr tempCloud (new pcXYZ);
    pcl::copyPointCloud(inputCloud, *tempCloud);

    pcl::ModelCoefficients::Ptr coeffi(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//
//    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr sacline;
//    sacline->setInputCloud();
//    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac();
    pcl::SACSegmentation<pcl::PointXYZ> sacSeg;
    sacSeg.setOptimizeCoefficients(true);
    sacSeg.setModelType(pcl::SACMODEL_LINE);
    sacSeg.setMethodType(pcl::SAC_RANSAC);
    sacSeg.setDistanceThreshold(0.1);///与模型的距离阈值，小于该值则为内点

    pcl::ExtractIndices<pcl::PointXYZ> extracIndice;
    std::vector<int> lineIndice;
    do
    {
        std::vector<int>().swap(lineIndice);
        sacSeg.setInputCloud(tempCloud);
        sacSeg.segment(*inliers, *coeffi);///输出内点以及直线参数
        std::cout<<"line parameters : "<<
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
        extracIndice.filter(lineIndice);

        if(lineIndice.size() < 5000)
        {
            std::cout<<"No enough points in line. "<<endl;
            break;
        }
        linesIndices.push_back(lineIndice);

        ///剔除平面点云
        extracIndice.setNegative(true);
        extracIndice.filter(*tempCloud);

    }while(tempCloud->points.size() > 0.3 * inputCloud.points.size());//剩余点云数量30%


    return true;
}