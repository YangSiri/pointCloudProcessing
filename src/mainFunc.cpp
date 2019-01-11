/*
 * executable->lidarOdometry
 */
// Created by cyz on 18-10-27.
//
#include "smoothing.h"
#include "lidarOdometry.h"
#include "pcapReader.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace lidarOdometry;
using namespace std;
using namespace pcl;

int main()
{
    char* pcapfilepath="../data/test.pcap";
    pcapReader pcapReaderClass;
    int flag = pcapReaderClass.readpcapfile(pcapfilepath);

    lidarOdometryClass lidarOdoClas;
    string posfile = "../data/pose.txt";
    vector<timeStamp> postimes;
    vector<Eigen::Vector3d> postrans;
    vector<Eigen::Quaterniond> posrotats;
    lidarOdoClas.readposefile(posfile,postimes, postrans, posrotats);

    pcXYZptr posCloud (new pcXYZ());
    lidarOdoClas.vector2pointcloudXYZ(postrans, posCloud);
    std::vector<std::vector<int>> linesIndices;
    lidarOdoClas.linefitting(*posCloud,linesIndices);
    std::cout<<"poseCloud size : "<<posCloud->points.size()<<endl;

    for(int i=0; i<linesIndices.size() ; i++)
    {
        pcl::io::savePCDFile("../data/lineCloud"+to_string(i)+".pcd",*posCloud,linesIndices[i]);
    }



    vector<pcXYZI> pcScans;
    std::vector<Eigen::MatrixXf> rangeImgs;
    std::vector<Eigen::MatrixXf> ptIDimgs;//point index of original point cloud
    std::vector<Eigen::MatrixXf> ptTypeImgs;//edge or planar points based on the smoothness

    std::string xyzfileFolder = "../data/scans";
    preprocess preprocessor;
    preprocessor.readpcfileFromFolder(xyzfileFolder,pcScans);
    cout<<"Read point cloud scan files successfully. "<<endl;

    rangeImgs.resize(pcScans.size());
    ptIDimgs.resize(pcScans.size());
    ptTypeImgs.resize(pcScans.size());

    for(int i=0 ; i<pcScans.size() ; i++)
    {
        pcXYZIptr pcScan (new pcXYZI(pcScans[i]));
        lidarOdoClas.getRangeAndptIdImage(pcScan,rangeImgs[i],ptIDimgs[i] );
        cout<<"/////////////////////////////////////////////////"<<endl;
        lidarOdoClas.calculateSmoothness(rangeImgs[i],ptTypeImgs[i]);
//        cout<<ptTypeImgs[i]<<endl;
        pcRGBptr pcColorScan(new pcRGB());
        copyPointCloud(*pcScan, *pcColorScan);
        int r = 255;
        int g = 255;
        int b = 255;
        int r1 = 0;
        int g1 = 255;
        int b1 = 0;
        for(int r=0 ; r<16 ; r++)
            for(int c=0 ; c<900 ; c++)
            {
                if(ptTypeImgs[i](r,c) == 1)
                {
                    pcColorScan->points[ptIDimgs[i](r,c)].r = r;
                    pcColorScan->points[ptIDimgs[i](r,c)].g = g;
                    pcColorScan->points[ptIDimgs[i](r,c)].b = b;
                }
                if(ptTypeImgs[i](r,c) == 2)
                {
                    pcColorScan->points[ptIDimgs[i](r,c)].r = r1;
                    pcColorScan->points[ptIDimgs[i](r,c)].g = g1;
                    pcColorScan->points[ptIDimgs[i](r,c)].b = b1;
                }

            }

        io::savePCDFile("../data/keypointScans/colorScan"+to_string(i)+".pcd",*pcColorScan);
    }

}