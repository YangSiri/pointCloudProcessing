//
// Created by joe on 2019/8/24.
//

//#include <gtsam/geometry/Rot3.h>
//#include <gtsam/geometry/Pose3.h>
//#include <gtsam/slam/PriorFactor.h>
//#include <gtsam/slam/BetweenFactor.h>
//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <gtsam/nonlinear/Values.h>
//#include <gtsam/nonlinear/ISAM2.h>

#include "commontools.h"

int main(int argc, char** argv){

    pcl::PointCloud<PointTypePose>::Ptr scanPoses(new pcl::PointCloud<PointTypePose>);
    std::string scanposefile = "/home/joe/workspace/testData/keyposesScanlinear.txt";
    int poseSize = readRPYposefromfile(scanposefile, scanPoses);
    std::cout<<"READ "<<poseSize<<" poses!"<<std::endl;

    std::string scanFolder ="/home/joe/workspace/testData/veloScans/";
    getAndsaveglobalmap(scanFolder, scanPoses);
    pcXYZIptr globalmap(new pcXYZI);
    pcXYZIptr localmap(new pcXYZI);


    for (int i = 0; i < poseSize; ++i) {
        pcXYZIptr scan(new pcXYZI);

        if (pcl::io::loadPCDFile(scanFolder+to_string(scanPoses->points[i].time)+".pcd", *scan) != -1){
            *globalmap += *transformPointCloud(scan, &scanPoses->points[i]);

        }
    }

//    gtsam::NonlinearFactorGraph gtGraph;
//    gtsam::Values initialvalues;
//    gtsam::Values optimizedvalues;
//    gtsam::ISAM2* isam;
//    gtsam::Values isamCurrentEstimate;

    return 0;
}
