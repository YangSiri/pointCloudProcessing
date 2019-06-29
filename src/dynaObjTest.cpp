//
// Created by cyz on 19-4-2.
//
#include <boost/filesystem.hpp>

#include "dbscanCLuster.h"
#include "smoothing.h"

#include <time.h>
#include <fstream>
#include <stdio.h>
#include <Eigen/Core>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "cubicfitting.h"

using namespace std;
using namespace curves;

int main(int argc, char** argv)
{
    clock_t st,et;
    double ut;
    int datanum = 0;

    /**
     * 位姿文件读取，拟合部分
     */
    pcl::PointCloud<PointTypePose>::Ptr pcRPYpose(new pcl::PointCloud<PointTypePose>());
    PointTypePose ptRPY;
    char line[256] ;
    ifstream infile("/home/cyz/Data/legoloam/poses/keyposesFinal.txt");
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
    int keyposeSize = pcRPYpose->points.size()-1;
    cout<<"--->read total poses : "<<keyposeSize<<endl;

    ///位姿样条线拟合
    std::vector<PointTypePose> posesTocurveFit;
    for(int i=0; i<keyposeSize-3 ; i++) {

        std::vector<PointTypePose>().swap(posesTocurveFit) ;
        posesTocurveFit.push_back(pcRPYpose->points[i]);
        posesTocurveFit.push_back(pcRPYpose->points[i+1]);
        posesTocurveFit.push_back(pcRPYpose->points[i+2]);
        posesTocurveFit.push_back(pcRPYpose->points[i+3]);

//        cubicHermitSE3fitting(posesTocurveFit);
        cumulativeSplineSE3fitting(posesTocurveFit);

        //利用曲线拟合修改原始位姿
        pcRPYpose->points[i+1] = posesTocurveFit[1];
        pcRPYpose->points[i+2] = posesTocurveFit[2];
//        pcRPYpose->points[i+1].x = posesTocurveFit[1].x;
    }


    ///储存拟合后的轨迹位姿信息
     FILE *fp2;
    fp2 = fopen("/home/cyz/Data/legoloam/poses/keyposesFinalcubic.txt","w");
    for (int j = 0; j < keyposeSize; ++j) {

        fprintf(fp2, "%lf %f %f %f %lf %lf %lf\n", pcRPYpose->points[j].time,
                pcRPYpose->points[j].x,
                pcRPYpose->points[j].y,
                pcRPYpose->points[j].z,
                pcRPYpose->points[j].roll,
                pcRPYpose->points[j].pitch,
                pcRPYpose->points[j].yaw);
    }
    fclose(fp2);
/**
 * 位姿部分结束
 */

//    string dataFolder = "/home/cyz/Data/legoloam/poses/keyframes/";
    string dataFolder = "/home/cyz/Data/legoloam/poses/velodyneScans/";
    pcXYZIptr globalmap(new pcXYZI());
    pcXYZIptr scan(new pcXYZI());
    pcl::visualization::PCLVisualizer visualizer;
    pcl::visualization::CloudViewer ccviewer("viewer");

    for (int k = 0; k < keyposeSize; ++k) {

        if(pcl::io::loadPCDFile<pcl::PointXYZI>(dataFolder+to_string(pcRPYpose->points[k].time)+".pcd", *scan) != -1){

//            float tmp=0;
//            for (int i = 0; i < scan->points.size(); ++i) {
//                tmp = scan->points[i].x;
//                scan->points[i].x = scan->points[i].y;
//                scan->points[i].y = scan->points[i].z;
//                scan->points[i].z = tmp;
//            }
            PointTypePose vlp_pose;
            vlp_pose = pcRPYpose->points[k];
            vlp_pose.x = pcRPYpose->points[k].z;
            vlp_pose.y = pcRPYpose->points[k].x;
            vlp_pose.z = pcRPYpose->points[k].y;
            vlp_pose.roll = pcRPYpose->points[k].yaw;
            vlp_pose.pitch = pcRPYpose->points[k].roll;
            vlp_pose.yaw = pcRPYpose->points[k].pitch;
            *globalmap += *tools::transformPointCloud(scan, &vlp_pose);

//            *globalmap += *tools::transformPointCloud(scan, &pcRPYpose->points[k]);

//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tc_handler(globalmap, 0, 255, 0);
//            visualizer.addPointCloud(globalmap, tc_handler, "scan"+to_string(k));
            ccviewer.showCloud(globalmap);
//            ccviewer.wasStwopped(100000);
        cout<<"scan "<<k<<" finished."<<endl;
        } else {
            cout<<"#no correspondent scan !"<<endl;
            continue;
        }

    }
    pcl::io::savePCDFile("/home/cyz/Data/legoloam/poses/globalmap.pcd",*globalmap);
    return 0;

/**
 * track部分
 */
//    pcXYZIptr globalmap(new pcXYZI());
//    pcXYZIptr transformedScan(new pcXYZI());
//    pcXYZIptr scan(new pcXYZI());
//    std::vector<tools::clusterTracker> trackersList;
//    string dataFolder = "/home/cyz/Data/legoloam/poses/velodyneScans/";
//    pcl::visualization::PCLVisualizer visualizer;
//    pcl::visualization::CloudViewer ccviewer("viewer");
////    const Eigen::Quaternionf noRotation(Eigen::Quaternionf::Identity());
//    for(int i=0; i<1000; i++)
//    {
//        cout<<"\n  --->Scan "<<i<<endl;
//        pcXYZIptr pcClusterCenters(new pcXYZI());
//        pcXYZIptr pcClusterMins(new pcXYZI());
//        pcXYZIptr pcClusterMaxs(new pcXYZI());
//        //当前帧被追踪到的
//        pcXYZIptr pcClusterCentersTracked(new pcXYZI());
//        pcXYZIptr pcClusterMinsTracked(new pcXYZI());
//        pcXYZIptr pcClusterMaxsTracked(new pcXYZI());
//        pcXYZIptr pcAllmeasurements(new pcXYZI());
//        vector<pcXYZI> vecTrackedallMeasurements;
//
//        if(pcl::io::loadPCDFile<pcl::PointXYZI>(dataFolder+to_string(i)+".pcd", *scan) != -1) {
////            *globalmap += *tools::transformPointCloud(scan, &pcRPYpose->points[i]);
////            *transformedScan = *tools::transformPointCloud(scan, &pcRPYpose->points[i]);
//
////            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tc_handler(scan, 0, 255, 0);
////            visualizer.addPointCloud(scan, tc_handler, "scan"+to_string(i));
//            ccviewer.showCloud(scan);
//
//            st = clock();
//
//            int clusterSize = tools::extractClusterEuclidean(scan, pcClusterCenters, pcClusterMins, pcClusterMaxs);
//            cout<<"Filtered cluster size : "<<clusterSize<<endl;
//
//            vecTrackedallMeasurements.clear();
//
//            if(i==0)//第一帧初始化
//            {
//                for(int j=0; j<clusterSize; j++)
//                {
//                    tools::clusterTracker trackerTmp(j, pcClusterCenters->points[j]);
//                    trackersList.push_back(trackerTmp);
//                }
//
//            } else{
//                for(std::vector<tools::clusterTracker>::iterator it=trackersList.begin();
//                        it!=trackersList.end(); )
//                {
//                    bool tracked=false;
//                    pcl::PointXYZI predicpt = it->predicState();//z=Vx, intensity=Vy
//                    pcXYZI lastmeasurept = it->getpreMeasurements();
//
//                    //根据预测值寻找当前帧对应的measurement--最近点
//                    //如果ID=0,证明当前帧无对应点
//                    int nearestID = tools::findNearestNeighborIndice(predicpt, pcClusterCenters);
//                    cout<<"NNpt ID "<<nearestID;
//                    pcl::PointXYZI nearestPtcentro = pcClusterCenters->points[nearestID];
//                    double nearestDis = dbscan2d::distance(predicpt.x, predicpt.y, nearestPtcentro.x, nearestPtcentro.y);
//                    cout<<" >> nearest distance in 2D: "<<nearestDis<<endl;
//
//                    if(nearestDis < 0.3)
//                    {
//                        it->addmeasurements(nearestPtcentro);
//                        tracked = true;
//                        //Vx > ? \\  || predicpt.z > 0
//                        if(abs(predicpt.z) - 2 < 0.05 )
//                        {
//                            pcClusterCentersTracked->push_back(pcClusterCenters->points[nearestID]);
//                            pcClusterMinsTracked->push_back(pcClusterMins->points[nearestID]);
//                            pcClusterMaxsTracked->push_back(pcClusterMaxs->points[nearestID]);
//                            vecTrackedallMeasurements.push_back(lastmeasurept);
//                        }
//
//                        pcClusterCenters->points[nearestID].intensity = 9999;
//                    }
//
//                    //lost in current scan
//                    if(!tracked)
//                    {
//                        //tracked times < ?
//                        if(it->getTrackedtimes() == -1) {
//
//
//                            it = trackersList.erase(it);
//                        }
//
//                        else
//                            it++;
//                    }
//                    else
//                        it++;
//
//                }
//                //old tracklist size
//                int previousTrackerSize = trackersList.size();
//                cout<<"\n New trackers >> >> >>  pevious Trackerlist : "<<previousTrackerSize<<endl;
//                for(int j=0; j<clusterSize; j++) {
//                    //new cluster new tracks
//                    if(pcClusterCenters->points[j].intensity < 9000)
//                    {
//                        tools::clusterTracker trackerTmp(previousTrackerSize, pcClusterCenters->points[j]);
//                        trackersList.push_back(trackerTmp);
//                        previousTrackerSize++;
//                    }
//                }
//
//            }
//            et = clock();
//            ut = double(et - st)/CLOCKS_PER_SEC;
//            printf("Time used is :%f s for tracking. \n===============================================",ut);
//            if(pcClusterCenters->points.size() > 3 )
//            {
////                pcl::io::savePCDFile("/home/cyz/Data/legoloam/poses/scanClusterCentro/scan"+to_string(i)+".pcd"
////                        ,*pcClusterCenters);
//
//                pcl::copyPointCloud(*pcClusterMins, *tools::pcCluster_Mins);
//                pcl::copyPointCloud(*pcClusterMaxs, *tools::pcCluster_Maxs);
//                pcl::copyPointCloud(*pcClusterCentersTracked, *tools::pcCluster_Centros);
//                tools::vecTrackedallmeasurements_.swap(vecTrackedallMeasurements);
////                ccviewer.runOnVisualizationThread(tools::viewClusterbox);
//                ccviewer.runOnVisualizationThreadOnce(tools::viewClusterbox);
//                boost::this_thread::sleep (boost::posix_time::microseconds (500000));
//                ccviewer.removeVisualizationCallable();
//                continue;
//            }
//
//        }
//
//
//    }
//
//
//    return 0;
    /**
     * tracker部分结束
     */

    /**
     * 测试部分
     */
    pcXYZIptr cloud(new pcXYZI());
    vector<pcXYZI> clouds;
    vector<int> labels;
    int clusterSize = 0;

    if(pcl::io::loadPCDFile<pcl::PointXYZI>(dataFolder+"/integrated.pcd", *cloud) == -1 )
    { cout<<"wrong while reading file"<<endl;
        return 0; }
    clusterSize =dbscan2d::dbscan( cloud, 0.07, 16);
    cout<<"DNSCAN size :"<<clusterSize<<endl;
    pcl::io::savePCDFile(dataFolder+"/clustered.pcd",*cloud);


    if(pcl::io::loadPCDFile<pcl::PointXYZI>(dataFolder+"/1552286992.732664000.pcd", *cloud) == -1 )
    { cout<<"wrong while reading file"<<endl;
        return 0; }
    st = clock();
    clusterSize = dbscan2d::dbscan( cloud, 0.1, 16);
    et = clock();
    cout<<"DNSCAN size :"<<clusterSize<<endl;
    clouds.push_back(*cloud);
    pcl::io::savePCDFile(dataFolder+"/clustered.pcd",*cloud);
    ut = double(et - st)/CLOCKS_PER_SEC;
    printf("time used is :%f s\n",ut);



    ///2DdbscanClusterTest:
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(dataFolder+"/1552286992.833538000.pcd", *cloud) == -1 )
    { cout<<"wrong while reading file"<<endl;
        return 0; }
    double dist=0;    st = clock();
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr condi(new pcl::ConditionAnd<pcl::PointXYZI>());
    condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                 (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT, -10.0)));
    condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                 (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT, 10.0)));
    condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                 (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT, -10.0)));
    condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                 (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT, 10.0)));
    pcl::ConditionalRemoval<pcl::PointXYZI> condRem;
    condRem.setCondition(condi);
    condRem.setInputCloud(cloud);
    condRem.setKeepOrganized(true);
    condRem.filter(*cloud);

//    for(int i=0; i<cloud->points.size(); i++)
//    {
////        if(cloud->points[i].x>10 || cloud->points[i].y>10)
////            continue;
//        for(int j=i+1; j<cloud->points.size(); j++){
//            if(cloud->points[j].intensity == cloud->points[i].intensity)
//                continue;
//            dist = dbscan2d::distance(cloud->points[j].x, cloud->points[j].y, cloud->points[i].x, cloud->points[i].y);
//            if(dist < 0.3)
//                cloud->points[j].intensity = cloud->points[i].intensity;
//        }
//
//    }
    clusterSize = dbscan2d::dbscan(cloud, 0.3, 50);
    cout<<"DNSCAN size :"<<clusterSize<<endl;
    et = clock();
    ut = double(et - st)/CLOCKS_PER_SEC;
    printf("time used is :%f s for per dbscan\n",ut);
    pcl::io::savePCDFile(dataFolder+"/clustered.pcd",*cloud);
    clouds.push_back(*cloud);


    if(pcl::io::loadPCDFile<pcl::PointXYZI>(dataFolder+"/1552286992.934405000.pcd", *cloud) == -1 )
    { cout<<"wrong while reading file"<<endl;
        return 0; }
//    for(int i=0; i<cloud->points.size(); i++)
//        cloud->points[i].z = 0;
    condRem.setInputCloud(cloud);
    condRem.setKeepOrganized(true);
    condRem.filter(*cloud);
    clouds.push_back(*cloud);

    cloud->clear();

    ///octreeChangeDetectorTest:
    st = clock();
    float resol = 0.1f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octreChangeDect(resol);
    pcXYZIptr scan1 = clouds[1].makeShared();
    pcXYZIptr scan2 = clouds[2].makeShared();

    octreChangeDect.setInputCloud(scan1);
    octreChangeDect.addPointsFromInputCloud();

    octreChangeDect.switchBuffers();

    octreChangeDect.setInputCloud(scan2);
    octreChangeDect.addPointsFromInputCloud();

    vector<int> newPtIndic;
    octreChangeDect.getPointIndicesFromNewVoxels(newPtIndic);

    cout<<"new points :"<<newPtIndic.size()<<endl;
    pcXYZIptr newptsCloud(new pcXYZI());
    for(int i=0; i<newPtIndic.size(); i++)
        newptsCloud->push_back(scan2->points[newPtIndic[i]]);
    clusterSize = dbscan2d::dbscan(newptsCloud, 0.17, 30);
    pcl::io::savePCDFile(dataFolder+"/changedptsCloud.pcd",*newptsCloud);

    et = clock();
    ut = double(et - st)/CLOCKS_PER_SEC;
    printf("time used is :%f s for changeDetection\n",ut);

    pcl::PointCloud<pcl::PointXYZL>::Ptr pcWith_labl(new pcl::PointCloud<pcl::PointXYZL>());
    pcl::PointXYZL ptlabl;
    for(int i=0; i<newptsCloud->points.size(); i++)
        if(newptsCloud->points[i].intensity != 0)
        {
            ptlabl.x = newptsCloud->points[i].x;
            ptlabl.y = newptsCloud->points[i].y;
            ptlabl.z = newptsCloud->points[i].z;
            ptlabl.label =(int)newptsCloud->points[i].intensity;
            pcWith_labl->push_back(ptlabl);
        }

    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL> pchandlerLabl;
    pchandlerLabl.setInputCloud(pcWith_labl);
    viewer.addPointCloud(pcWith_labl, pchandlerLabl, "newptsCLoud");
    while (!viewer.wasStopped())
    {
        viewer.spin();
    }


    ///octomapTest:
    octomap::ColorOcTree colorTree(0.1);
    octomap::ColorOcTreeNode* node = nullptr;
    pcXYZIptr changedcloud(new pcXYZI());
    st = clock();
    for(int i=0 ; i<3 ; i++)
    {
        pcl::removeNaNFromPointCloud(clouds[i], clouds[i], labels);

        labels.clear();
//        int clusterSize = dbscan( clouds[i], labels, 0.3, 16);
//        cout<<"DNSCAN size :"<<clusterSize<<endl;

        for(auto p:clouds[i].points)
        {
            if(abs(p.x) > 10||abs(p.y)> 10)
                continue;

            if(i == 0)
                colorTree.updateNode(octomap::point3d(p.x, p.y, 0), true);
            else
            {
                node = colorTree.search(octomap::point3d(p.x, p.y, 0));
                if(node == nullptr)
                {
                    changedcloud->push_back(p);
                    colorTree.updateNode(octomap::point3d(p.x, p.y, 0), true);
                    node = colorTree.search(octomap::point3d(p.x, p.y, 0));
                    node->setColor(255,0,0);
//                    octomap::OcTreeKey key = colorTree.coordToKey(octomap::point3d(p.x, p.y, 0), 1);
//                    cout <<"key index: "<< key.k[0] <<", "<< key.k[1] <<", "<< key.k[2] <<endl;

                }

            }

//            vector<octomap::point3d> rayintersec_points;
//            if(colorTree.computeRay(octomap::point3d(0,0,0), octomap::point3d(p.x, p.y, 0), rayintersec_points)) {
//                for(int i=0; i<rayintersec_points.size(); i++) {
//                    node = colorTree.search(rayintersec_points[i]);
//                    if (node == nullptr)
//                        continue;
//                    if (colorTree.isNodeOccupied(node))
//                    {
//                        //      colorTree.deleteNode(rayintersec_points[i],0);
//                        cout << "coming trough..." << rayintersec_points[i] << endl;
//                        node->setColor(255, 0, 0);
////                        node->setValue(0);
//                    }
//                }
//            }
//            rayintersec_points.clear();

//            colorTree.insertRay(octomap::point3d(0,0,0), octomap::point3d(p.x, p.y, 0));
//            node = colorTree.search(octomap::point3d(p.x, p.y, 0));

//            cout<<"occupancy probability： "<<node->getOccupancy()<<endl;
//             cout<<"node value： "<<node->getValue()<<endl;
//            double grey = 255.0 * node->getOccupancy() ;
//            colorTree.integrateNodeColor(p.x, p.y, 0, grey, grey, grey);
//            if(node->getOccupancy() > 0.9)
//                colorTree.integrateNodeColor(p.x, p.y, 0, 255, 0, 0);
//            else
//                colorTree.integrateNodeColor(p.x, p.y, 0, 0, 255, 0);

        }

        colorTree.updateInnerOccupancy();

        datanum++;
        if(datanum > 2)
            break;

    }
    pcl::io::savePCDFile(dataFolder+"/changedCloudoctomap.pcd",*changedcloud);
    et = clock();
    ut = double(et - st)/CLOCKS_PER_SEC;
    printf("time used is :%f s for octomap\n",ut);
    colorTree.write("/home/cyz/Data/legoloam/seg_pure/octo.ot");


    return 0;

}