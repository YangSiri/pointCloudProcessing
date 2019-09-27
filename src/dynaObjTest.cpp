//
// Created by cyz on 19-4-2.
//
#include <boost/filesystem.hpp>

#include <time.h>
#include <fstream>
#include <string.h>
#include <strings.h>
#include <cstring>

#include <Eigen/Core>

#include <octomap/ColorOcTree.h>

#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "cubicfitting.h"
#include "dbscanCLuster.h"
#include "smoothing.h"
#include "commontools.h"
#include "ringProjection.h"

using namespace std;
//using namespace curves;

int main(int argc, char** argv) {

/**
 * track部分
 */
    {
        clock_t st, et;
        double ut;

        pcl::PointCloud<PointTypePose>::Ptr pcRPYpose(new pcl::PointCloud<PointTypePose>());
        std::string posefile = "/home/joe/workspace/testData/integratedposesQuan.txt";

        int posesSize = readQuanPosefromfile(posefile, pcRPYpose);
        cout << "--->read total poses : " << posesSize << endl;

        pcXYZIptr globalmap(new pcXYZI());
        pcXYZIptr transformedScan(new pcXYZI());
        pcXYZIptr transformedSeg(new pcXYZI());
        pcXYZIptr scan(new pcXYZI());
        pcXYZIptr segedCloud(new pcXYZI());

        std::vector <DynaTools::clusterTracker> trackersList;
        string scanFolder = "/home/joe/workspace/testData/veloScans/";
        string segFolder = "/home/joe/workspace/testData/segpure/";

        std::vector<int> dynaPtsIndices;
        pcXYZIptr pcClusterCenters(new pcXYZI());
        pcXYZIptr pcClusterMins(new pcXYZI());
        pcXYZIptr pcClusterMaxs(new pcXYZI());
        //当前帧被追踪到的
        pcXYZIptr pcClusterCentersTracked(new pcXYZI());
        pcXYZIptr pcClusterMinsTracked(new pcXYZI());
        pcXYZIptr pcClusterMaxsTracked(new pcXYZI());
        pcXYZIptr pcAllmeasurements(new pcXYZI());
        vector <pcXYZI> vecTrackedallMeasurements;

        int nearestID =-1;
        double nearestDis = -1;
        pcl::PointXYZI predicpt;
        pcl::visualization::CloudViewer ccviewer("Viewer");
        bool init = false;
        RingProjector ringProjector(2, 2);

        for (int i = 0; i < posesSize; i++) {

            cout << "\n  --->Scan " << i << endl;

            pcClusterCenters.reset(new pcXYZI());
            pcClusterMins.reset(new pcXYZI());
            pcClusterMaxs.reset(new pcXYZI());
            //当前帧被追踪到的
            pcClusterCentersTracked.reset(new pcXYZI());
            pcClusterMinsTracked.reset(new pcXYZI());
            pcClusterMaxsTracked.reset(new pcXYZI());
            pcAllmeasurements.reset(new pcXYZI());
            vector <pcXYZI>().swap(vecTrackedallMeasurements) ;

            //read segmented cloud first
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(scanFolder + to_string(pcRPYpose->points[i].time) + ".pcd",
                                                     *scan) != -1) {
                cout << "  --->Scan " << to_string(pcRPYpose->points[i].time) << endl;

                st = clock();
                segedCloud->clear();
                transformedScan->clear();
                transformedSeg->clear();

                RingProjector ringProjector(2, 2);
                ringProjector.setInputCloud(scan);
                ringProjector.fullSegmentation();
                ringProjector.publishCloud(segedCloud);

                PointTypePose vlp_pose;
                vlp_pose = pcRPYpose->points[i];

//                //转换到VLP坐标系下
//                vlp_pose.y = pcRPYpose->points[k].x;
//                vlp_pose.z = pcRPYpose->points[k].y;
//                vlp_pose.roll = pcRPYpose->points[k].yaw;
//                vlp_pose.pitch = pcRPYpose->points[k].roll;
//                vlp_pose.yaw = pcRPYpose->points[k].pitch;
//                *transformedScan = *tools::transformPointCloud(scan, &vlp_pose);

                Eigen::Vector3f t(vlp_pose.z, vlp_pose.x, vlp_pose.y);
                Eigen::Quaternionf quan(vlp_pose.intensity, vlp_pose.yaw, vlp_pose.roll, vlp_pose.pitch);
                pcl::transformPointCloud(*segedCloud, *transformedSeg, t, quan);

                //打开相对应的vlpscan
//                if (pcl::io::loadPCDFile<pcl::PointXYZI>(scanFolder + to_string(pcRPYpose->points[i].time) + ".pcd",
//                                                         *scan) == -1)
//                    continue;
                pcl::transformPointCloud(*scan, *transformedScan, t, quan);

//            ccviewer.showCloud(transformedScan);


//                int clusterSize = dbscan2d::dbscan(segedCloud, 0.15, 20);// slow:0.2s / 1000pts
                int clusterSize = DynaTools::clusterDBSCAN(transformedSeg, 0.2, 20,
                                                           pcClusterCenters,
                                                           pcClusterMins,
                                                           pcClusterMaxs);// 0.03s
//                int clusterSize = DynaTools::extractClusterEuclidean(segedCloud, 0.2
//                                                                 pcClusterCenters,
//                                                                 pcClusterMins,
//                                                                 pcClusterMaxs);
                cout << "Filtered cluster size : " << clusterSize << endl;
                ccviewer.showCloud(transformedSeg);
//                transformedSeg->width = 1;
//                transformedSeg->height = transformedSeg->points.size();
//                pcl::io::savePCDFile("/home/joe/workspace/testData/cloudaboveground/clusterCentro/"+
//                                     to_string(pcRPYpose->points[i].time)+".pcd", *transformedSeg);

                vecTrackedallMeasurements.clear();

                st = clock();

                if (!init)//第一帧初始化
                {
                    for (int j = 0; j < clusterSize; j++) {
                        DynaTools::clusterTracker trackerTmp(j, pcClusterCenters->points[j]);
                        trackersList.push_back(trackerTmp);
                    }
                    init = true;

                } else {
                    for (auto it = trackersList.begin(); it != trackersList.end();) {

                        bool tracked = false;
                        predicpt = it->predicState();//z=Vx, intensity=Vy
                        pcXYZI lastmeasurept = it->getpreMeasurements();

                        //根据预测值寻找当前帧对应的measurement--最近点
                        //如果ID=0,证明当前帧无对应点
                        nearestID = DynaTools::findNearestNeighborIndice(predicpt, pcClusterCenters);
                        cout << "NNpt ID " << nearestID;
                        pcl::PointXYZI nearestPtcentro = pcClusterCenters->points[nearestID];
                        nearestDis = dbscan2d::distance(predicpt.x, predicpt.y,
                                                        nearestPtcentro.x,
                                                        nearestPtcentro.y);
                        cout << " >> nearest distance in 2D: " << nearestDis << endl;

                        if (nearestDis < 0.6) {
                            it->addmeasurements(nearestPtcentro);
                            tracked = true;

                            //Vx > ? \\  || predicpt.z > 0
                            if (abs(predicpt.z) > 0.6 || abs(predicpt.intensity) > 0.6){
//                        if( predicpt.z > 0 || predicpt.z < -2){
                                pcClusterCentersTracked->push_back(pcClusterCenters->points[nearestID]);
                                pcClusterMinsTracked->push_back(pcClusterMins->points[nearestID]);
                                pcClusterMaxsTracked->push_back(pcClusterMaxs->points[nearestID]);
                                vecTrackedallMeasurements.push_back(lastmeasurept);//for track visualization

                                if (it->getTrackedtimes() > 3)
                                    DynaTools::removepointsInBoundingbox(transformedScan,
//                                    DynaTools::removepointsByCropBox(transformedScan,
                                                                     pcClusterMins->points[nearestID],
                                                                     pcClusterMaxs->points[nearestID],
                                                                     dynaPtsIndices);

                            }

                            pcClusterCenters->points[nearestID].intensity = 9999;//标记已跟踪中心点
                        }

                        //if lost in current scan
                        if (!tracked) {
                            //tracked times < ?
                            if (it->getTrackedtimes() == -1) {
                                it = trackersList.erase(it);
                            } else
                                it++;
                        } else
                            it++;
                    }

                    //old tracklist size
                    int previousTrackerSize = trackersList.size();
                    cout << "\n New trackers >> >> >>  pevious Trackerlist : " << previousTrackerSize << endl;
                    for (int j = 0; j < clusterSize; j++) {
                        //new cluster new tracks
                        if (pcClusterCenters->points[j].intensity < 9000) {
                            DynaTools::clusterTracker trackerTmp(previousTrackerSize, pcClusterCenters->points[j]);
                            trackersList.push_back(trackerTmp);
                            previousTrackerSize++;
                        }
                    }

                }

                et = clock();
                ut = double(et - st) / CLOCKS_PER_SEC;
                printf("Time used is :%f s for tracking. \n===============================================", ut);

//                ccviewer.showCloud(transformedScan);

                cout << "\n\n### The number of dynamic points is " << dynaPtsIndices.size() << endl;
                *globalmap += *transformedScan;

                if(!dynaPtsIndices.empty() )
                    filterOutFromCloudByIndices(scan, dynaPtsIndices);
                dynaPtsIndices.clear();
//                pcl::io::savePCDFile("/home/joe/workspace/testData/veloScansNodyna/"+
//                                     to_string(pcRPYpose->points[i].time) + ".pcd", *scan);

                //visualizaiton
                if (pcClusterCenters->points.size() > 3) {
//                pcl::io::savePCDFile("/home/cyz/Data/legoloam/poses/scanClusterCentro/scan"+to_string(i)+".pcd"
//                        ,*pcClusterCenters);

                    pcl::copyPointCloud(*pcClusterMins, *DynaTools::pcCluster_Mins);
                    pcl::copyPointCloud(*pcClusterMaxs, *DynaTools::pcCluster_Maxs);
//                    pcl::copyPointCloud(*pcClusterCenters, *DynaTools::pcCluster_Centros);
                    pcl::copyPointCloud(*pcClusterCentersTracked, *DynaTools::pcCluster_Centros);
                    DynaTools::vecTrackedallmeasurements_.swap(vecTrackedallMeasurements);
//                ccviewer.runOnVisualizationThread(tools::viewClusterbox);
                    ccviewer.runOnVisualizationThreadOnce(DynaTools::viewClusterbox);
//                    boost::this_thread::sleep (boost::posix_time::microseconds (300000));//sleep for a while
                    ccviewer.removeVisualizationCallable();
                    continue;
                }

            }


        }
//        if (!globalmap->empty())
//            pcl::io::savePCDFile("/home/joe/workspace/testData/mapwithoutdyn.pcd", *globalmap);
        globalmap->clear();

        return 0;
    }
    /**
     * tracker部分结束
     */


    /**
     * 测试部分
     */
//    {
//        ///octreeChangeDetectorTest:
//        st = clock();
//        float resol = 0.1f;
//        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octreChangeDect(resol);
//        pcXYZIptr scan1 = clouds[1].makeShared();
//        pcXYZIptr scan2 = clouds[2].makeShared();
//
//        octreChangeDect.setInputCloud(scan1);
//        octreChangeDect.addPointsFromInputCloud();
//
//        octreChangeDect.switchBuffers();
//
//        octreChangeDect.setInputCloud(scan2);
//        octreChangeDect.addPointsFromInputCloud();
//
//        vector<int> newPtIndic;
//        octreChangeDect.getPointIndicesFromNewVoxels(newPtIndic);
//
//        cout << "new points :" << newPtIndic.size() << endl;
//        pcXYZIptr newptsCloud(new pcXYZI());
//        for (int i = 0; i < newPtIndic.size(); i++)
//            newptsCloud->push_back(scan2->points[newPtIndic[i]]);
//        clusterSize = dbscan2d::dbscan(newptsCloud, 0.17, 30);
//        pcl::io::savePCDFile(dataFolder + "/changedptsCloud.pcd", *newptsCloud);
//
//        et = clock();
//        ut = double(et - st) / CLOCKS_PER_SEC;
//        printf("time used is :%f s for changeDetection\n", ut);
//
//        pcl::PointCloud<pcl::PointXYZL>::Ptr pcWith_labl(new pcl::PointCloud<pcl::PointXYZL>());
//        pcl::PointXYZL ptlabl;
//        for (int i = 0; i < newptsCloud->points.size(); i++)
//            if (newptsCloud->points[i].intensity != 0) {
//                ptlabl.x = newptsCloud->points[i].x;
//                ptlabl.y = newptsCloud->points[i].y;
//                ptlabl.z = newptsCloud->points[i].z;
//                ptlabl.label = (int) newptsCloud->points[i].intensity;
//                pcWith_labl->push_back(ptlabl);
//            }
//
//        pcl::visualization::PCLVisualizer viewer;
//        pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL> pchandlerLabl;
//        pchandlerLabl.setInputCloud(pcWith_labl);
//        viewer.addPointCloud(pcWith_labl, pchandlerLabl, "newptsCLoud");
//        while (!viewer.wasStopped()) {
//            viewer.spin();
//        }
//
//
//        ///octomapTest:
//        octomap::ColorOcTree colorTree(0.1);
//        octomap::ColorOcTreeNode *node = nullptr;
//        pcXYZIptr changedcloud(new pcXYZI());
//        st = clock();
//        for (int i = 0; i < 3; i++) {
//            pcl::removeNaNFromPointCloud(clouds[i], clouds[i], labels);
//
//            labels.clear();
////        int clusterSize = dbscan( clouds[i], labels, 0.3, 16);
////        cout<<"DNSCAN size :"<<clusterSize<<endl;
//
//            for (auto p:clouds[i].points) {
//                if (abs(p.x) > 10 || abs(p.y) > 10)
//                    continue;
//
//                if (i == 0)
//                    colorTree.updateNode(octomap::point3d(p.x, p.y, 0), true);
//                else {
//                    node = colorTree.search(octomap::point3d(p.x, p.y, 0));
//                    if (node == nullptr) {
//                        changedcloud->push_back(p);
//                        colorTree.updateNode(octomap::point3d(p.x, p.y, 0), true);
//                        node = colorTree.search(octomap::point3d(p.x, p.y, 0));
//                        node->setColor(255, 0, 0);
////                    octomap::OcTreeKey key = colorTree.coordToKey(octomap::point3d(p.x, p.y, 0), 1);
////                    cout <<"key index: "<< key.k[0] <<", "<< key.k[1] <<", "<< key.k[2] <<endl;
//
//                    }
//
//                }
//
////            vector<octomap::point3d> rayintersec_points;
////            if(colorTree.computeRay(octomap::point3d(0,0,0), octomap::point3d(p.x, p.y, 0), rayintersec_points)) {
////                for(int i=0; i<rayintersec_points.size(); i++) {
////                    node = colorTree.search(rayintersec_points[i]);
////                    if (node == nullptr)
////                        continue;
////                    if (colorTree.isNodeOccupied(node))
////                    {
////                        //      colorTree.deleteNode(rayintersec_points[i],0);
////                        cout << "coming trough..." << rayintersec_points[i] << endl;
////                        node->setColor(255, 0, 0);
//////                        node->setValue(0);
////                    }
////                }
////            }
////            rayintersec_points.clear();
//
////            colorTree.insertRay(octomap::point3d(0,0,0), octomap::point3d(p.x, p.y, 0));
////            node = colorTree.search(octomap::point3d(p.x, p.y, 0));
//
////            cout<<"occupancy probability： "<<node->getOccupancy()<<endl;
////             cout<<"node value： "<<node->getValue()<<endl;
////            double grey = 255.0 * node->getOccupancy() ;
////            colorTree.integrateNodeColor(p.x, p.y, 0, grey, grey, grey);
////            if(node->getOccupancy() > 0.9)
////                colorTree.integrateNodeColor(p.x, p.y, 0, 255, 0, 0);
////            else
////                colorTree.integrateNodeColor(p.x, p.y, 0, 0, 255, 0);
//
//            }
//
//            colorTree.updateInnerOccupancy();
//
//            datanum++;
//            if (datanum > 2)
//                break;
//
//        }
//        pcl::io::savePCDFile(dataFolder + "/changedCloudoctomap.pcd", *changedcloud);
//        et = clock();
//        ut = double(et - st) / CLOCKS_PER_SEC;
//        printf("time used is :%f s for octomap\n", ut);
//        colorTree.write("/home/cyz/Data/legoloam/seg_pure/octo.ot");
//
//
//        return 0;
//    }

}