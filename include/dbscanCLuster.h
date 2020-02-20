//
// Created by cyz on 19-4-2.
//

#ifndef POINTCLOUDPROCESSING_DBSCANCLUSTER_H
#define POINTCLOUDPROCESSING_DBSCANCLUSTER_H


#include "commontools.h"
#include "smoothing.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/octree/octree_search.h>

#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>

//#include "curves/PolynomialSplineVectorSpaceCurve.hpp"
#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
//#include <opencv2/legacy/legacy.hpp>

using namespace cv;


namespace dbscan3d {
    static const inline double distance(pcl::PointXYZI p1, pcl::PointXYZI p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;

        return sqrt(dx * dx + dy * dy + dz*dz);
    }

    const inline int region_query(const pcXYZIptr input, int p, std::vector<int> &output, double eps) {

        for(int i = 0; i < (int)input->points.size(); i++){

            if(distance(input->points[i], input->points[p]) < eps){
                output.push_back(i);
            }
        }

        return output.size();
    }

    bool expand_cluster(const pcXYZIptr input, int p, std::vector<int> &output, int cluster, double eps, int min) {

        std::vector<int> seeds;

        if(region_query(input, p, seeds, eps) < min){

            //this point is noise
            output[p] = -1;
            return false;

        }else{

            //set cluster id
            for(int i = 0; i < (int)seeds.size(); i++){
                output[seeds[i]] = cluster;
            }

            //delete paint from seeds
            seeds.erase(std::remove(seeds.begin(), seeds.end(), p), seeds.end());

            //seed -> empty
            while((int)seeds.size() > 0){

                int cp = seeds.front();
                std::vector<int> result;

                if(region_query(input, cp, result, eps) >= min){

                    for(int i = 0; i < (int)result.size(); i++){

                        int rp = result[i];

                        //this paint is noise or unmarked point
                        if(output[rp] < 1){

                            //unmarked point
                            if(!output[rp]){
                                seeds.push_back(rp);
                            }

                            //set cluster id
                            output[rp] = cluster;
                        }
                    }
                }

                //delete point from seeds
                seeds.erase(std::remove(seeds.begin(), seeds.end(), cp), seeds.end());
            }
        }

        return true;
    }

    int dbscan(pcXYZIptr input, double eps, int min) {

        int size = input->points.size();
        int cluster = 1;

        std::vector<int> state(size);

        for(int i = 0; i < size; i++){

            if(!state[i]){

                if(expand_cluster(input, i, state, cluster, eps, min)){
                    cluster++;
                }
            }
        }

        for(int i=0 ; i<size ; i++)
        {
            if(state[i] > 0)
                input->points[i].intensity = state[i] * 1.0;
            else
                input->points[i].intensity = 0.0;
        }

        return cluster - 1;
    }
}
namespace dbscan2d{//too slow !

    static const inline double distance(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;

        return sqrt(dx * dx + dy * dy);
    }

    //返回p点eps半径内点数
    const inline int region_query(const pcXYZIptr input, int p, std::vector<int> &output, double eps)
    {
        vector<float > dists;
        pcl::KdTreeFLANN<pcl::PointXYZI> kdTreeFlann;
        kdTreeFlann.setInputCloud(input);
        kdTreeFlann.radiusSearch(input->points[p], eps, output, dists);
        dists.clear();

//        for(int i = 0; i < (int)input->points.size(); i++)
//            if(distance(input->points[i].x, input->points[i].y, input->points[p].x, input->points[p].y) < eps)
//                output.push_back(i);

        return output.size();
    }

    bool expand_cluster(const pcXYZIptr input, int p, std::vector<int> &output, int cluster, double eps, int min)
    {
        std::vector<int> seeds;//邻域点indices

        if(region_query(input, p, seeds, eps) < min){
            //this point is noise
            output[p] = -1;
            return false;

        }else{

            //set cluster id
            for(int i = 0; i < (int)seeds.size(); i++)
                output[seeds[i]] = cluster;

            //delete paint from seeds
            seeds.erase(std::remove(seeds.begin(), seeds.end(), p), seeds.end());

            //seed -> empty
            while((int)seeds.size() > 0){

                int cp = seeds.front();
                std::vector<int> result;

                if(region_query(input, cp, result, eps) >= min){

                    for(int i = 0; i < (int)result.size(); i++){

                        int rp = result[i];

                        //this point is noise or unmarked point
                        if(output[rp] < 1){

                            //unmarked point
                            if(!output[rp])
                                seeds.push_back(rp);

                            //set cluster id
                            output[rp] = cluster;
                        }
                    }
                }

                //delete point from seeds
                seeds.erase(std::remove(seeds.begin(), seeds.end(), cp), seeds.end());
            }
        }

        return true;
    }

    /// DBSCAN 密度聚类: 最小半径内含有不少于最少点的点为中心点，位于中心点搜索半径内的点为边界点
    /// \param input 输入点云
    /// \param eps 搜索半径
    /// \param min 半径内最少点
    /// \return 簇的个数
    int dbscan( pcXYZIptr input, double eps, int min){

        int size = input->points.size();
        int cluster = 1;

        pcXYZIptr cloud2d(new pcXYZI());
        pcl::copyPointCloud(*input, *cloud2d);
        for (int j = 0; j < size; ++j)
            cloud2d->points[j].z = 0;

        std::vector<int> state(size);
//        int *state;
//        state = new int[size];

        for(int i = 0; i < size; i++)
            if(!state[i])
                if(expand_cluster(cloud2d, i, state, cluster, eps, min))
                    cluster++;

        for(int i=0 ; i<size ; i++){

            if(state[i] > 0)
                input->points[i].intensity = state[i] * 1.0;
            else
                input->points[i].intensity = 0.0;
        }

        return cluster - 1;
    }
}

namespace DynaTools{//tools for dynaObjTest

    // kdtreeFLANN accelerated DBSCAN
    int clusterDBSCAN(pcXYZIptr inCloud, double r, int minptNum,
                      pcXYZIptr pcClusterCentos,
                      pcXYZIptr pcClusterMinpts,
                      pcXYZIptr pcClusterMaxpts){

        int size = inCloud->points.size();
        int clusterID = 0;
        std::vector<int> state(size);//初始化为0,-1为visited,-2为噪点

        pcXYZIptr cloud2d(new pcXYZI());
        pcl::copyPointCloud(*inCloud, *cloud2d);
        for (int j = 0; j < size; ++j)
            cloud2d->points[j].z = 0;

        std::vector<float > dists;
        std::vector<int > neighbors;
        std::vector<int > neighborsTmp;
        pcl::KdTreeFLANN<pcl::PointXYZI> kdTreeFlann;
        kdTreeFlann.setInputCloud(cloud2d);

        for (int i = 0; i < size; ++i) {

            if(state[i] != 0)
                continue;

            state[i] = -1;//is visited
            kdTreeFlann.radiusSearch(cloud2d->points[i], r, neighbors, dists);
            if(neighbors.size() < minptNum)
                state[i] = -2;// NOISE
            else{
                clusterID ++;
                state[i] = clusterID;
                for (int j = 0; j < neighbors.size(); ++j) {

                    if(state[neighbors[j]] == 0){//is not visited
                        state[neighbors[j]] = -1;
                        kdTreeFlann.radiusSearch(cloud2d->points[neighbors[j]], r, neighborsTmp, dists);
                        if(neighborsTmp.size() >= minptNum)
                            neighbors.insert(neighbors.end(), neighborsTmp.begin(), neighborsTmp.end());
                    }
                    if(state[neighbors[j]] == -1)//is not a member of any cluster
                        state[neighbors[j]] = clusterID;
                }
            }

        }

//        for(int i=0 ; i<size ; i++){
//
//            if(state[i] > 0)
//                inCloud->points[i].intensity = state[i] * 1.0;
//            else
//                inCloud->points[i].intensity = 0.0;
//        }

        std::vector<pcl::PointIndices> clusters(clusterID);

        for(int i=0 ; i<size ; i++)
            if(state[i] > 0)
                clusters[state[i]-1].indices.push_back(i);

        pcXYZIptr pcSingleCluster(new pcXYZI());
        Eigen::Vector4f centroVec;
        pcl::PointXYZI centro, max, min;


        if(clusterID > 3)
            for(int i=0; i<clusterID; i++){

                for(int j=0; j<clusters[i].indices.size(); j++)
                    pcSingleCluster->push_back(inCloud->points[clusters[i].indices[j]]);

                pcl::compute3DCentroid(*pcSingleCluster,centroVec);
                pcl::getMinMax3D(*pcSingleCluster, min, max);
                pcSingleCluster->clear(); // !!!

                if(max.x-min.x>4 || max.y-min.y>4 || max.z-min.z<0.3 )
                    continue;
//                if(max.x-min.x>8 || max.y-min.y>8 || max.z-min.z<0.6 || max.z-min.z>5)
//                    continue;
                centro.x = centroVec[0];
                centro.y = centroVec[1];
                centro.z = centroVec[2];
                pcClusterCentos->push_back(centro);
                pcClusterMinpts->push_back(min);
                pcClusterMaxpts->push_back(max);
            }

        return clusterID;
    }



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


    /**
     * from lcas/onlinemapping at github
     * Adapative EuclideanClustering by setting range rings
     */
    const int nested_regions_ = 3;
//    const int nested_regions_ = 14;
//    int zone_[nested_regions_] = {2,3,3,3,3,3,3,2,3,3,3,3,3,3}; // for more details, see our IROS'17 paper.
    int zone_[nested_regions_] = {5,3,3};
    void extractClusterAdaptiveEuclidean(pcXYZIptr pc, pcXYZIptr pcClusterCentos) {
//        features_.clear();
        float z_limit_min_= -0.3;
        float z_limit_max_= 5;
        int cluster_size_min_ = 30;
        int cluster_size_max_ = 1000;

        pcl::IndicesPtr pc_indices(new std::vector<int>);
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(pc);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_limit_min_, z_limit_max_);//依据高程去地面
        pass.filter(*pc_indices);

        //投影到地面上
        for(int i=0; i<pc->points.size(); i++)
            pc->points[i].z=0;

        boost::array<std::vector<int>, nested_regions_> indices_array;
        for(int i = 0; i < pc_indices->size(); i++) {
            float range = 0.0;
            for(int j = 0; j < nested_regions_; j++) {
                float d2 = pc->points[(*pc_indices)[i]].x * pc->points[(*pc_indices)[i]].x +
                           pc->points[(*pc_indices)[i]].y * pc->points[(*pc_indices)[i]].y +
                           pc->points[(*pc_indices)[i]].z * pc->points[(*pc_indices)[i]].z;
                if(d2 > range*range && d2 <= (range+zone_[j])*(range+zone_[j])) {
                    indices_array[j].push_back((*pc_indices)[i]);
                    break;
                }
                range += zone_[j];
            }
        }

        float tolerance = 0.3;
        for(int i = 0; i < nested_regions_; i++) {
            tolerance += 0.1;
            if(indices_array[i].size() > cluster_size_min_) {
                boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
                pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
                tree->setInputCloud(pc, indices_array_ptr);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
                ec.setClusterTolerance(tolerance);
                ec.setMinClusterSize(cluster_size_min_);
                ec.setMaxClusterSize(cluster_size_max_);
                ec.setSearchMethod(tree);
                ec.setInputCloud(pc);
                ec.setIndices(indices_array_ptr);
                ec.extract(cluster_indices);

                for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
                    it != cluster_indices.end(); it++) {
                    pcXYZIptr cluster(new pcXYZI());
                    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                        cluster->points.push_back(pc->points[*pit]);
                    cluster->width = cluster->size();
                    cluster->height = 1;
                    cluster->is_dense = true;

//                    Eigen::Vector4f min, max, centroid;
                    pcl::PointXYZI min, max, centroid;
                    Eigen::Vector4f ceterVec;
                    pcl::getMinMax3D(*cluster, min, max);

                    if(max.x-min.x > 1 || max.y-min.y > 1)
                        continue;
                    pcl::compute3DCentroid(*cluster, ceterVec);
                    centroid.x = ceterVec[0];
                    centroid.y = ceterVec[1];
                    centroid.z = ceterVec[2];

                    pcClusterCentos->push_back(centroid);
                    // Size limitation is not reasonable, but it can increase fps.
//                    if(human_size_limit_ &&
//                       (max[0]-min[0] < 0.2 || max[0]-min[0] > 1.0 ||
//                        max[1]-min[1] < 0.2 || max[1]-min[1] > 1.0 ||
//                        max[2]-min[2] < 0.5 || max[2]-min[2] > 2.0))
//                        continue;
//
//                    Feature f;
//                    extractFeature(cluster, f, min, max, centroid);
//                    features_.push_back(f);
                }
            }
        }
    }


    /**
     * 截取点云 降采样 简单欧氏距离聚类
     * @param pc //input point cloud
     * @param pcClusterCentos //cluster centro point
     * @param pcClusterMinpts //bounding box min point
     * @param pcClusterMaxpts //bounding box max point
     * @return  //size of clusters
     */
    int extractClusterEuclidean(const pcXYZIptr pc, double disThre,
                                pcXYZIptr pcClusterCentos,
                                pcXYZIptr pcClusterMinpts,
                                pcXYZIptr pcClusterMaxpts){

//        pcl::ConditionAnd<pcl::PointXYZI>::Ptr condi(new pcl::ConditionAnd<pcl::PointXYZI>());
//        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
//                                     (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT, -16.0)));
//        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
//                                     (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT, 16.0)));
//        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
//                                     (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT, -10.0)));
//        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
//                                     (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT, 10.0)));
//        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
//                                     (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::GT, -0.3)));
//        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
//                                     (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::LT, 5.0)));
//        pcl::ConditionalRemoval<pcl::PointXYZI> condRem;
//        condRem.setCondition(condi);
//        condRem.setInputCloud(pc);
//        condRem.setKeepOrganized(true);
//        condRem.filter(*pc);
//        cout<<"pc size in pov :"<<pc->points.size()<<endl;

//        pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
//        voxelGrid.setLeafSize(0.1, 0.1, 0.1);
//        voxelGrid.setInputCloud(pc);
//        voxelGrid.filter(*pc);
//        cout<<"pc size after voxel down-sample :"<<pc->points.size()<<endl;

//        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radRemv;
//        radRemv.setInputCloud(pc);
//        radRemv.setRadiusSearch(0.3);
//        radRemv.setMinNeighborsInRadius(5);
//        radRemv.filter(*pc);

        pcXYZIptr pcNoheights(new pcXYZI());///z=0 for better cluster
        pcl::copyPointCloud(*pc, *pcNoheights);
        for(int i=0; i<pc->points.size(); i++)
            pcNoheights->points[i].z = 0;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(pc);
//        tree->setInputCloud(pcNoheights);
        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> eucliClusterExt;
        eucliClusterExt.setClusterTolerance(disThre);
        eucliClusterExt.setMinClusterSize(20);
        eucliClusterExt.setMaxClusterSize(4000);
        eucliClusterExt.setSearchMethod(tree);
//        eucliClusterExt.setInputCloud(pcNoheights);
        eucliClusterExt.setInputCloud(pc);
        eucliClusterExt.extract(clusters);

        cout<<"Original Euclidean cluster size : "<<clusters.size()<<endl;
        pcXYZIptr pcSingleCluster(new pcXYZI());
        Eigen::Vector4f centroVec;
        pcl::PointXYZI centro, max, min;

        if(clusters.size() > 3)
            for(int i=0; i<clusters.size(); i++){

                for(int j=0; j<clusters[i].indices.size(); j++)
                    pcSingleCluster->push_back(pc->points[clusters[i].indices[j]]);

                pcl::compute3DCentroid(*pcSingleCluster,centroVec);
                pcl::getMinMax3D(*pcSingleCluster, min, max);
                pcSingleCluster->clear(); // !!!

                if(max.x-min.x>6 || max.y-min.y>6 || max.z-min.z<0.4 )
                    continue;
//                if(max.x-min.x>8 || max.y-min.y>8 || max.z-min.z<0.6 || max.z-min.z>5)
//                    continue;
                centro.x = centroVec[0];
                centro.y = centroVec[1];
                centro.z = centroVec[2];
                pcClusterCentos->push_back(centro);
                pcClusterMinpts->push_back(min);
                pcClusterMaxpts->push_back(max);
            }

        return pcClusterCentos->points.size();

    }


    ///remove points in bounding box by octreeBoxSearch
    void removepointsInBoundingbox(const pcXYZIptr pcIn, pcl::PointXYZI minpt, pcl::PointXYZI maxpt,
                                   std::vector<int>& indices){

        std::vector<int> indices_;
        int ptInboxNum = 0;
        Eigen::Vector3f buffer(0.3, 0.3, 0.3);
//        Eigen::Vector3f buffer(0, 0, 0);

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octreePointCloudSearch(0.1);
        octreePointCloudSearch.setInputCloud(pcIn);
        octreePointCloudSearch.addPointsFromInputCloud();// dont forget !!!
        ptInboxNum = octreePointCloudSearch.boxSearch(minpt.getVector3fMap()-buffer,
                                                      maxpt.getVector3fMap()+buffer, indices_);
        cout<<"\n-- Dynamic points num "<<ptInboxNum<<endl;

        for (int i = 0; i < ptInboxNum; ++i)
            indices.push_back(indices_[i]);

//        filterOutFromCloudByIndices(pcIn, indices_);

//        pcXYZIptr pcfilteredOut(new pcXYZI());
//        for (int i = 0; i < ptInboxNum; ++i) {
//            double buffer = 0.5;//delete buffer(m)
//            if(pcIn->points[i].x > minpt.x-buffer &&
//               pcIn->points[i].y > minpt.y-buffer &&
//               pcIn->points[i].z > minpt.z
//               && pcIn->points[i].x < maxpt.x+buffer
//               && pcIn->points[i].y < maxpt.y+buffer
//               && pcIn->points[i].z < maxpt.z+buffer)
//                    continue;
//            pcfilteredOut->push_back(pcIn->points[i]);
//        }
//        pcIn->clear();
//        pcl::copyPointCloud(*pcfilteredOut, *pcIn);
    }

    /// TODO cant work ?
    void removepointsByCropBox(pcXYZIptr pcIn, pcl::PointXYZI minpt, pcl::PointXYZI maxpt){

        std::vector<int> indices;
        double buffer = 0.5 ;
//        pcXYZIptr pcfilteredOut(new pcXYZI());
        pcl::CropBox<pcl::PointXYZI>::Ptr cropper(new pcl::CropBox<pcl::PointXYZI>(true));
        cropper->setInputCloud(pcIn);
        cropper->setMin(Eigen::Vector4f(minpt.x-buffer, minpt.y-buffer, minpt.z-buffer, 1.0));
        cropper->setMax(Eigen::Vector4f(maxpt.x+buffer, maxpt.y+buffer, maxpt.z+buffer, 1.0));
        cropper->setNegative(true);

        cropper->filter(indices);

        cout<<"\n-- Dynamic points num "<<indices.size()<<endl;
//        pcl::copyPointCloud(*pcfilteredOut, *pcIn);
        for (int i = 0; i < indices.size(); ++i)
            pcIn->points[indices[i]].intensity = 150;

    }


    /**
     * just for visualization multiple bounding boxes
     */
    pcXYZIptr pcCluster_Centros(new pcXYZI());//tracked centros
    pcXYZIptr pcCluster_Mins(new pcXYZI());
    pcXYZIptr pcCluster_Maxs(new pcXYZI());
//    pcXYZIptr pclastmeasurements_(new pcXYZI());
    std::vector<pcXYZI> vecTrackedallmeasurements_;
    void viewClusterbox(pcl::visualization::PCLVisualizer& viewer){

        viewer.removeAllShapes();

        for(int i=0; i<pcCluster_Mins->points.size(); i++){

//            Eigen::Vector3f centro;
//            centro[0] = pcCluster_Centros->points[i].x;
//            centro[1] = pcCluster_Centros->points[i].y;
//            centro[2] = pcCluster_Centros->points[i].z;
//            const Eigen::Quaternionf noRotation(Eigen::Quaternionf::Identity());
//            viewer.addCube(centro, noRotation, 1,1,1, std::to_string(i+100));
//
            viewer.addCube(pcCluster_Mins->points[i].x, pcCluster_Maxs->points[i].x,
                           pcCluster_Mins->points[i].y, pcCluster_Maxs->points[i].y,
                           pcCluster_Mins->points[i].z, pcCluster_Maxs->points[i].z,
                           0,0,255, std::to_string(i));
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                               pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                               std::to_string(i));


        }

        //tracked centers
        Eigen::Vector3f centro;
        for(int i=0; i<pcCluster_Centros->points.size(); i++){

//            centro[0] = pcCluster_Centros->points[i].x;
//            centro[1] = pcCluster_Centros->points[i].y;
//            centro[2] = pcCluster_Centros->points[i].z;
//            const Eigen::Quaternionf noRotation(Eigen::Quaternionf::Identity());
//            viewer.addCube(centro, noRotation, 1,1,1, std::to_string(i+100));

            for (int j = 0; j < vecTrackedallmeasurements_[i].points.size()-1; ++j) {

                viewer.addLine(vecTrackedallmeasurements_[i].points[j],
                               vecTrackedallmeasurements_[i].points[j+1], 0,255,0, std::to_string(i+1000+j*30));

                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5,
                                                   std::to_string(i+1000+j*30));
            }


        }

        viewer.spinOnce(100);

        pcCluster_Centros.reset(new pcXYZI());
        pcCluster_Mins.reset(new pcXYZI());
        pcCluster_Maxs.reset(new pcXYZI());
        vecTrackedallmeasurements_.clear();
    }



    /***
     * 寻找最近点的位置
     * @param source
     * @param pctargetlist
     * @return
     */
    int findNearestNeighborIndice(pcl::PointXYZI source, pcXYZIptr pctargetlist){

        double mindis=100;
        int listsize= pctargetlist->points.size();

        pcl::PointXYZI curpt;
        std::vector<double> curdis;
        curdis.resize(listsize, 0);
//        std::vector<double>().swap(curdis);
        for(int i=0; i<listsize; i++)
        {
            curpt = pctargetlist->points[i];
            if( curpt.intensity > 9000)
                continue;
            curdis[i] = (dbscan2d::distance(source.x, source.y, curpt.x, curpt.y));
            if(curdis[i] < mindis)
                mindis = curdis[i];
        }

        for(int i=0; i<listsize; i++)
            if(curdis[i] == mindis)
                return i;
        return 0;//if no points left

    }


    /**
     * each tracker corresponding to one cluster with one KF
     */
    class clusterTracker{

    private:
        pcXYZIptr centrosMeasurements_pc;//x,y
        pcXYZIptr centrosPredicsXYVxVy_pc;//z=Vx, intensity=Vy
        int trackedtimes;
        int lostTimes;
        int trackerID;
        bool isTracked;
        bool isLost;
        KalmanFilter kalmantracker;
        pcl::PointXYZI predpt;


    public:
        clusterTracker(int id, pcl::PointXYZI centro)
        {
            initialization();
            centrosMeasurements_pc->points.push_back(centro);
            trackerID = id;

            //tracker 初始化（仅执行一次）
            kalmantracker.init(4, 2, 0);
            kalmantracker.transitionMatrix = (Mat_<float>(4,4)<<1,0,0.1,0, 0,1,0,0.1, 0,0,1,0, 0,0,0,1);//delatT=0.1
            setIdentity(kalmantracker.measurementMatrix);
            setIdentity(kalmantracker.processNoiseCov, Scalar::all(1e-5));
            setIdentity(kalmantracker.measurementNoiseCov, Scalar::all(1e-1));
            setIdentity(kalmantracker.errorCovPost, Scalar::all(1));
            kalmantracker.statePost= (Mat_<float>(4,1)<< centro.x, centro.y, 0 ,0);//初始速度为0
            cout<<"kalman tracker initialized. ID : "<<trackerID<<endl;

        }

        void initialization() {

            isTracked = false;
            isTracked = false;
            trackedtimes = 0;
            lostTimes = 0;
            centrosMeasurements_pc.reset( new pcXYZI());
            centrosPredicsXYVxVy_pc.reset( new pcXYZI());

        }

        ~clusterTracker(){}


        pcl::PointXYZI predicState(){

            Mat pred = kalmantracker.predict();
            predpt.x = pred.at<float>(0);
            predpt.y = pred.at<float>(1);
            predpt.z = pred.at<float>(2);
            predpt.intensity = pred.at<float>(3);
            if(!isLost)
                centrosPredicsXYVxVy_pc->points.push_back(predpt);

            cout<<"\n-----------tracker ID : "<<trackerID<<"---------------"<<endl;
            cout<<"-predicted position: ("<<pred.at<float>(0)<<", "<<pred.at<float>(1)<<") "<<endl;
            cout<<"---predicted Velocity: ("<<pred.at<float>(2)<<", "<<pred.at<float>(3)<<") "<<endl;

            return predpt;
        }

        pcXYZI getpreMeasurements(){

//            int size = centrosMeasurements_pc->points.size();
            return *centrosMeasurements_pc;
        }

        void addmeasurements(pcl::PointXYZI centroMeasure){

            isTracked = true;
            isLost = false;
            centrosMeasurements_pc->points.push_back(centroMeasure);
            trackedtimes++;

            Mat measurement = Mat::zeros(2, 1, CV_32F);
            measurement.at<float>(0) = centroMeasure.x;
            measurement.at<float>(1) = centroMeasure.y;

            kalmantracker.correct(measurement);
            cout<<">>>tracked object ID "<<trackerID<<" ---"<<trackedtimes<<" times so far."<<endl;
//            cout<<"corrected: ("<<measurement.at<float>(0)<<", "<<measurement.at<float>(1)<<") "<<endl;
            lostTimes =0 ;
        }


        int getTrackedtimes(){

            isLost = true;
            lostTimes++;
            if(lostTimes > 3){//erase this tracker !

//                if(trackedtimes > 30){//保存losted跟踪的轨迹信息
//                    fstream checkfile ;
//                    std::string path = "/home/cyz/Data/legoloam/poses/trackinginfo/"+std::to_string(trackerID)+".txt";
//                    checkfile.open(path, ios::in);
//
//                    if(!checkfile){
//
//                    }else{
//                        path = "/home/cyz/Data/legoloam/poses/trackinginfo/"+std::to_string(trackerID)+"_"+
//                               std::to_string(random())+".txt";
//                    }
//                    checkfile.close();
//
//                    int measureSize = centrosMeasurements_pc->points.size();
//                    int predicSize = centrosPredicsXYVxVy_pc->points.size();
//
////                    cout<<"measureSize : "<<measureSize<<" ; "<<"predicSize : "<<predicSize<<endl;
//
//                    const char* pathtxt = path.c_str();
//                    FILE *fp;
//                    fp = fopen(pathtxt,"w");
//                    for(int i=0; i<measureSize; i++)
//                        fprintf(fp,"%lf %lf %lf %lf %lf %lf\n",
//                                centrosMeasurements_pc->points[i].x, centrosMeasurements_pc->points[i].y,
//                                centrosPredicsXYVxVy_pc->points[i].x, centrosPredicsXYVxVy_pc->points[i].y,
//                                centrosPredicsXYVxVy_pc->points[i].z, centrosPredicsXYVxVy_pc->points[i].intensity);
//
//                    fclose(fp);
//                }
                return -1;
            }

            else
                return trackedtimes;
        }


    };
}
#endif //POINTCLOUDPROCESSING_DBSCANCLUSTER_H
