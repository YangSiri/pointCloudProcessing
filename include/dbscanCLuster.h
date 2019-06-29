//
// Created by cyz on 19-4-2.
//

#ifndef POINTCLOUDPROCESSING_DBSCANCLUSTER_H
#define POINTCLOUDPROCESSING_DBSCANCLUSTER_H

#include <math.h>
#include "smoothing.h"
#include <algorithm>
#include <stdio.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>

#include "curves/PolynomialSplineVectorSpaceCurve.hpp"
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

namespace dbscan2d{
    static const inline double distance(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;

        return sqrt(dx * dx + dy * dy);
    }

    const inline int region_query(const pcXYZIptr input, int p, std::vector<int> &output, double eps)
    {
        for(int i = 0; i < (int)input->points.size(); i++){

            if(distance(input->points[i].x, input->points[i].y, input->points[p].x, input->points[p].y) < eps){
                output.push_back(i);
            }
        }

        return output.size();
    }

    bool expand_cluster(const pcXYZIptr input, int p, std::vector<int> &output, int cluster, double eps, int min)
    {
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

    int dbscan( pcXYZIptr input, double eps, int min)
    {
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
                                          (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

namespace tools{

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
     * @param pc
     * @param pcClusterCentos
     * @param pcClusterMinpts
     * @param pcClusterMaxpts
     * @return
     */
    int extractClusterEuclidean(pcXYZIptr pc, pcXYZIptr pcClusterCentos,
                                 pcXYZIptr pcClusterMinpts, pcXYZIptr pcClusterMaxpts){

        pcl::ConditionAnd<pcl::PointXYZI>::Ptr condi(new pcl::ConditionAnd<pcl::PointXYZI>());
        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                     (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT, -16.0)));
        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                     (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT, 16.0)));
        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                     (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT, -10.0)));
        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                     (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT, 10.0)));
        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                     (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::GT, -0.3)));
        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                     (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::LT, 5.0)));
        pcl::ConditionalRemoval<pcl::PointXYZI> condRem;
        condRem.setCondition(condi);
        condRem.setInputCloud(pc);
        condRem.setKeepOrganized(true);
        condRem.filter(*pc);
        cout<<"pc size in pov :"<<pc->points.size()<<endl;

        pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
        voxelGrid.setLeafSize(0.1, 0.1, 0.1);
        voxelGrid.setInputCloud(pc);
        voxelGrid.filter(*pc);
        cout<<"pc size after voxel down-sample :"<<pc->points.size()<<endl;

//        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radRemv;
//        radRemv.setInputCloud(pc);
//        radRemv.setRadiusSearch(0.3);
//        radRemv.setMinNeighborsInRadius(5);
//        radRemv.filter(*pc);

        pcXYZIptr pcNoheights(new pcXYZI());///z=0 for better cluster
        pcl::copyPointCloud(*pc, *pcNoheights);
        for(int i=0; i<pc->points.size(); i++)
            pcNoheights->points[i].z=0;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(pc);
        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> eucliClusterExt;
        eucliClusterExt.setClusterTolerance(0.2);
        eucliClusterExt.setMinClusterSize(30);
        eucliClusterExt.setMaxClusterSize(500);
        eucliClusterExt.setSearchMethod(tree);
        eucliClusterExt.setInputCloud(pcNoheights);
        eucliClusterExt.extract(clusters);

        cout<<"Original Euclidean cluster size : "<<clusters.size()<<endl;
        if(clusters.size() > 3)
            for(int i=0; i<clusters.size(); i++)
            {
                pcXYZIptr pcSingleCluster(new pcXYZI);
                for(int j=0; j<clusters[i].indices.size(); j++)
                    pcSingleCluster->push_back(pc->points[clusters[i].indices[j]]);

                Eigen::Vector4f centroVec;
                pcl::PointXYZI centro, max, min;
                pcl::compute3DCentroid(*pcSingleCluster,centroVec);
                pcl::getMinMax3D(*pcSingleCluster, min, max);

                if(max.x-min.x>3 || max.y-min.y>3 || max.z-min.z<0.6 )
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

        for(int i=0; i<pcCluster_Mins->points.size(); i++)
        {

//            Eigen::Vector3f centro;
//            centro[0] = pcCluster_Centros->points[i].x;
//            centro[1] = pcCluster_Centros->points[i].y;
//            centro[2] = pcCluster_Centros->points[i].z;
//            const Eigen::Quaternionf noRotation(Eigen::Quaternionf::Identity());
//            viewer.addCube(centro, noRotation, 1,1,1, std::to_string(i+100));

            viewer.addCube(pcCluster_Mins->points[i].x, pcCluster_Maxs->points[i].x,
                           pcCluster_Mins->points[i].y, pcCluster_Maxs->points[i].y,
                           pcCluster_Mins->points[i].z, pcCluster_Maxs->points[i].z,
                           0,0,255, std::to_string(i));
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                               pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                               std::to_string(i));


        }

        //tracked centers
        for(int i=0; i<pcCluster_Centros->points.size(); i++)
        {
            Eigen::Vector3f centro;
            centro[0] = pcCluster_Centros->points[i].x;
            centro[1] = pcCluster_Centros->points[i].y;
            centro[2] = pcCluster_Centros->points[i].z;
            const Eigen::Quaternionf noRotation(Eigen::Quaternionf::Identity());
            viewer.addCube(centro, noRotation, 1,1,1, std::to_string(i+100));

            for (int j = 0; j < vecTrackedallmeasurements_[i].points.size()-1; ++j) {

                viewer.addLine(vecTrackedallmeasurements_[i].points[j],
                               vecTrackedallmeasurements_[i].points[j+1], 0,255,0, std::to_string(i+1000+j*30));

                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,8,
                                                   std::to_string(i+1000+j*30));
            }
//            viewer.addArrow(pclastmeasurements_->points[i], pcCluster_Centros->points[i], 255,0,0, std::to_string(i+1000));

//            pcl::ModelCoefficients sphere_coeff;
//            sphere_coeff.values.resize (4);    // We need 4 values
//            sphere_coeff.values[0] = pcCluster_Centros->points[i].x;
//            sphere_coeff.values[1] = pcCluster_Centros->points[i].y;
//            sphere_coeff.values[2] = pcCluster_Centros->points[i].z;
//            sphere_coeff.values[2] = 0.6;
//            viewer.addSphere(sphere_coeff,std::to_string(i+100));

            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                               pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                               std::to_string(i+100));

        }

        viewer.spinOnce(100);

        pcCluster_Centros.reset(new pcXYZI());
        pcCluster_Mins.reset(new pcXYZI());
        pcCluster_Maxs.reset(new pcXYZI());
        vecTrackedallmeasurements_.clear();
    }



    /**
     * not finished yet
     * 多项式样条曲线拟合函数
     */
//    typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType ValueType;
    typedef typename curves::PolynomialSplineCubicVector6Curve::ValueType ValueType;
    typedef typename curves::Time Time;
    bool polySplineCubicVec6curveFitting(std::vector<PointTypePose> keyposes){

        curves::PolynomialSplineCubicVector6Curve curveCubic;
//        curves::PolynomialSplineQuinticVector3Curve curveQuin;
        std::vector<Time> times;
        std::vector<ValueType> values;

//        //位移拟合
//        times.push_back(0.0);
////        values.push_back( ValueType(0.0, 0.0, 0.0) );
//        values.push_back(ValueType(keyposes[0].x, keyposes[0].y, keyposes[0].z));
//        times.push_back(0.4);
//        values.push_back(ValueType(keyposes[1].x, keyposes[1].y, keyposes[1].z));
//        times.push_back(0.8);
//        values.push_back(ValueType(keyposes[2].x, keyposes[2].y, keyposes[2].z));
//        times.push_back(1.2);
//        values.push_back(ValueType(keyposes[3].x, keyposes[3].y, keyposes[3].z));
//
//
//        curveCubic.fitCurve(times, values);
//        curveCubic.clear();
//        std::vector<Time>().swap(times);
//        std::vector<ValueType>().swap(values);
//
//        FILE *fp;
//        fp = fopen("/home/cyz/Data/legoloam/poses/keyposesFinalsplined.txt","a");
//        std::vector<PointTypePose> keyposesSplineKnots;
//        ValueType value;
//        keyposesSplineKnots.resize(3);
//        for(double t=times[1]; t<=times[2]; t+=0.1)
//        {
//            curveCubic.evaluate(value,t);
//            keyposesSplineKnots[0].x = value[0];
//        }
//
//
//        //角度拟合
//        times.push_back(0.0);
//        values.push_back(ValueType(keyposes[0].roll, keyposes[0].pitch, keyposes[0].yaw));
//        times.push_back(0.4);
//        values.push_back(ValueType(keyposes[1].roll, keyposes[1].pitch, keyposes[1].yaw));
//        times.push_back(0.8);
//        values.push_back(ValueType(keyposes[2].roll, keyposes[2].pitch, keyposes[2].yaw));
//        times.push_back(1.2);
//        values.push_back(ValueType(keyposes[3].roll, keyposes[3].pitch, keyposes[3].yaw));
//
//        curveCubic.fitCurve(times,values);



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
            kalmantracker.transitionMatrix = (Mat_<float>(4,4)<<1,0,0.1,0, 0,1,0,0.1, 0,0,1,0, 0,0,0,1);
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
            if(lostTimes > 3){//erase this tracker

                if(trackedtimes > 30){//保存losted跟踪的轨迹信息
                    fstream checkfile ;
                    std::string path = "/home/cyz/Data/legoloam/poses/trackinginfo/"+std::to_string(trackerID)+".txt";
                    checkfile.open(path, ios::in);

                    if(!checkfile){

                    }else{
                        path = "/home/cyz/Data/legoloam/poses/trackinginfo/"+std::to_string(trackerID)+"_"+
                               std::to_string(random())+".txt";
                    }
                    checkfile.close();

                    int measureSize = centrosMeasurements_pc->points.size();
                    int predicSize = centrosPredicsXYVxVy_pc->points.size();

//                    cout<<"measureSize : "<<measureSize<<" ; "<<"predicSize : "<<predicSize<<endl;

                    const char* pathtxt = path.c_str();
                    FILE *fp;
                    fp = fopen(pathtxt,"w");
                    for(int i=0; i<measureSize; i++)
                        fprintf(fp,"%lf %lf %lf %lf %lf %lf\n",
                                centrosMeasurements_pc->points[i].x, centrosMeasurements_pc->points[i].y,
                                centrosPredicsXYVxVy_pc->points[i].x, centrosPredicsXYVxVy_pc->points[i].y,
                                centrosPredicsXYVxVy_pc->points[i].z, centrosPredicsXYVxVy_pc->points[i].intensity);

                    fclose(fp);
                }
                return -1;
            }

            else
            return trackedtimes;
        }


    };
}
#endif //POINTCLOUDPROCESSING_DBSCANCLUSTER_H
