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
     * 利用6D位姿将点云进行转换
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
     */
    void extractClusterEuclidean(pcXYZIptr pc, pcXYZIptr pcClusterCentos){

        pcl::ConditionAnd<pcl::PointXYZI>::Ptr condi(new pcl::ConditionAnd<pcl::PointXYZI>());
        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                     (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT, -10.0)));
        condi->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr
                                     (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT, 10.0)));
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

        for(int i=0; i<pc->points.size(); i++)
            pc->points[i].z=0;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(pc);
        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> eucliClusterExt;
        eucliClusterExt.setClusterTolerance(0.2);
        eucliClusterExt.setMinClusterSize(30);
        eucliClusterExt.setMaxClusterSize(300);
        eucliClusterExt.setSearchMethod(tree);
        eucliClusterExt.setInputCloud(pc);
        eucliClusterExt.extract(clusters);

        cout<<"Euclidean cluster size : "<<clusters.size()<<endl;
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

                if(max.x-min.x>10 || max.y-min.y>10)
                    continue;
                centro.x = centroVec[0];
                centro.y = centroVec[1];
                centro.z = centroVec[2];
                pcClusterCentos->push_back(centro);
            }


    }


    /**
     *
     */
    using namespace cv;
    bool kalmanfilterTracking2Dopencv(){
        const int stateNum=4;
        const int measureNum=2;
        KalmanFilter KF(stateNum, measureNum, 0);
        KF.transitionMatrix = (Mat_<float>(4,4)<<1,0,0.1,0, 0,1,0,0.1, 0,0,1,0, 0,0,0,1);
        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
        setIdentity(KF.errorCovPost, Scalar::all(1));
        Mat measurement = Mat::zeros(measureNum, 1, CV_32F);

        KF.statePost = (Mat_<float>(4,1)<<1,0,0,0);

    }

    /**
     * just for visualization multiple bounding boxes
     */
    pcXYZIptr pcCluster_Centros(new pcXYZI());
    void viewClusterbox(pcl::visualization::PCLVisualizer& viewer){
        viewer.removeAllShapes();

        for(int i=0; i<pcCluster_Centros->points.size(); i++)
        {

            Eigen::Vector3f centro;
            centro[0] = pcCluster_Centros->points[i].x;
            centro[1] = pcCluster_Centros->points[i].y;
            centro[2] = pcCluster_Centros->points[i].z;
            const Eigen::Quaternionf noRotation(Eigen::Quaternionf::Identity());

            viewer.addCube(centro, noRotation, 1,1,1, std::to_string(i));
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                               pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                               std::to_string(i));

        }
        viewer.spinOnce(100);
        pcCluster_Centros.reset(new pcXYZI());
    }


    /**
     * not finished yet
     */
    typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType ValueType;
//    typedef typename curves::PolynomialSplineCubicVector6Curve::ValueType ValueType;
    typedef typename curves::Time Time;
    bool polySplineCubicVec6curveFitting(std::vector<PointTypePose> keyposes){

//        curves::PolynomialSplineCubicVector6Curve curveCubic;
        curves::PolynomialSplineQuinticVector3Curve curveQuin;
        std::vector<Time> times;
        std::vector<ValueType> values;

        //位移拟合
        times.push_back(0.0);
//        values.push_back( ValueType(0.0, 0.0, 0.0) );
        values.push_back(ValueType(keyposes[0].x, keyposes[0].y, keyposes[0].z));
        times.push_back(0.4);
        values.push_back(ValueType(keyposes[1].x, keyposes[1].y, keyposes[1].z));
        times.push_back(0.8);
        values.push_back(ValueType(keyposes[2].x, keyposes[2].y, keyposes[2].z));
        times.push_back(1.2);
        values.push_back(ValueType(keyposes[3].x, keyposes[3].y, keyposes[3].z));


        curveQuin.fitCurve(times, values);
        curveQuin.clear();
        std::vector<Time>().swap(times);
        std::vector<ValueType>().swap(values);

        FILE *fp;
        fp = fopen("/home/cyz/Data/legoloam/poses/keyposesFinalsplined.txt","a");
        std::vector<PointTypePose> keyposesSplineKnots;
        ValueType value;
        keyposesSplineKnots.resize(3);
        for(double t=times[1]; t<=times[2]; t+=0.1)
        {
            curveQuin.evaluate(value,t);
            keyposesSplineKnots[0].x = value[0];
        }


        //角度拟合
        times.push_back(0.0);
        values.push_back(ValueType(keyposes[0].roll, keyposes[0].pitch, keyposes[0].yaw));
        times.push_back(0.4);
        values.push_back(ValueType(keyposes[1].roll, keyposes[1].pitch, keyposes[1].yaw));
        times.push_back(0.8);
        values.push_back(ValueType(keyposes[2].roll, keyposes[2].pitch, keyposes[2].yaw));
        times.push_back(1.2);
        values.push_back(ValueType(keyposes[3].roll, keyposes[3].pitch, keyposes[3].yaw));

        curveQuin.fitCurve(times,values);



    }


    /**
     * each tracker corresponding to one cluster
     */
    class clusterTracker{
    private:
        pcXYZIptr centrosMeasurements_pc;
        pcXYZIptr centrosPredicsXYVxVy_pc;
        int trackedtimes;
        int trackerID;
        bool isTracked;
        KalmanFilter kalmantracker;


    public:
        clusterTracker(int id, pcl::PointXYZI centro)
        {
            initialization();
            centrosMeasurements->points.push_back(centro);
            trackerID = id;

            //tracker 初始化（仅执行一次）
            kalmantracker.init(4, 2, 0);
            kalmantracker.transitionMatrix = (Mat_<float>(4,4)<<1,0,0.1,0, 0,1,0,0.1, 0,0,1,0, 0,0,0,1);
            setIdentity(kalmantracker.measurementMatrix);
            setIdentity(kalmantracker.processNoiseCov, Scalar::all(1e-5));
            setIdentity(kalmantracker.measurementNoiseCov, Scalar::all(1e-1));
            setIdentity(kalmantracker.errorCovPost, Scalar::all(1));
            kalmantracker.statePost= (Mat_<float>(4,1)<< centro.x, centro.y, 0 ,0);//初始速度为0
            cout<<"kalman tracker initialized "<<endl;

        }

        initialization() {

            isTracked = true;
            trackedtimes = 1;
            centrosMeasurements.reset( new pcXYZI());
            centrosPredics.reset( new pcXYZI());
        }
        ~clusterTracker(){};


        void addmeasurements(pcl::PointXYZI centroMeasure){

            centrosMeasurements_pc->points.push_back(centroMeasure);
            trackedtimes++;

            Mat measurement = Mat::zeros(2, 1, CV_32F);
            measurement.at<float>(0) = centroMeasure.x;
            measurement.at<float>(1) = centroMeasure.y;

            kalmantracker.correct(measurement);
            cout<<"corrected: ("<<measurement.at<float>(0)<<", "<<measurement.at<float>(1)<<") "<<endl;
        }




    };
}
#endif //POINTCLOUDPROCESSING_DBSCANCLUSTER_H
