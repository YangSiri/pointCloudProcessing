//
// Created by joe on 2020/1/13.
//
// test for jiangteping

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>

#include "commontools.h"

//typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZRGB PointTypeIO;
//typedef pcl::PointXYZINormal PointTypeFull;
typedef pcl::PointXYZRGBNormal PointTypeFull;


//bool enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b,
//                                 float squared_distance)
//{
//    if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
//        return (true);
//    else
//        return (false);
//}
//
//bool enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b,
//                                            float squared_distance)
//{
//    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
//    if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
//        return (true);
//    if (std::abs (point_a_normal.dot (point_b_normal)) < 0.05)
//        return (true);
//    return (false);
//}
//
//bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b,
//                          float squared_distance)
//{
//    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
//    if (squared_distance < 10000)
//    {
//        if (std::abs (point_a.intensity - point_b.intensity) < 8.0f)
//            return (true);
//        if (std::abs (point_a_normal.dot (point_b_normal)) < 0.06)
//            return (true);
//    }
//    else
//    {
//        if (std::abs (point_a.intensity - point_b.intensity) < 3.0f)
//            return (true);
//    }
//    return (false);
//}


int main (int argc, char** argv){

    // Data containers used
    pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr cloud_temp (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_outo (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);
    pcl::console::TicToc tt;

    // Load the input point cloud
    std::cerr << "Loading...\n", tt.tic ();
    readpcXYZRGBfromtxt("/home/joe/workspace/jiang/2nongroundclean.txt", cloud_in);
    std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";
    search_tree->setInputCloud(cloud_in);

    // Downsample the cloud using a Voxel Grid class
    std::cerr << "Downsampling...\n", tt.tic ();
    pcl::VoxelGrid<PointTypeIO> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (0.3, 0.3, 0.3);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud_temp);
    std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_temp->points.size () << " points\n";

    // Set up a Normal Estimation class and merge data in cloud_with_normals
//    std::cerr << "Computing normals...\n", tt.tic ();
//    pcl::copyPointCloud (*cloud_temp, *cloud_with_normals);
//    pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
//    ne.setInputCloud (cloud_temp);
//    ne.setSearchMethod (search_tree);
//    ne.setRadiusSearch (300.0);
//    ne.compute (*cloud_with_normals);
//    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    // Set up a Conditional Euclidean Clustering class
    std::cerr << "Segmenting to clusters...\n", tt.tic ();
//    pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
    pcl::EuclideanClusterExtraction<PointTypeIO> cec ;

    cec.setInputCloud (cloud_temp);
//    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (0.7);
    cec.setSearchMethod(search_tree);
    cec.setMinClusterSize (cloud_temp->points.size () / 10000);
    cec.setMaxClusterSize (cloud_temp->points.size () / 3);
//    cec.segment (*clusters);
    cec.extract (*clusters);
//    cec.getRemovedClusters (small_clusters, large_clusters);
    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    // Using the intensity channel for lazy visualization of the output
//    for (int i = 0; i < small_clusters->size (); ++i)
//        for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
//            cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
//    for (int i = 0; i < large_clusters->size (); ++i)
//        for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
//            cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
    cloud_out->width = cloud_temp->points.size();
    cloud_out->height = 1;
    cloud_out->points.resize(cloud_temp->points.size());
    for (int i = 0; i < clusters->size (); ++i){

        int label = rand () % 8;
        for (int j = 0; j < (*clusters)[i].indices.size (); ++j){

            cloud_out->points[(*clusters)[i].indices[j]].x = cloud_temp->points[(*clusters)[i].indices[j]].x;
            cloud_out->points[(*clusters)[i].indices[j]].y = cloud_temp->points[(*clusters)[i].indices[j]].y;
            cloud_out->points[(*clusters)[i].indices[j]].z = cloud_temp->points[(*clusters)[i].indices[j]].z;
            cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
        }
    }

    cloud_outo->clear();
    for (int k = 0; k < cloud_out->points.size(); ++k) {
        if(cloud_out->points[k].getVector3fMap().norm()< 100)
            continue;
        cloud_outo->points.push_back(cloud_out->points[k]);
    }
    // Save the output point cloud
    std::cerr << "Saving...\n", tt.tic ();

    pcl::io::savePCDFileBinary ("/home/joe/workspace/jiang/output.pcd", *cloud_out);
    pcl::io::savePCDFileBinary ("/home/joe/workspace/jiang/output.pcd", *cloud_outo);
    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    return (0);
}