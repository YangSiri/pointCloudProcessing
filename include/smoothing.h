//
// Created by cyz on 18-9-23.
//

#ifndef POINTCLOUDPROCESSING_SMOOTHING_H
#define POINTCLOUDPROCESSING_SMOOTHING_H

#include <string>
#include <vector>

#include <pcl/surface/mls.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <boost/filesystem.hpp>

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr  pcXYZIptr;
typedef pcl::PointCloud<pcl::PointXYZI>  pcXYZI;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr  pcXYZptr;
typedef pcl::PointCloud<pcl::PointXYZ>  pcXYZ;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcRGBptr;
typedef pcl::PointCloud<pcl::PointXYZRGB> pcRGB;

class preprocess
{
public:
    bool txt2pc(std::string txtpath, pcXYZI &origiCloud);

    bool xyz2pc(std::string txtpath, pcXYZI &origiCloud);

    bool readpcfileFromFolder(std::string folder,std::vector<pcXYZI> &pcScans);

    ///类模板只能定义在*.h(*.hpp)文件中
    template <typename ptType>
    bool visualize(const typename pcl::PointCloud <ptType>::ConstPtr pointCloud, int id, std::string name)
    {
//        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("pc viewer"));
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(name));

//    viewer->setBackgroundColor(0,0,0);
        viewer->addPointCloud<ptType>(pointCloud,std::to_string(id));
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        while(!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        return true;
    }

    template <typename ptType, typename ptNorType>
    bool visualize_withNormals(typename pcl::PointCloud <ptType>::ConstPtr pointCloud, int id,
                               typename pcl::PointCloud<ptNorType>::ConstPtr normals )
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pc viewer"));

//    viewer->setBackgroundColor(0,0,0);
        viewer->addPointCloud<ptType>(pointCloud,std::to_string(id));
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        viewer->addPointCloudNormals<ptType, ptNorType>(pointCloud,normals,500,6);

        while(!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        return true;
    };


    bool statisticalOutlierRemoval(pcXYZIptr cloud, pcXYZIptr cloud_new );

    bool downSample(pcXYZIptr cloud, pcXYZIptr cloud_new);

    bool planeSeg(pcXYZIptr cloud, std::vector<pcXYZI> &cloud_plane);

    bool normalestimate(pcXYZIptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    bool regionGrow_flat(pcXYZIptr cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &clustersCloud);
};

#endif //POINTCLOUDPROCESSING_SMOOTHING_H
