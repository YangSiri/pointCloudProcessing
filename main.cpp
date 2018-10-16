#include <iostream>
#include <vector>

#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "smoothing.h"
#include "pcaAnalysis.h"
#include "buildingFacadeExtraction.h"

using namespace pcl;

int main()
{

    PointCloud<PointXYZI>::Ptr pc(new PointCloud<PointXYZI>()) ;

<<<<<<< HEAD
    std::string filepath="/home/cyz/pointCloudProcessing/data/playground_building.pcd";
    preprocess preprocessor;
//    preprocessor.txt2pc(filepath,*pc);
    pcl::PCDReader pcdReader;
    pcdReader.read(filepath, *pc);
=======
    std::string filepath="/home/cyz/Data/prj_park/laserpts.txt";
    preprocess preprocessor;
    preprocessor.txt2pc(filepath,*pc);
//    pcl::PCDReader pcdReader;
//    pcdReader.read(filepath, *pc);
>>>>>>> master
//    if(pcl::io::loadPCDFile<pcl::PointXYZI>
//            ("/home/cyz/CLionProjects/PointCloudProcessing/bin/pointCloud_filtered.pcd",*pc) == -1)
//        return 0;
    cout<<"Read point cloud file successfully. "<<endl;
//    preprocess1.visualize(pc,1);

    PointCloud<PointXYZI>::Ptr pc_filtered(new PointCloud<PointXYZI>()) ;
//    PointCloud<PointXYZI>::Ptr pc_filtered_downSample(new PointCloud<PointXYZI>()) ;
    PointCloud<PointXYZI>::Ptr pc_filtered_plane(new PointCloud<PointXYZI>()) ;

    preprocessor.statisticalOutlierRemoval(pc,pc_filtered);///statistical outlier remove
    std::cout<<"statistical outlier removal Done"<<endl;
<<<<<<< HEAD
    preprocessor.visualize<pcl::PointXYZI>(pc_filtered,1);

    buildingFacadeExtractor bFE;
    pcXYZIptr nonGroundCloud(new pcXYZI());
    bFE.groundFilter(pc_filtered, nonGroundCloud);
    std::cout<<"Ground filtered. "<<endl;
    preprocessor.visualize<pcl::PointXYZI>(nonGroundCloud,1);
//    preprocessor.visualize<pcl::PointXYZI>(pc_filtered,2);

    bFE.meanShiftClustering(nonGroundCloud,3);
    std::cout<<"mean shift done. "<<endl;
=======



    pcXYZIptr nonGroundCloud(new pcXYZI());
    buildingFacadeExtractor bFE;
    bFE.groundFilter(pc_filtered, nonGroundCloud);
    preprocessor.visualize<pcl::PointXYZI>(nonGroundCloud,1);
//    preprocessor.visualize<pcl::PointXYZI>(pc_filtered,2);
    std::cout<<"Ground filtered. "<<endl;

    bFE.meanShiftClustering(nonGroundCloud,1);
>>>>>>> master
    preprocessor.visualize<pcl::PointXYZI>(nonGroundCloud,2);

    pcXYZIptr projectedCloud(new pcXYZI());
    pcl::ModelCoefficients::Ptr planeParas(new pcl::ModelCoefficients());
    planeParas->values.resize(4);
    planeParas->values[0] = 0;
    planeParas->values[1] = 0;
    planeParas->values[2] = 1.0;
    planeParas->values[3] = 0;
    bFE.planeProjection(nonGroundCloud, planeParas, projectedCloud);
    preprocessor.visualize<pcl::PointXYZI>(projectedCloud, 3);
    std::cout<<"Cloud projected. "<<endl;

//    preprocess1.downSample(pc_filtered, pc_filtered_downSample);///voxel grid downsample

//    std::vector<pcXYZI> pc_planes;
//    if( preprocessor.planeSeg(pc_filtered, pc_planes) )///extract planes(from large to small)
//    {
//        for(size_t i=0 ; i<pc_planes.size() ; i++)
//        {
//            *pc_filtered_plane = pc_planes[i];
//            preprocessor.visualize<pcl::PointXYZI>(pc_filtered_plane, i);
//            cout << "plane size : " << pc_filtered_plane->points.size() << endl;
//        }
//
//    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr bigCurv(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr pcNormals (new pcl::PointCloud<pcl::Normal>);
    preprocessor.normalestimate(pc_filtered, pcNormals);///normal estimation
    for(int i=0 ; i<pcNormals->points.size() ; i++)
    {
        if(pcNormals->points[i].curvature * 100 > 25)
        {
            pcl::PointXYZRGB ptRGB ;
            ptRGB.x = pc_filtered->points[i].x;
            ptRGB.y = pc_filtered->points[i].y;
            ptRGB.z = pc_filtered->points[i].z;

            ptRGB.r = 255;
            ptRGB.g = 0;
            ptRGB.b = 0;

            bigCurv->push_back(ptRGB);
//            cout<<"curvature : "<<pcNormals->points[i].curvature * 100<<endl;

        }

    }

    ///PCA
//    pcl::PointCloud<pcl::PointXYZI>::Ptr keyCloud (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::copyPointCloud(*bigCurv, *keyCloud);
//
//    pcaAnalysist pcaAnalysistr;
//    std::vector<pcaAnalysist::pcaFeature> pcaFeasOfkeyCloud;
//    pcaAnalysistr.calculatePCAofPointCloud(keyCloud, 3, pcaFeasOfkeyCloud);


//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::copyPointCloud(*pc_filtered, *pc_rgb);
//    preprocessor.visualize<pcl::PointXYZRGB>(pc_rgb, 2);
//
//    *bigCurv = *bigCurv + *pc_rgb ;
    preprocessor.visualize<pcl::PointXYZRGB>(bigCurv, 5);

    ///region growing based on curvature
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustersCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    preprocessor.regionGrow_flat(pc_filtered, pcNormals, clustersCloud);
//    preprocessor.visualize<pcl::PointXYZRGB>(clustersCloud, 2);

//    preprocess1.visualize<PointXYZI>(pc_filtered, 3);
//    preprocess1.visualize_withNormals(pc_filtered,2,pcNormals);

//    cout<<pc_filtered->points.size()<<endl;
//    pcl::PCDWriter pcdWriter;
//    pcdWriter.write("pcAboveGround_electric.pcd",*pc_filtered_downSample);
//    cout<<pc_filtered_downSample->points.size()<<endl;

    return 0;
}