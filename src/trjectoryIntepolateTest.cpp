//
// Created by joe on 2019/9/23.
//
/// test for pose linear and spline interpolation
////////////////////////////////////////////////////////////////////////////////////////

#include "commontools.h"
#include "cubicfitting.h"

int main(int argc, char** argv){

    int datanum = 0;

    // reading per scan timestamp for interpolation
    FILE *fp;
    fp = fopen("/home/joe/workspace/testData/veloScans/timesOfscan.txt", "r");
    double t1 = 0;
    vector<double> timesofscan;
    while (1 == fscanf(fp, "%lf\n", &t1))
        timesofscan.push_back(t1);
    fclose(fp);
    cout << "Total " << timesofscan.size() << " scans." << endl;

    /**
     * 位姿文件读取，拟合部分
     */
    {
        pcl::PointCloud<PointTypePose>::Ptr pcRPYpose(new pcl::PointCloud<PointTypePose>());
        std::string posefile = "/home/joe/workspace/testData/keyposes6d_outdoor.txt";
//        std::string posefile = "/home/joe/workspace/testData/integratedposesQuan.txt";

        int keyposeSize = readRPYposefromfile(posefile, pcRPYpose);
//        int keyposeSize = readQuanPosefromfile(posefile, pcRPYpose);
        cout << "--->read total poses : " << keyposeSize << endl;


        /**
         *     位姿样条线拟合
         */
        pcl::PointCloud<PointTypePose>::Ptr pcSplinepose(new pcl::PointCloud<PointTypePose>());

        std::vector<PointTypePose> posesTocurveFit;
        for(int i=2; i<keyposeSize-2 ; i++) {

            std::vector<PointTypePose>().swap(posesTocurveFit) ;
            posesTocurveFit.push_back(pcRPYpose->points[i-1]);
            posesTocurveFit.push_back(pcRPYpose->points[i]);
            posesTocurveFit.push_back(pcRPYpose->points[i+1]);
            posesTocurveFit.push_back(pcRPYpose->points[i+2]);

//        cubicHermitSE3fitting(posesTocurveFit);

            // 利用曲线拟合修改原始位姿
            pcRPYpose->points[i] = cumulativeSplineSE3fitting(posesTocurveFit,pcRPYpose->points[i].time);

            for (int j = 0; j < 4; ++j)
                pcSplinepose->points.push_back(
                        cumulativeSplineSE3fitting(posesTocurveFit,pcRPYpose->points[i].time+j*0.1));
        }
        ///储存拟合后的轨迹位姿信息
        FILE *fp2;
        fp2 = fopen("/home/joe/workspace/testData/keyposesSpline.txt","w");
        int splineSize = pcSplinepose->points.size();
        for (int j = 0; j < splineSize; ++j) {

            fprintf(fp2, "%lf %f %f %f %lf %lf %lf\n", pcSplinepose->points[j].time,
                    pcSplinepose->points[j].x,
                    pcSplinepose->points[j].y,
                    pcSplinepose->points[j].z,
                    pcSplinepose->points[j].roll,
                    pcSplinepose->points[j].pitch,
                    pcSplinepose->points[j].yaw);
        }
        fclose(fp2);


        /**
         * 线性插值每一帧VLP数据位姿
         */
        pcl::PointCloud<PointTypePose>::Ptr pcIntepolatedposes(new pcl::PointCloud<PointTypePose>());
        PointTypePose tmppose;
        for (int j = 0; j < timesofscan.size(); ++j) {
            for (int i = 0; i < pcRPYpose->points.size() - 1; ++i) {

                if (timesofscan[j] == pcRPYpose->points[i].time)
                    pcIntepolatedposes->push_back(pcRPYpose->points[i]);
                if (timesofscan[j] > pcRPYpose->points[i].time && timesofscan[j] < pcRPYpose->points[i + 1].time) {
                    tmppose = linearposeInterpolateAtTimestamp(pcRPYpose->points[i],
                                                               pcRPYpose->points[i + 1],
                                                               timesofscan[j]);
                    if (tmppose.time < 0)
                        break;

                    pcIntepolatedposes->push_back(tmppose);
                }
            }
        }
        cout << "Linear intepolation of poses done !" << endl;

        ///储存插值后的位姿（per scan）
        FILE *fp1;
        fp1 = fopen("/home/joe/workspace/testData/keyposesScanlinear.txt", "w");
        for (int j = 0; j < pcIntepolatedposes->points.size(); ++j) {

            fprintf(fp1, "%lf %f %f %f %lf %lf %lf\n", pcIntepolatedposes->points[j].time,
                    pcIntepolatedposes->points[j].x,
                    pcIntepolatedposes->points[j].y,
                    pcIntepolatedposes->points[j].z,
                    pcIntepolatedposes->points[j].roll,
                    pcIntepolatedposes->points[j].pitch,
                    pcIntepolatedposes->points[j].yaw);
        }
        fclose(fp1);
        cout << "--and saved !" << endl;


//    string dataFolder = "/home/cyz/Data/legoloam/poses/keyframes/";
        string dataFolder = "/home/joe/workspace/testData/veloScans/";
        getAndsaveglobalmapRPY(dataFolder, pcRPYpose);

        return 0;
    }
/**
 * 位姿部分结束
 */

}