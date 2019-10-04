//
// Created by joe on 2019/9/26.
//

#include "ringProjection.h"

void RingProjector::setInputCloud(const pcXYZIptr inCloud) {
    pcl::copyPointCloud(*inCloud, *laserCloudIn);
}

void RingProjector::findStartEndAngle() {

    startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                              laserCloudIn->points[laserCloudIn->points.size() - 2].x) + 2*M_PI;

    if (endOrientation - startOrientation > 3 * M_PI) {
        endOrientation -= 2 * M_PI;
    } else if (endOrientation - startOrientation < M_PI)
        endOrientation += 2 * M_PI;
    orientationDiff = endOrientation - startOrientation;
}

void RingProjector::projectPointCloud() {

    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    pcl::PointXYZI thisPoint;

    int longrangeCounts = 0;//用来判断室内室外

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;

        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        columnIdn = -round( (horizonAngle-90.0)/ang_res_x ) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        rangeMat.at<float>(rowIdn, columnIdn) = range;

        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

        index = columnIdn  + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;

        fullInfoCloud->points[index].intensity = range;
        fullInfoCloud->points[index].x = i;//原始点云序号

        //根据range判断室内外场景
        if(range > 60)
            longrangeCounts++;
    }

    cout<<"longe range measurements size "<<longrangeCounts<<endl;
//        if( (longrangeCounts*1.0)/cloudSize < 0.1)//百分比
    if( longrangeCounts < 20){
        cout<<"@@@   indoor  --------"<<endl;
        isOutdoor = false;
    }
    else
        cout<<"@@@   Outdoor ++++++++"<<endl;

    //cyz>> save depth images
//        string depthimgfolder = "/home/cyz/Data/legoloam/poses/depthimgs/";
//        cv::imwrite(depthimgfolder+to_string(cloudHeader.stamp.toSec())+".jpg",rangeMat);
}

void RingProjector::groundRemoval() {

    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;

    for (size_t j = 0; j < Horizon_SCAN; ++j)
        for (size_t i = 0; i < groundScanInd; ++i)//row
        {

            lowerInd = j + ( i )*Horizon_SCAN;//indice
            upperInd = j + (i+1)*Horizon_SCAN;

            if (fullCloud->points[lowerInd].intensity == -1 ||
                fullCloud->points[upperInd].intensity == -1)//which is initialized as nanPoint
            {
                groundMat.at<int8_t>(i,j) = -1;
                continue;
            }

            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= 10){
                groundMat.at<int8_t>(i,j) = 1;  //地面点标记为1
                groundMat.at<int8_t>(i+1,j) = 1;
            }
        }

    //地面点和无效range点标签-1
    //计算每一列地面点的最大range
    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j){

            if( groundMat.at<int8_t>(i,j) == 1 ){
                labelMat.at<int>(i,j) = -1;
                if(rangeMat.at<float>(i,j) > maxGroundrangeOfcol[j])
                    maxGroundrangeOfcol[j] = rangeMat.at<float>(i,j);
            }
            if (rangeMat.at<float>(i,j) == FLT_MAX)
                labelMat.at<int>(i,j) = -1;
        }

    for (size_t i = 0; i <= groundScanInd; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
            if (groundMat.at<int8_t>(i,j) == 1){
                groundCloud->push_back(fullCloud->points[j+i*Horizon_SCAN]);
                groundCloud->back().intensity = fullInfoCloud->points[j+i*Horizon_SCAN].x;
            }


}

void RingProjector::cloudSegmentation() {

    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
            if (labelMat.at<int>(i,j) == 0)//所有非地面点
                labelComponents(i, j);

    int sizeOfSegCloud = 0;
    for (size_t i = 0; i < N_SCAN; ++i) {

        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1)
            {
                if (labelMat.at<int>(i,j) == 999999)//杂乱点
                {
                    if (i > groundScanInd && j % 5 == 0){   //行号>groundScanInd且列号为5的倍数的点加入outlier
                        outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        continue;
                    }else{
                        continue;
                    }
                }

                if (groundMat.at<int8_t>(i,j) == 1){
                    if (j%5!=0 && j>5 && j<Horizon_SCAN-5)  //5的倍数序号的地面点加入segmentedCloud
                        continue;
                }

                //labelMat中有效标记点(segments)加入segmentedCloud
                segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
//                    segmentedCloud->points.back().intensity = labelMat.at<int>(i,j);//cyz

                ++sizeOfSegCloud;
            }
        }

    }


    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999) {
                segmentedCloudPure->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                segmentedCloudPure->points.back().intensity = labelMat.at<int>(i, j);

            }
            if(groundMat.at<int8_t>(i,j) != 1 && rangeMat.at<float>(i,j) < maxGroundrangeOfcol[j]){
                cloudaboveGround->points.push_back(
                        laserCloudIn->points[fullInfoCloud->points[j + i*Horizon_SCAN].x]);
            }

        }



//            ///save segmented cloud without ground   ---cyz
//            string datapath = "/home/joe/workspace/testData/segpure/";
//            pcl::io::savePCDFile(datapath+to_string(cloudHeader.stamp.toSec())+".pcd",*segmentedCloudPure);

}

void RingProjector::labelComponents(int row, int col) {

    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY;
    bool lineCountFlag[N_SCAN] = {false};

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;

    while(queueSize > 0) {
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        --queueSize;
        ++queueStartInd;
        labelMat.at<int>(fromIndX, fromIndY) = labelCount;

        //neighborIterator 上下左右四个邻域
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){

            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;

            if (thisIndX < 0 || thisIndX >= N_SCAN)//row出界
                continue;

            if (thisIndY < 0)//begin col of row
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)//end col of row
                thisIndY = 0;

            if (labelMat.at<int>(thisIndX, thisIndY) != 0)//已标记
                continue;

            d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                          rangeMat.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                          rangeMat.at<float>(thisIndX, thisIndY));

            if ((*iter).first == 0)
                alpha = segmentAlphaX;//0.2°→rad
            else
                alpha = segmentAlphaY;//2°→rad

            angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

            if (angle > segmentTheta)//同一类生长
            {

                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                lineCountFlag[thisIndX] = true;

                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }


    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
        feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum)//点数大于5且占据连续三行row以上
    {
        int lineCount = 0;
        for (size_t i = 0; i < N_SCAN; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;
        if (lineCount >= segmentValidLineNum)
            feasibleSegment = true;
    }

    if (feasibleSegment == true){
        ++labelCount;
    }else{
        for (size_t i = 0; i < allPushedIndSize; ++i)
            labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;//无效标记
    }
}

void RingProjector::publishCloud(pcXYZIptr outCloud) {

    pcl::copyPointCloud(*cloudaboveGround, *outCloud);
    cout<<"There are "<<outCloud->points.size()<<" points above ground. "<<endl;

    laserCloudIn->clear();
    groundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();
    cloudaboveGround->clear();
}

