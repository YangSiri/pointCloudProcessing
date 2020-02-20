//
// Created by cyz on 19-6-19.
//

#ifndef POINTCLOUDPROCESSING_CUBICFITTING_H
#define POINTCLOUDPROCESSING_CUBICFITTING_H

#include<iostream>
#include<fstream>
#include <sstream>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/io/pcd_io.h>

//#include "dbscanCLuster.h"
#include "commontools.h"
//#include "curves/CubicHermiteSE3Curve.hpp"

//#include <gtest/gtest.h>
//#include <kindr/Core>
//#include <kindr/common/gtest_eigen.hpp>
//#include <limits>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
//#include <sophus/se3.h>
//#include <sophus/so3.h>

using namespace std;


/**
 * from ETH Zurich @github/curves
 */
//using namespace curves;
//typedef std::numeric_limits<double > dbl;
//typedef typename curves::CubicHermiteSE3Curve::ValueType ValueType;
//typedef typename curves::CubicHermiteSE3Curve::DerivativeType DerivativeType;
//typedef typename curves::Time Time;
//bool cubicHermitSE3fitting(vector<PointTypePose>&poses){
//
//    CubicHermiteSE3Curve curve;
//    std::vector<Time> times;
//    std::vector<ValueType> values;
//
//    for (int i = 0; i < poses.size(); ++i) {
//
//        times.push_back(poses[i].time);
//        values.push_back(ValueType(ValueType::Position(poses[i].x, poses[i].y, poses[i].z),
//                                   ValueType::Rotation(kindr::EulerAnglesRpyD(poses[i].roll,
//                                                                              poses[i].pitch,
//                                                                              poses[i].yaw))));
//    }
//    curve.fitCurve(times, values);
//
//
//    //不改变第一个与最后一个时间点的pose
//    for (int j = 1; j < poses.size()-1; ++j) {
//
//        ValueType value;
////        ASSERT_TRUE(curve.evaluate(value, times[j]));
//        curve.evaluate(value, times[j]);
//
//        //translation intepolated
//        double x = value.getPosition().x();
//        double y = value.getPosition().y();
//        double z = value.getPosition().z();
//
//        //四元数->欧拉角
//        Eigen::Quaterniond q(value.getRotation().w(),
//                             value.getRotation().x(),
//                             value.getRotation().y(),
//                             value.getRotation().z());
//        Eigen::Vector3d eular = q.matrix().eulerAngles(2,1,0);
////        Eigen::Vector3d eular = q.matrix().eulerAngles(0,1,2);
//        cout<<"Eular angles :"<<eular<<endl;
//
//        poses[j].x = x;
//        poses[j].y = y;
//        poses[j].z = z;
//        poses[j].roll = eular(2);
//        poses[j].pitch = eular(1);
//        poses[j].yaw = eular(0);
//
////        fprintf(fp, "%lf %f %f %f %lf %lf %lf\n", times[j], x, y, z, eular(2), eular(1), eular(0));
//    }
//
//    curve.clear();
//
//}
#define PI 3.1415
inline float judgeRad(double rad){
    if (abs(rad-3) < 0.5)
        if(rad > PI)
            return -(rad-PI); // no
        else
            return -(PI-rad); // yaw
    if (abs(rad+3) < 0.5)
        if(rad < -PI)
            return -(rad+PI); // no
        else
            return (rad+PI); // roll

    return rad*1.0;
}
// TODO :why pitch angle is different from roll/yaw ?
inline float judgeRad2(double rad){
    if (abs(rad-3) < 0.5)
        if(rad > PI)
            return -(rad-PI); // no
        else
            return -(PI-rad); // yaw
    if (abs(rad+3) < 0.5)
        if(rad < -PI)
            return (rad+PI); // no
        else
            return -(rad+PI); // roll

    return rad*1.0;
}

/**
 * 线性插值位姿
 * @param startpose
 * @param endpose
 * @param num 插值的位姿点个数
 * @return intepolated poses
 */
bool poselinearInterpolation(PointTypePose startpose, PointTypePose endpose, int num){

    double delta_t = endpose.time - startpose.time;
    double dt = delta_t / (num+1) ;

    double deltatx = endpose.x - startpose.x;
    double deltaty = endpose.y - startpose.y;
    double deltatz = endpose.z - startpose.z;
    Eigen::Vector3d deltatranslation(deltatx,deltaty,deltatz);

    Eigen::Matrix3d startrotation=(
            Eigen::AngleAxisd(startpose.roll, Eigen::Vector3d::UnitX())*
            Eigen::AngleAxisd(startpose.pitch, Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(startpose.yaw, Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

    Eigen::Matrix3d endrotation=(
            Eigen::AngleAxisd(startpose.roll, Eigen::Vector3d::UnitX())*
            Eigen::AngleAxisd(startpose.pitch, Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(startpose.yaw, Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

    Eigen::AngleAxisd deltaR  (startrotation.inverse() * endrotation);

    vector<PointTypePose> posesNew_intepolated;//without start and end pose
    for (int i = 1 ; i <= num; ++i) {

        double increT = dt * i;
        PointTypePose tmp_pose;
        tmp_pose.x = startpose.x + deltatx*(increT/delta_t);
        tmp_pose.y = startpose.y + deltaty*(increT/delta_t);
        tmp_pose.z = startpose.z + deltatz*(increT/delta_t);

        Eigen::AngleAxisd increR(deltaR.angle()*(increT/delta_t), deltaR.axis());
        Eigen::Matrix3d Rotation = startrotation * increR;
        Eigen::Vector3d eulars = Rotation.eulerAngles(2,1,0);

        cout<<"Eular Angles :"<<eulars<<endl;
        tmp_pose.roll = eulars(2);
        tmp_pose.pitch = eulars(1);
        tmp_pose.yaw = eulars(0);

        tmp_pose.time = startpose.time + increT;
        posesNew_intepolated.push_back(tmp_pose);
    }
    return true;
}

/// 线性插值两帧之间的6D位姿
/// \param startpose：[tx,ty,tz,roll,pitch,yaw]
/// \param endpose
/// \param timestamp
/// \return pose at timestamp
PointTypePose linearposeInterpolateAtTimestamp(PointTypePose startpose, PointTypePose endpose, double timestamp){

    double delta_t = endpose.time - startpose.time;
    double increT = timestamp - startpose.time;

    double deltatx = endpose.x - startpose.x;
    double deltaty = endpose.y - startpose.y;
    double deltatz = endpose.z - startpose.z;
    Eigen::Vector3d deltatranslation(deltatx,deltaty,deltatz);

    Eigen::Matrix3d startrotation=(
            Eigen::AngleAxisd(startpose.yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(startpose.pitch, Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(startpose.roll, Eigen::Vector3d::UnitX())
    ).toRotationMatrix();

    Eigen::Matrix3d endrotation=(
            Eigen::AngleAxisd(endpose.yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(endpose.pitch, Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(endpose.roll, Eigen::Vector3d::UnitX())
    ).toRotationMatrix();
    Eigen::AngleAxisd deltaR  (startrotation.inverse() * endrotation);


    PointTypePose tmp_pose;
    tmp_pose.x = startpose.x + deltatx*(increT/delta_t);
    tmp_pose.y = startpose.y + deltaty*(increT/delta_t);
    tmp_pose.z = startpose.z + deltatz*(increT/delta_t);

    Eigen::AngleAxisd increR(deltaR.angle()*(increT/delta_t), deltaR.axis());
    Eigen::Matrix3d Rotation = startrotation * increR;
    Eigen::Vector3d eulars = Rotation.eulerAngles(2,1,0);

    cout<<"Eular Angles :"<<eulars<<endl;
    tmp_pose.roll = judgeRad(eulars(2));
    tmp_pose.pitch = judgeRad2(eulars(1));
    tmp_pose.yaw = judgeRad(eulars(0));

    // in case of PI
//    if(abs(tmp_pose.roll - startpose.roll) >1 )
//        if(tmp_pose.roll < 0)
//            tmp_pose.roll = - PI - tmp_pose.roll;
//        else
//            tmp_pose.roll = PI - tmp_pose.roll;
//
//    if(abs(tmp_pose.pitch - startpose.pitch) >1 )
//        if(tmp_pose.pitch < 0)
//            tmp_pose.pitch = - PI - tmp_pose.pitch;
//        else
//            tmp_pose.pitch = PI - tmp_pose.pitch;
//
//    if(abs(tmp_pose.yaw - startpose.yaw) >1 )
//        if(tmp_pose.yaw < 0)
//            tmp_pose.yaw = - PI - tmp_pose.yaw;
//        else
//            tmp_pose.yaw = PI - tmp_pose.yaw;

//    if(tmp_pose.roll > startpose.roll ||
//       tmp_pose.pitch > startpose.pitch ||
//       tmp_pose.yaw > startpose.yaw)
//        tmp_pose.time = -1;
//    else
    tmp_pose.time = timestamp;
    return tmp_pose;
}


/**
 * 利用控制点拟合B-Spline
 * @param deltaT
 * @return
 */
//求解cumulative basis参数 -- B~(u)
Eigen::Vector4d splineBaseCumulativeMatrixBu(double deltaT){

    Eigen::Vector4d paras(1,0,0,0);
    paras(1) = (deltaT*deltaT*deltaT - 3*deltaT*deltaT + 3*deltaT +5)/6.0;
    paras(2) = (-2*deltaT*deltaT*deltaT + 3*deltaT*deltaT + 3*deltaT +1)/6.0;
    paras(3) = (deltaT*deltaT*deltaT)/6.0 ;

    return paras;
}
// TODO 反求B-Spline控制点
bool originalposes2controlposes(vector<Sophus::SE3<double>>poses, vector<Sophus::SE3<double>> &controlposes){

    controlposes.swap(poses);

    Sophus::SE3<double> tmpT2 = poses[0]*Sophus::SE3<double>::exp(4*poses[1].log());


}
// 利用四个位姿控制点进行Spline拟合
// 获得时刻t的位姿，t介于第二个位姿与第三个位姿之间
typedef Eigen::Matrix< double, 6, 1 > Vector6d;
PointTypePose cumulativeSplineSE3fitting(vector<PointTypePose> &poses, double t){

    vector<double> times;
    vector<Sophus::SE3<double> > controlposes;
    for (int i = 0; i < poses.size(); ++i) {

        times.push_back(poses[i].time);
//        Eigen::Vector3d eular(poses[i].yaw, poses[i].pitch, poses[i].roll);
        Eigen::Matrix3d R=(
                Eigen::AngleAxisd(poses[i].yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(poses[i].pitch, Eigen::Vector3d::UnitY())*
                Eigen::AngleAxisd(poses[i].roll, Eigen::Vector3d::UnitX())
        ).toRotationMatrix();
        Eigen::Vector3d t (poses[i].x, poses[i].y, poses[i].z);

        Sophus::SE3<double> T(R, t);
//        cout<<"T: "<<T<<endl;
        controlposes.push_back(T);
    }

    PointTypePose poseOut;
    double dt = (t-times[1]) / 0.4;
//        double dt = times[j]-times[0]+0.1;
    Eigen::Vector4d parasBu = splineBaseCumulativeMatrixBu(dt);

    Vector6d update1se3 = parasBu(1) * ( controlposes[0].inverse() * controlposes[1] ).log();
    Vector6d update2se3 = parasBu(2) * ( controlposes[1].inverse() * controlposes[2] ).log();
    Vector6d update3se3 = parasBu(3) * ( controlposes[2].inverse() * controlposes[3] ).log();

    Vector6d newposeSE31 = parasBu(0) * controlposes[0].log() ;
    Sophus::SE3<double> newposeSE3 = Sophus::SE3<double>::exp(newposeSE31) *
                                     Sophus::SE3<double>::exp(update1se3) *
                                     Sophus::SE3<double>::exp(update2se3) *
                                     Sophus::SE3<double>::exp(update3se3);

//        cout<<"#intepolated T :"<<newposeSE3<<endl;
    Eigen::Vector3d eularAgl= newposeSE3.rotationMatrix().eulerAngles(2,1,0);
    Eigen::Vector3d translations = newposeSE3.translation();
//        cout<<"Intepolated Eular Angles :"<<eularAgl<<endl;
//        cout<<"Intepolated Translations :"<<translations<<endl;

    poseOut.x = translations(0);
    poseOut.y = translations(1);
    poseOut.z = translations(2);
    poseOut.roll = judgeRad(eularAgl(2));
    poseOut.pitch = judgeRad2(eularAgl(1));
    poseOut.yaw = judgeRad(eularAgl(0));

    poseOut.time = t;

    return poseOut;
}



/*
 * 三次样条曲线 --欧式空间
 * 插值过控制点，拟合不过控制点
 * (插值需要反求控制点)
 */
bool dataregularization(vector<PointTypePose> posesVec){

    int datasize = posesVec.size();

    double *time = new double[datasize];
    double *tx = new double[datasize];
    double *ty = new double[datasize];
    double *tz = new double[datasize];
    double *roll = new double[datasize];
    double *pitch = new double[datasize];
    double *yaw = new double[datasize];

    for (int i = 0; i < datasize; ++i) {
        time[i] = posesVec[i].time;

        tx[i] = posesVec[i].x;
        ty[i] = posesVec[i].y;
        tz[i] = posesVec[i].z;

        roll[i] = posesVec[i].roll;
        pitch[i] = posesVec[i].pitch;
        yaw[i] = posesVec[i].yaw;

    }



    delete [] time,tx,ty,tz,roll,pitch,yaw;
}
//求得控制点坐标
void LU(int n,double *a,double *b,double *c,double d,double e,double *pointx,double *pointy) {

    double *f=new double[n+1];
    double *g=new double[n+1];
    double *h=new double[n+1];
    double *k=new double[n+1];
    //求解得到h[],f[]
    h[1]=b[1];
    for(int i=1;i<n-1;i++)
    {
        f[i]=a[i]/h[i];
        h[i+1]=b[i+1]-f[i]*c[i];
    }
    //求解得到g[]和f[n-1]
    g[1]=d/h[1];
    for(int i=1;i<n-2;i++)
    {
        g[i+1]=-g[i]*c[i]/h[i+1];
    }
    f[n-1]=(a[n-1]-g[n-2]*c[n-2])/h[n-1];
    //求解得到k[]和h[n]
    k[1]=e;
    for(int i=1;i<n-2;i++)
    {
        k[i+1]=-f[i]*k[i];
    }
    k[n-1]=c[n-1]-f[n-2]*k[n-2];
    double gk_sum=0;
    for(int i=1;i<n-1;i++)
    {
        gk_sum=gk_sum+g[i]*k[i];
    }
    h[n]=b[n]-gk_sum-f[n-1]*c[n-1];
    //矩阵求解过程
    //追的过程
    double *y=new double[n+1];
    double *x=new double[n+1];
    x[0]=6*pointx[n-1];
    y[0]=6*pointy[n-1];
    for(int i=0;i<n-2;i++)
    {
        x[i+1]=6*pointx[i]-f[i+1]*x[i];
        y[i+1]=6*pointy[i]-f[i+1]*y[i];
    }
    double gx_sum=0,gy_sum=0;
    for(int i=0;i<n-2;i++)
    {
        gx_sum=gx_sum+g[i+1]*x[i];
        gy_sum=gy_sum+g[i+1]*y[i];
    }
    x[n-1]=6*pointx[n-2]-gx_sum-f[n-1]*x[n-2];
    y[n-1]=6*pointy[n-2]-gy_sum-f[n-1]*y[n-2];
    //赶的过程
    double *px=new double[n+2];
    double *py=new double[n+2];
    px[n-1]=x[n-1]/h[n];
    px[n-2]=(x[n-2]-k[n-1]*px[n-1])/h[n-1];
    py[n-1]=y[n-1]/h[n];
    py[n-2]=(y[n-2]-k[n-1]*py[n-1])/h[n-1];
    for(int i=n-3;i>=0;i--)
    {
        px[i]=(x[i]-c[i+1]*px[i+1]-k[i+1]*px[n-1])/h[i+1];
        py[i]=(y[i]-c[i+1]*py[i+1]-k[i+1]*py[n-1])/h[i+1];
    }
    px[n]=px[0];px[n+1]=px[1];
    py[n]=py[0];py[n+1]=py[1];
    //将排序后的端点写入到文件中
    ofstream ftxt("/home/cyz/workspace/me.txt");
    if(!ftxt)
    {
        cout<<"创建文件失败！请检查文件写路径是否正确！"<<endl;
        return;
    }
    //ftxt.open("e:\\me.txtme.txt",ios::trunc);//打开文件之前清空文件
    ftxt<<n<<endl;
    int count=0;
    for(int i=0;i<n;i++)
    {
        //cout<<vecCorners[i]<<endl;
        ftxt<<px[i]<<","<<py[i]<<endl;
        count++;
    }
    for(int i=0;i<n;i++)
    {
        //cout<<vecCorners[i]<<endl;
        ftxt<<pointx[i]<<","<<pointy[i]<<endl;
    }
    ftxt.close();
    cout<<"写入文件成功！共写入："<<count<<"个特征点"<<endl;
    delete [] f,k,g,h,x,y,px,py;
}
void Equalication(double* pointx, double* pointy,int inum=4) {

    //初始化
    double *a=new double[inum];
    double *b=new double[inum];
    double *c=new double[inum];
    int d=1,e=1;
    for(int i=1;i<=inum;i++)
    {
        b[i]=4;
        if((i+1)<=inum)
        {
            a[i]=1;c[i]=1;
        }
    }
    LU(inum,a,b,c,d,e,pointx,pointy);
}



#endif //POINTCLOUDPROCESSING_CUBICFITTING_H
