//
// Created by echo on 19-7-22.
//

#ifndef POINT_LOCALIZATION_EXPERIMENT_H
#define POINT_LOCALIZATION_EXPERIMENT_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>

#include "sophus/so3.h"
#include "sophus/se3.h"

class experiment {
public:
	experiment(){
		_last_Odom_pose.setIdentity();
		_last_Lidar_pose.setIdentity();
	};
	//1.变量
	//2.函数
	double calculateTransDistLidar(Eigen::Isometry3d cur);//1.计算雷达的运行距离
	double calculateTransDistOdom(Eigen::Isometry3d cur);
	bool lidarStatusCheck(pcl::PointCloud<pcl::PointXYZ> input);
	double getOdomErrorRate();
	Eigen::Isometry3d ReOrthogonalization(Eigen::Isometry3d input);
	Eigen::Isometry3d ReOrthogonalizationQR(Eigen::Isometry3d input);
private:
	//1.变量
	double max_rate;
	double _lidarDistance = 0;
	double _odomDistance = 0;
	int begin_count = 0;
	Eigen::Isometry3d _last_Lidar_pose;
	Eigen::Isometry3d _last_Odom_pose;
protected:
};

class Prediction {
public:
	Prediction(){};
	void rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion);
	void quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R);
	void trinterp(Eigen::Matrix4d &T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T); //trinterp 插值
	void qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r, Eigen::Vector4d &q_quaternion_interpolation);
	Eigen::Isometry3d motion_b_spline(double rate,Eigen::Isometry3d trans); //计算在当前情况下的增量和预测值
private:
protected:
};

class CloudPretreatment{
public:
	CloudPretreatment(){};
	

};
class SplineFusion{
public:
	SplineFusion() = default;
	//公式(23)的实现
	Eigen::Isometry3d cumulativeForm(Eigen::Isometry3d T_1,Eigen::Isometry3d T_2,
									 Eigen::Isometry3d T_3,Eigen::Isometry3d T_4, double u);
	double getUt(double t, double ti, double dt){return (t-ti)/dt;};
	Sophus::SE3 fromAtoB(Sophus::SE3 a,Sophus::SE3 b);
	void SE3Eigen2Sophus(Eigen::Isometry3d e,Sophus::SE3 & s);
	void test();
private:
	Sophus::SE3 t1,t2,t3,t4;
	pcl::PointCloud<pcl::PointXYZ> after;
protected:

};

#endif //POINT_LOCALIZATION_EXPERIMENT_H
