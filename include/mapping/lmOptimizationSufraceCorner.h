//
// Created by echo on 2019/8/8.
//
#include"cv.h"
#include"cxcore.h"
#include"highgui.h"
#include<cstdio>
#include<cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <Eigen/QR>
#include <Eigen/SVD>
#include "../sophus/so3.h"
#include "../sophus/se3.h"
#ifndef PERCENT_LMOPTIMIZATIONSUFRACECORNER_H
#define PERCENT_LMOPTIMIZATIONSUFRACECORNER_H


class lmOptimizationSufraceCorner {
public:
	lmOptimizationSufraceCorner(){};
	//构建平面约束的方程
	//输入1. 附近点云的surface 点 2. 当前雷达选中的点 3. 返回约束值
	bool surfConstraint(pcl::PointCloud<pcl::PointXYZI> surfNear,pcl::PointXYZI curPoint, Eigen::Vector4d& result);
	//输入1. 附近点云的cornenr 点 2. 当前雷达选中的点 3. 返回约束值
	bool cornerConstraint(pcl::PointCloud<pcl::PointXYZI> cornerNear,pcl::PointXYZI curPoint, Eigen::Vector4d& result);
	//对_constraint 关于_lidar_pose进行lm优化
	bool LMoptimization(int interationTimes);
	//SE3 转roll pitch yaw
	void currentPose(Eigen::Isometry3d pose);
	void eigen2RPYXYZ(Eigen::Isometry3d pose,std::vector<double>& vector);
	void RPYXYZ2eigen(std::vector<double>& vector,Eigen::Isometry3d & pose);
	std::vector<double>_lidar_pose;
private:
	cv::Mat matA0;
	cv::Mat matB0;
	cv::Mat matX0;
	cv::Mat matA1;
	cv::Mat matD1;
	cv::Mat matV1;
	cv::Mat matP;
	bool isDegenerate;
	std::vector<std::pair<Eigen::Vector4d,pcl::PointXYZI>> _constraint;//前面优化量,后面关于优化量的当前扫描点
protected:

};


#endif //PERCENT_LMOPTIMIZATIONSUFRACECORNER_H
