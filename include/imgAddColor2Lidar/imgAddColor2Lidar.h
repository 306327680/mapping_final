//
// Created by echo on 2020/4/3.
//

#ifndef PCD_COMPARE_IMGADDCOLOR2LIDAR_H
#define PCD_COMPARE_IMGADDCOLOR2LIDAR_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include<opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "tools/util.h"
class imgAddColor2Lidar {
public:
	imgAddColor2Lidar(){};
	void setInternal(Eigen::Matrix3d intrinsics1,double k11,double k21, double k31,double p11,double p21);
	void readExInt(std::string path);
	pcl::PointCloud<PointXYZRGBI>  alignImg2LiDAR(cv::Mat mat,pcl::PointCloud<pcl::PointXYZI> cloudin);
	cv::Mat pcd2img(cv::Mat mat,pcl::PointCloud<pcl::PointXYZI> cloudin);
	pcl::PointCloud<pcl::PointXYZRGB> pclalignImg2LiDAR(cv::Mat mat,VLPPointCloud cloudin);
	pcl::PointCloud<pcl::PointXYZRGB> pclalignImg2LiDAR(cv::Mat mat, pcl::PointCloud<pcl::PointXYZI> cloudin);
	
private:
	Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
	Eigen::Vector3d translation;
	Eigen::Vector3d point_3d, point_t3d, point_3d_3;
	
	int publish_option;
	double k1, k2, k3, p1, p2; //distortion parameters
};


#endif //PCD_COMPARE_IMGADDCOLOR2LIDAR_H
