//
// Created by echo on 2019/11/26.
//

#ifndef PCD_COMPARE_REGISTRATION_H
#define PCD_COMPARE_REGISTRATION_H
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
typedef PointMatcher<float> PM;
class registration {
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
public:
	//pcl::IterativeClosestPointWithNormals a;
/*	pcl::IterativeClosestPointWithNormals a;*/
	registration(){};
	void setParam(std::string configFileName = "/media/echo/DataDisc/3_program/mapping/cfg/icp.yaml");
	void setMap(pcl::PointCloud<pcl::PointXYZI> pcin);
	//pcl 配准部分
	void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
	
	void SetNormalICP();
	pcl::PointCloud<pcl::PointXYZI> normalIcpRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
									 pcl::PointCloud<pcl::PointXYZI> target) ;
	PM::TransformationParameters  setScan(pcl::PointCloud<pcl::PointXYZI> pcin);
	Eigen::Matrix4f transform_frame_to_frame = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f icp_init = Eigen::Matrix4f::Identity();//icp的初值
	Eigen::Matrix4f increase = Eigen::Matrix4f::Identity();//两次icp的结果
	
private:
	PM::DataPoints *mapPointCloud;
	PM::ICPSequence icp;
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr pcl_plane_plane_icp;
};


#endif //PCD_COMPARE_REGISTRATION_H
