//
// Created by echo on 2019/11/26.
//

#ifndef PCD_COMPARE_REGISTRATION_H
#define PCD_COMPARE_REGISTRATION_H
/*#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"*/
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d_omp.h>
#include "tools/util.h"
#include <pcl/surface/mls.h>

/*typedef PointMatcher<float> PM;*/
class registration {
/*	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;*/
public:
	//pcl::IterativeClosestPointWithNormals a;
/*	pcl::IterativeClosestPointWithNormals a;*/
	registration(){reset();};//初始化ptr用的
	void reset(){pcl_plane_plane_icp.reset(new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>);}
	void setParam(std::string configFileName = "/media/echo/DataDisc/3_program/mapping/cfg/icp.yaml"){};
	void setMap(pcl::PointCloud<pcl::PointXYZI> pcin);
	//pcl 配准部分
	void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
	void addNormalRadius(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
	//scan-scan 参数
	void SetNormalICP();
	void SetNormalICP(int Correspondence);
	//scan-map参数
	void SetPlaneICP();
	void SetPlaneICP(int Correspondence);
	pcl::PointCloud<pcl::PointXYZI> normalIcpRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
									 pcl::PointCloud<pcl::PointXYZI> target);
	pcl::PointCloud<pcl::PointXYZI> normalIcpRegistrationlocal(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
														  pcl::PointCloud<pcl::PointXYZI> target);
	pcl::PointCloud<pcl::PointXYZI>  IcpWithConvriance(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
															   pcl::PointCloud<pcl::PointXYZI> target, Eigen::MatrixXd &ICP_COV);
	//PM::TransformationParameters  setScan(pcl::PointCloud<pcl::PointXYZI> pcin);
	Eigen::Matrix4f transform_frame_to_frame = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(); //全局位姿
	Eigen::Matrix4f icp_init = Eigen::Matrix4f::Identity();//icp的初值
	Eigen::Matrix4f increase = Eigen::Matrix4f::Identity();//两次icp的结果
	//tools ReOrthogonalization 防止累计误差
	Eigen::Isometry3d  ReOrthogonalization(Eigen::Isometry3d input);
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr pcl_plane_plane_icp;
	//可视化normal
	pcl::PointCloud<pcl::PointXYZINormal> local_map_with_normal;
private:
/*	PM::DataPoints *mapPointCloud;
	PM::ICPSequence icp;*/
	
};


#endif //PCD_COMPARE_REGISTRATION_H
