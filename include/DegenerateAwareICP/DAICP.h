//
// Created by echo on 2020/9/5.
//

#ifndef PCD_COMPARE_DAICP_H
#define PCD_COMPARE_DAICP_H
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <chrono>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
class DAICP {
public:
	DAICP(){};
	//svd 求解
	//1
	Eigen::Isometry3d solveICP(pcl::PointCloud<pcl::PointXYZI>::Ptr target, pcl::PointCloud<pcl::PointXYZI> source,
							   Eigen::Isometry3d init_pose) ;
	//2
	void solveOneSVD(pcl::PointCloud<pcl::PointXYZI>::Ptr target, pcl::PointCloud<pcl::PointXYZI> source);
	//3
	bool solveOneLMSVD (const std::vector<cv::Point3f> &pts1, const std::vector<cv::Point3f> &pts2,
						cv::Mat &R, cv::Mat &t, Eigen::Isometry3d &se3);
	//tools
	void timeUsed() {
		auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>
				(std::chrono::high_resolution_clock::now() - _now_ms);
		std::cout<<name_global<<" time is :"<< duration.count()/1e9<<std::endl;
		
	}
	
	void timeCalcSet(std::string name) {
		_now_ms = std::chrono::high_resolution_clock ::now();
		name_global = name;
		
	}
private:
	std::string name_global;
	std::chrono::high_resolution_clock::time_point _now_ms;
	Eigen::Isometry3d T;
	std::vector<cv::Point3f> pts1, pts2; //目前的点对 1:目标点 2.调整点
	Eigen::Vector3d vp1,vp2;
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeLast;
};


#endif //PCD_COMPARE_DAICP_H
