//
// Created by echo on 2020/4/15.
//

#ifndef PCD_COMPARE_LOOPCLOSURE_H
#define PCD_COMPARE_LOOPCLOSURE_H
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <dirent.h>
#include <pcl/filters/voxel_grid.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

using namespace g2o;
class loopClosure {
public:
	loopClosure(){};
	void addLoopEdge(int num1, int num2, std::string g2o_read,std::string g2o_save,std::string pcd_path);
private:
	//函数
	bool GetFileNames(const std::string directory,const std::string suffix);
	bool FindFileseq(int64_t seq);
	
	void saveFile(std::string outFilename, std::vector<VertexSE3*> vertices,
				  std::vector<EdgeSE3*> edges);
	void SaveTrans(Eigen::Isometry3d curr); //定义存储ICP edge 的函数
	pcl::PointCloud<pcl::PointXYZ> GICP(const pcl::PointCloud<pcl::PointXYZ> & cloud_source,
										const pcl::PointCloud<pcl::PointXYZ> & cloud_target,
										Eigen::Isometry3d & icp_matrix);
	void genlocalmap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
					 int num1,std::string filepath,pcl::PointCloud<pcl::PointXYZ>& bigmap);
	
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getEigenPoseFromg2oFile(std::string &g2ofilename);
	//变量
	std::vector<std::string> file_names_;
	std::string filename;
	std::string filepath = "/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_pcd/";
	Eigen::Isometry3d curICP = Eigen::Isometry3d::Identity();
	int cur_id = 0;
	int past = 0;

	std::string g2o_path = "null";//打开g2o文件路径
	std::string save_g2o_path = "/home/echo/test1.g2o";//存g2o路径
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector;
};


#endif //PCD_COMPARE_LOOPCLOSURE_H
