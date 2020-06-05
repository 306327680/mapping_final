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
#include <pcl/kdtree/kdtree_flann.h>
#include "tools/util.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <6DOFcalib/Calibration6DOF.h>
#include <ndt_omp/include/pclomp/ndt_omp.h>
#include "ndt_omp/include/pclomp/gicp_omp.h"
#include <pcl/surface/mls.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
//todo 现在需要通过gps给定初值
using namespace g2o;
class loopClosure {
public:
	loopClosure(){};
	void addLoopEdge(int num1, int num2, std::string g2o_read,std::string g2o_save,std::string pcd_path);
	//1.点云里程计路径 2.优化后保存路径 3.雷达pcd路径 4.gps csv格式的数据 5.LiDAR csv格式数据
	void autoMaticLoopClosure(std::string LiDAR_g2o_read,std::string Final_g2o_save,std::string LiDAR_pcd_path,std::string GPScsvPath,std::string LiDARcsv);
	//6. 通过gps约束初次优化factor
	void GPSLoopClosureCalc(std::string g2o_file_path);
	//2. 专用VLP 去畸变的local map
	
	void genVLPlocalmap(int length,std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
						int num1,std::string filepath,pcl::PointCloud<pcl::PointXYZ>& bigmap);
	void genVLPlocalmap(int length,std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
						int num1,std::string filepath,pcl::PointCloud<pcl::PointXYZI>& bigmap);
private:
	std::vector<VertexSE3*> vertices;
	std::vector<EdgeSE3*> edges;
	g2o::SparseOptimizer optimizer;
	//函数
	bool GetFileNames(const std::string directory,const std::string suffix);
	bool FindFileseq(int64_t seq);
	
	void saveFile(std::string outFilename, std::vector<VertexSE3*> vertices,
				  std::vector<EdgeSE3*> edges);
	void SaveTrans(Eigen::Isometry3d curr); //定义存储ICP edge 的函数
	void SaveLiDAREdge();
	pcl::PointCloud<pcl::PointXYZ> GICP_OMP(const pcl::PointCloud<pcl::PointXYZ> & cloud_source,
										const pcl::PointCloud<pcl::PointXYZ> & cloud_target,
										Eigen::Isometry3d & icp_matrix);
	pcl::PointCloud<pcl::PointXYZ> GICP(const pcl::PointCloud<pcl::PointXYZ> & cloud_source,
										const pcl::PointCloud<pcl::PointXYZ> & cloud_target,
										Eigen::Isometry3d & icp_matrix);
	pcl::PointCloud<pcl::PointXYZ> NDT(const pcl::PointCloud<pcl::PointXYZ> & cloud_source,
										const pcl::PointCloud<pcl::PointXYZ> & cloud_target,
										Eigen::Isometry3d & icp_matrix);
	pcl::PointCloud<pcl::PointXYZ> NDT_OMP(const pcl::PointCloud<pcl::PointXYZ> & cloud_source,
									   const pcl::PointCloud<pcl::PointXYZ> & cloud_target,
									   Eigen::Isometry3d & icp_matrix);
	//1. 带surface normal的
	pcl::PointCloud<pcl::PointXYZ> NormalICP(const pcl::PointCloud<pcl::PointXYZ> & cloud_source,
										const pcl::PointCloud<pcl::PointXYZ> & cloud_target,
											 Eigen::Isometry3d init_pose,Eigen::Isometry3d & icp_matrix);
	float ndt_score;
	void genlocalmap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
					 int num1,std::string filepath,pcl::PointCloud<pcl::PointXYZ>& bigmap);

	void gps2pcd(std::string GPScsvPath);
	void LiDAR2pcd(std::string LiDARcsvPath);
	//3.通过GPS index得到当前的 lla坐标系下 pose 6dof
	Eigen::Isometry3d GPSPose(int index);
	//4.得到T_G_L * GPS 到 LiDAR的相机变换
	Eigen::Isometry3d GPSLiDARExtrinsicParam(int GPSindex);
	//5.找到correspondece
	std::vector<Eigen::Vector2d> LiDAR_GPS_math();

	//7. 再次进行加边的操作
	//8. 找到闭环条件
	void findLoopFromGPS();
	//9. 通过gps给定初值 + icp匹配
	void GPS_align_TF_calc();
	//10. 自身的odom去增加闭环
	void Odom_align_TF_calc();
	//11. 自身odom找到闭环
	void findLoopFromOdom();
	int FromGPSgetLiDARindex(int gpsIndex);//从雷达得到gps index
	void simpleDistortion(VLPPointCloud::Ptr pointIn, Eigen::Isometry3d transform, VLPPointCloud &pointOut);//
	void trinterp(Eigen::Matrix4d &T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T);
	void qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r,
							  Eigen::Vector4d &q_quaternion_interpolation);
	void rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion);
	void quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R);
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getEigenPoseFromg2oFile(std::string &g2ofilename);
	//变量
	std::vector<std::string> file_names_;
	std::string filename;
	std::string filepath = "/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_pcd/";
	Eigen::Isometry3d curICP = Eigen::Isometry3d::Identity();
	int cur_id = 0;
	int past = 0;

	std::string g2o_path = "null";//打开g2o文件路径
	std::string save_g2o_path = "/home/echo/autoLoop.g2o";//存g2o路径
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector;
	//autoMaticLoopClosure 用
	pcl::PointCloud<pcl::PointXYZI> gps; //intensity 为时间
	std::vector<Eigen::Vector2d> LiDAR_index_time;//
	std::vector<Eigen::Vector2d> GPS_Loop_from_to;//LiDAR_GPS_math()用
	std::vector<Eigen::Vector2d> Odom_Loop_from_to;//LiDAR_GPS_math()用
	std::vector<Eigen::Vector2d> LiDARmatchIndex; //1.lidar 和 2.gps 对应关系
	nav_msgs::Path loopClosurePath;
	pcl::PointCloud<pcl::PointXYZI> select;
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> optimized_lidar_odom;
	pcl::PointCloud<pcl::PointXYZI> optimized_lidar_odom_pcd;
};


#endif //PCD_COMPARE_LOOPCLOSURE_H
