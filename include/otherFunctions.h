//
// Created by echo on 19-5-24.
//

#ifndef PCD_COMPARE_MAIN_H
#define PCD_COMPARE_MAIN_H

#define PCL_NO_PRECOMPILE
#include <iostream>
#include <pcl/io/pcd_io.h>//不能两次引用头文件
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
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
#include "distortion/beam_separate.h"
#include "mapping/lmOptimizationSufraceCorner.h"
#include "oneFrameGND/ground_seg.h"
#include "spline/experiment.h"
#include "g2oIO/PoseGraphIO.h"
#include "DataIO/ReadBag.h"
#include "GPS/gpsTools.h"
#include "registration/registration.h"
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/cloud_viewer.h>
#include "ndt_omp/include/pclomp/ndt_omp.h"
#include <ndt_omp/include/pclomp/gicp_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ros/ros.h>
#include <6DOFcalib/Calibration6DOF.h>
#include <nav_msgs/Path.h>
#include <string>
#include <iostream>
#include "imgAddColor2Lidar/imgAddColor2Lidar.h"
#include "loopClosure/loopClosure.h"
#include "GPS_constraint_mapping/GPS_loop_mapping.h"
#include <pcl/octree/octree_search.h>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Isometry3d)
using namespace g2o;
class main_function {
public:
	main_function(){};
	std::string LiDAR_type = "Velodyne";
//extern const std::string LiDAR_type = "Hesai";
// 1. 参数初始化
	bool tensorvoting = true;

	std::vector<std::string> file_names_;
	std::vector<std::string> PNG_file_names_;
	std::string filename;
	std::string filepath = "/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/car_pcd";
	Eigen::Isometry3d curICP = Eigen::Isometry3d::Identity();
	int cur_id = 0;
	int start_id = 0;//设置开始结束的点
	int end_id = 2;
//打开g2o文件路径
	std::string g2o_path = "/media/echo/DataDisc/3_program/mapping/cmake-build-debug/gps_constrained.g2o";
	int past = 0;
//存g2o路径
	std::string save_g2o_path = "/home/echo/test.g2o";
//存点云的路径
	std::string save_pcd_path = "/home/echo/map.pcd";
	std::string save_color_pcd_path = "/home/echo/map_color.pcd";
//存储g2o为iso3d
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector;
//闭环测试
	loopClosure lc;
//去动态点云的部分
	pcl::PointCloud<pcl::PointXYZI> DynamicLocalMap;
	pcl::PointCloud<pcl::PointXYZI> DynamicGlobalMap;
	pcl::PointCloud<pcl::PointXY> LocalMap_Yaw_pitch;
	//oc tree
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> *octree;
	pcl::PointCloud<pcl::PointXYZI>::Ptr octpcd;
	float resolution;
//0.设置起始结束的pcd
	void setStartEnd();

//1. 设置输入模式: 1.g2o+pcd的传统模式(pcd+g2o路径) 2.point to plane ICP (需要提供pcd路径)
	int status = 0;

//功能1 用g2o pose 建图/**/ 就是用一一对应的g2o文件 进行拼图操作
	int g2omapping();
	
	int g2oColorMapping();

//2. 输入模式的函数
	int getParam(int argc, char **argv);

// 3. 得到名称
	bool GetFileNames(const std::string directory, const std::string suffix);
	
	bool GetPNGFileNames(const std::string directory, const std::string suffix);

//4. 排序
	bool FindFileseq(int64_t seq);

//5. 保存g2o
	void saveFile(std::string outFilename, std::vector<VertexSE3 *> vertices,
				  std::vector<EdgeSE3 *> edges);

//6 .读取g2o文件
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getEigenPoseFromg2oFile(
			std::string g2ofilename);

//7. 读trans和附近pcd拼成点云
//7.1.trans vector 7.2.查找的位置 7.3.点云读取的路径
	void genlocalmap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
					 std::string filepath, pcl::PointCloud<pcl::PointXYZI> &bigmap);
	
	void genColormap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
					 std::string picParam);

//8. 用来测试插值的 输入: 位姿vector
	void testspline(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector);

//9. feature map loam 特针点的地图
	void genfeaturemap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
					   std::string filepath, pcl::PointCloud<pcl::PointXYZI> &bigmap);

//10.0 线性去畸变的接口
	void simpleDistortion(mypcdCloud input, Eigen::Matrix4f increase, pcl::PointCloud<pcl::PointXYZI> &output);

//11.1 omp NDT 配准
	pcl::PointCloud<pcl::PointXYZI>::Ptr align(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration,
											   const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud,
											   const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud);

//12.1.1 距离滤波
	void pointCloudRangeFilter(pcl::PointCloud<pcl::PointXYZI> &input, float range);

//13. 转换一个点的坐标
	void transformOnePoint(Eigen::Matrix4f t, pcl::PointXYZI &input);

//14. tools 建立局部地图 //a. 挑选关键帧(有必要?) //b. 维持队列长度 //d. downsample 15 帧大概1.2 s
	pcl::PointCloud<pcl::PointXYZI>
	lidarLocalMap(std::vector<Eigen::Matrix4f> &poses, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clouds,
				  int buffer_size);

//15. 对4个位姿的点进行连续去畸变 自己维护一下序列() input 4个位姿 从t-1 到t+2 输入 t时刻的点云//输出 continusTime 去过畸变的点云
	pcl::PointCloud<pcl::PointXYZI>
	continusTimeDistrotion(std::vector<Eigen::Matrix4f> &poses, std::vector<mypcdCloud> &clouds);

//16 LiDAR 局部地图生成(){} 通过距离来
	pcl::PointCloud<pcl::PointXYZI>
	lidarLocalMapDistance(std::vector<Eigen::Matrix4f> &poses, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clouds,
						  double distiance, int buffer_size, bool &local_map_updated,
						  pcl::PointCloud<pcl::PointXYZI> last_local_map);
//17 动态物体去除的local map 输出当前帧的点云和位姿,输出去除动态物体的点云 同时维护一个没有动态物体的点云

	void localMapOctInit(float resolution_1){
		resolution = resolution_1;
		octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(resolution_1);

		//octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::InterestPoint>(resolution_1)) ;
		octpcd.reset(new pcl::PointCloud<pcl::PointXYZI>());
		octree->setInputCloud (octpcd);
	};
	pcl::PointCloud<pcl::PointXYZI> localMapOct(pcl::PointCloud<pcl::PointXYZI> last_fine,pcl::PointCloud<pcl::PointXYZI> this_coarse);
	
	ros::Time fromPath2Time(std::string s);

//6.2 建图前端 点面icp ****************
	int point2planeICP();

// 功能3 设置road curb 的mapping
	void traversableMapping();

// 功能4 使用encoder 和GPS LiDAR 去建图
	void encoderMapping();

//功能5.1 LiDAR + GNSS mapping 杆臂值 标定
//程序准备设计的方案: 1. 读取一个bag 取其中的一段时间 eg 60s 600 帧进行 odom的计算//2. 读取gps 转成lla//3. 时间戳对齐//4. 设置两个优化变量 一个是两个传感器的外参 一个是旋转的角度,就是雷达初始时刻相对于正北的朝向;
// /media/echo/DataDisc/9_rosbag/rsparel_64_ins 这个好像能work gps时间戳不太对 就用时间戳减1s
	Eigen::Isometry3d LiDARGNSScalibration(std::string lidar_g2o, std::string gps_pcd);
	
	void LiDARGNSSMapping() {}

//功能6. NDT mapping
	void NDTmapping();

//功能7 读取hesaipcd
	void readAndSaveHesai(std::string path) {
		ReadBag a;
		//a.readHesai(path);
		//a.readVLP16("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/2020-05-11-16-07-59.bag","/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/car_pcd");
		//a.readTopRobosense("/media/echo/DataDisc/9_rosbag/9_huawei_jialuowuliu/2020-04-09-11-44-45.bag","/home/echo/2_huawei");
		
		//a.saveRTK2PCD("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/2020-05-11-16-07-59.bag");//把rtk保存成 csv+pcd
		a.readcamera("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/2020-05-11-16-07-59.bag","/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/car_pcd1");
		a.readVLP16("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/2020-05-11-16-07-59.bag","/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/car_pcd");
/*		std::vector<std::pair<Eigen::Isometry3d,double>>  gps_pose ;
		Eigen::Vector3d lla_origin;
 		a.gnssPCDExtrinsicParameters("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_fov.bag",gps_pose,lla_origin);*/
		
	}

//功能8 用来测试模块好使不
	void testFunction() {
		//1.测时间戳
/*		ros::Time first_time;
		first_time = fromPath2Time(file_names_[0]);
		for(int i = 0;  i <file_names_ .size();i++){
			ros::Time cur_time;
			cur_time = fromPath2Time(file_names_[i]);
			std::cout<<(cur_time-first_time).toSec()<<std::endl;
		}*/
		//2.测彩色点云
/*		pcl::PCDWriter writer;
		imgAddColor2Lidar a;
		a.readExInt("/home/echo/fusion_ws/src/coloured_cloud/ex_params.txt");
		pcl::PointCloud<pcl::PointXYZRGB> tosave;
		cv::Mat mat;VLPPointCloud cloudin;
		mat = cv::imread("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/pic/1586507671.986369610.png");
		pcl::io::loadPCDFile<VLPPoint>("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_pcd/1586507671.84489393.pcd", cloudin);
		GetFileNames("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/pic","png");
		tosave  = a.pclalignImg2LiDAR(mat,cloudin);
		pcl::PointXYZRGB a1;
		writer.write("/home/echo/fusion_ws/result.pcd",tosave, true);*/
		//3.测闭环gps约束
		GPS_loop_mapping g;
		g.GPSandPose("/home/echo/car_imu_lidar_rtk_cam/test1.g2o", "/home/echo/car_imu_lidar_rtk_cam/gps.pcd",
					 LiDARGNSScalibration("/home/echo/car_imu_lidar_rtk_cam/test1.g2o",
										  "/home/echo/car_imu_lidar_rtk_cam/gps.pcd"));


		//4.测闭环 1.加闭环边
		//lc.addLoopEdge(780,7550,"/home/echo/test.g2o","/home/echo/test1.g2o","/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/car_pcd/");
		//5.测闭环后的建图
		trans_vector = getEigenPoseFromg2oFile(
				"/media/echo/DataDisc/3_program/mapping/cmake-build-debug/gps_constrained.g2o");
		start_id = 0;
		end_id = 8500;
		pcl::PointCloud<pcl::PointXYZI> fxxk;
		genlocalmap(trans_vector, "", fxxk);
		//genColormap(trans_vector,""); //5.1 带颜色的pcd
		
	}

//功能9 普通的16线建图
	void rslidarmapping() {
	}

//功能10. gpsbased mapping
	void gpsBasedOptimziation(std::string lidar_path, std::string gps_path, Eigen::Isometry3d lidar_to_gps,
							  std::string save_path);

private:
};


#endif //PCD_COMPARE_MAIN_H
