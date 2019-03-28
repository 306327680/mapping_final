//
// Created by echo on 19-2-25.
//

#ifndef PCD_COMPARE_BEAM_SEPARATE_H
#define PCD_COMPARE_BEAM_SEPARATE_H
/*//#include <iostream>
//#include <boost/shared_ptr.hpp>
//#include <boost/thread/thread.hpp>
//#include <string>
//#include <dirent.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
//#include "g2o/solvers/pcg/linear_solver_pcg.h"
//#include "g2o/core/factory.h"
//#include "g2o/types/slam3d/vertex_se3.h"
//#include "g2o/types/slam3d/edge_se3.h"
//#include "g2o/stuff/sampler.h"
//#include "g2o/stuff/command_args.h"
//#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <Eigen/StdVector>
//#include <Eigen/Core>
//#include <pcl/registration/gicp.h>
//#include <pcl/registration/ndt.h>
//#include <pcl/register_point_struct.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/segmentation/progressive_morphological_filter.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/point_cloud_handlers.h>
//#include <pcl/filters/radius_outlier_removal.h>*/
#include <yaml-cpp/yaml.h>
#define PCL_NO_PRECOMPILE
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/gicp.h>
/*#include <pcl/visualization/cloud_viewer.h>*/
#include "util.h"

class beam_separate {


};

//特征提取
class featureExtraction {
public:
#define PointCloud_T  pcl::PointCloud<pcl::PointXYZI>
#define PointCloudPtr_T pcl::PointCloud<pcl::PointXYZI>::Ptr
#define PointCloudConstPtr_T pcl::PointCloud<pcl::PointXYZI>::ConstPtr

//pcd writer
	pcl::PCDWriter writer_pcd;
	std::vector<int> scanStartInd;
	std::vector<int> scanEndInd;
	
	typedef PointTypeSm PointType;
	pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
	pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
	pcl::PointCloud<PointType>::Ptr surfPointsFlat;
	pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
	//函数
//1.1 构造
	featureExtraction(){
		cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
		cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
		surfPointsFlat.reset(new pcl::PointCloud<PointType>());
		surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());
		scanStartInd.resize(N_SCAN, 0);
		scanEndInd.resize(N_SCAN, 0);
	};
//1.2------计算三点的圆心------
	void getCircleCenter(PointTypeSm &p1, PointTypeSm &p2, PointTypeSm &p3,
						 float &x0,float &y0,float &z0,float &r);
//1.3------计算特征------
	void calcFeature(pcl::PointCloud<PointTypeSm>::Ptr &laserCloud);
//1.4------计算光滑度------
	void calculateSmoothness(pcl::PointCloud<PointTypeBeam>::Ptr segmentedCloud_bef,
							 pcl::PointCloud<PointTypeSm>::Ptr &segmentedCloud);

//1.------读点云序列------
	bool GetFileNames(const std::string directory,const std::string suffix,
					  std::vector<std::string> & file_names_);
//2.------复制类型------
	void copyPCD(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef,
				 pcl::PointCloud<PointTypeBeam>::Ptr &test);

//3.------GICP------
	PointCloud_T GICP(const PointCloud_T & cloud_source,
					  const PointCloud_T & cloud_target,
					  Eigen::Isometry3d & icp_matrix, double distance) ;
	
//6. 分割字符串
	void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c);
//4.普通 icp
	PointCloud_T ICP(const PointCloud_T & cloud_source,
					 const PointCloud_T & cloud_target,
					 Eigen::Isometry3d & icp_matrix, double distance) ;

//8.用来检查这个型号的激光雷达数据读入的顺序 用pcl_viewer可视化一下
	void checkorder(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef,
					pcl::PointCloud<PointTypeBeam>::Ptr &out);
	
// 10.2 矩阵转四元数  ， 实部在前，虚部在后
	void rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion);

// 10.3 四元数转旋转矩阵
	void quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R);
	
//10.4 四元数插值计算
	void  qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d & Q2, double r,
				  Eigen::Vector4d &q_quaternion_interpolation);
	
// 10.5 两个矩阵之间插值计算
// input :1. 输入的第一个矩阵 2. 输入的第二个矩阵 3. 中间插多少 4. 输出
	void trinterp(Eigen::Matrix4d &T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T) ;
	
//9.点云去畸变 Isometry 3d 是 下一次的位姿(运动到的位姿/odom的位姿)
	void adjustDistortion(pcl::PointCloud<PointTypeBeam>::Ptr pointIn ,
						  pcl::PointCloud<PointTypeBeam>::Ptr &pointOut,
						  Eigen::Isometry3d transform);
	
private:
	// VLP-16
	const int N_SCAN = 16;
//每根线的间距
	const float Div = 0.95;
	const int Horizon_SCAN = 1800;
	const float ang_res_x = 0.2;
	const float ang_res_y = 2.0;
	const float ang_bottom = 15.0+0.1;
	const int groundScanInd = 7;
//枚举类型
	enum PointLabel {
		CORNER_SHARP = 2,       ///< sharp corner point
		CORNER_LESS_SHARP = 1,  ///< less sharp corner point
		SURFACE_LESS_FLAT = 0,  ///< less flat surface point
		SURFACE_FLAT = -1       ///< flat surface point
	};
//点的曲度
	float cloudCurvature[90000];
	int cloudSortInd[90000];
	int cloudNeighborPicked[90000];
	PointLabel cloudLabel[90000];

//13 loam 的特征提取部分
/** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
	int N_FEATURE_REGIONS = 6;
/** The curvature threshold below / above which a point is considered a flat / corner point. */
	float SURFACE_CURVATURE_THRESHOLD = 0.1;
/** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
	int CURVATURE_REGION = 5;
/** The maximum number of sharp corner points per feature region. */
	int MAX_CORNER_SHARP = 2;
/** The maximum number of less sharp corner points per feature region. */
	int MAX_CORNER_LESS_SHARP = 20;
/** The maximum number of flat surface points per feature region. */
	int MAX_SURFACE_FLAT = 4;
/** The voxel size used for down sizing the remaining less flat surface points. */
	float LESS_FLAT_FILTER_SIZE = 0.2;
//特征提取权重
	float X_INDEX = 1;
	float Y_INDEX = 1;
	float Z_INDEX = 1;
//线束距离差 初值设定这里
	double orientationDiff = 6.34;
};


#endif //PCD_COMPARE_BEAM_SEPARATE_H
