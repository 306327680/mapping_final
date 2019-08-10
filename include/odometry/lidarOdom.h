//
// Created by echo on 19-3-11.
//

#ifndef PCD_COMPARE_LIDARODOM_H
#define PCD_COMPARE_LIDARODOM_H
#include <yaml-cpp/yaml.h>
#define PCL_NO_PRECOMPILE
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/gicp.h>
/*#include <pcl/visualization/cloud_viewer.h>*/
#include "tools/util.h"
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>



class lidarOdom {
public:
	//大循环中的迭代
	int iterCount = 0;
	//从第二次开始计算
	bool systemReady = false;
	typedef  PointTypeSm PointType;
	// VLP-16
	const int N_SCAN = 16;
//每根线的间距
	const float Div = 0.95;
	const int Horizon_SCAN = 1800;
	const float ang_res_x = 0.2;
	const float ang_res_y = 2.0;
	const float ang_bottom = 15.0+0.1;
	const int groundScanInd = 7;
	
	pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
	pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
	pcl::PointCloud<PointType>::Ptr surfPointsFlat;
	pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
	pcl::PointCloud<PointType>::Ptr laserCloudOri;
	pcl::PointCloud<PointType>::Ptr coeffSel;
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
	pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans;
	//测试结果
	pcl::PointCloud<PointType> test_result;
	const float scanPeriod = 0.1;
	
	const int skipFrameNum = 1;
	bool systemInited = false;
	
	double timeCornerPointsSharp = 0;
	double timeCornerPointsLessSharp = 0;
	double timeSurfPointsFlat = 0;
	double timeSurfPointsLessFlat = 0;
	double timeLaserCloudFullRes = 0;
	double timeImuTrans = 0;
	
	bool newCornerPointsSharp = false;
	bool newCornerPointsLessSharp = false;
	bool newSurfPointsFlat = false;
	bool newSurfPointsLessFlat = false;
	bool newLaserCloudFullRes = false;
	bool newImuTrans = false;
	bool firstPC = false;
	//增量
	double deltaR;
	double deltaT;
	//迭代结果
	Twist transform;
	//函数
	lidarOdom(){
		initializationValue();
	};
	//1.初始指针什么的
	void initializationValue();
	//2.得到点云
	void getPC(pcl::PointCloud<PointType>::Ptr cornerPointsSharp_out,
			   pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp_out,
			   pcl::PointCloud<PointType>::Ptr surfPointsFlat_out,
			   pcl::PointCloud<PointType>::Ptr surfPointsLessFlat_out,
			   pcl::PointCloud<PointType>::Ptr laserCloudFullRes_out);
	//3. 对边缘点进行处理
	void cornerConstraint();
	//4. 对平面点进行处理
	void surfConstraint();
	//5. loam自用函数
	void TransformToStart(PointType const * const pi, PointType * const po) {
/*		po->x = pi->x;
		po->y = pi->y;
		po->z = pi->z;
		po->intensity = pi->intensity;*/
		float s = 1;
//		float s = 10 * (pi->intensity - int(pi->intensity));
		//s 为时间的系数 线性差值
		Angle rx = s * transform.rot_x.value();
		Angle ry = s * transform.rot_y.value();
		Angle rz = s * transform.rot_z.value();
		//进行transform的补偿
		Vector3 v0( Vector3(*pi) -s * transform.pos );
		//进行旋转的补偿 z轴x轴y轴
		Vector3 v1 = rotateZ( v0, -rz );
		Vector3 v2 = rotateX( v1, -rx );
		Vector3 v3 = rotateY( v2, -ry );
		
		po->x = v3.x();
		po->y = v3.y();
		po->z = v3.z();
		po->intensity = pi->intensity;
	}
	bool frameOptiLM();

private:
	inline double rad2deg(double radians){
		return radians * 180.0 / M_PI;
	}
	
	int laserCloudCornerLastNum;
	int laserCloudSurfLastNum;
	
	int pointSelCornerInd[40000];
	float pointSearchCornerInd1[40000];
	float pointSearchCornerInd2[40000];
	
	int pointSelSurfInd[40000];
	float pointSearchSurfInd1[40000];
	float pointSearchSurfInd2[40000];
	float pointSearchSurfInd3[40000];
	
	pcl::KdTreeFLANN<PointType> kdtreeCornerLast;
	pcl::KdTreeFLANN<PointType> kdtreeSurfLast;
	PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;
	//lm的变量
	bool isDegenerate = false;
	Eigen::Matrix<float,6,6> matP;
	
};


#endif //PCD_COMPARE_LIDARODOM_H
