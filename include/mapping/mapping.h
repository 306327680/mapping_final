//
// Created by echo on 19-3-18.
//

#ifndef PCD_COMPARE_MAPPING_H
#define PCD_COMPARE_MAPPING_H
#include <iostream>
#include <fstream>
#include <string>

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix标准函数定义*/
#include <sys/types.h>  /**/
#include <sys/stat.h>   /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/

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
#define PCL_NO_PRECOMPILE
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/gicp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;
using namespace g2o;
typedef pcl::PointXYZI PointType;

class Vector3 : public Eigen::Vector4f
{
public:
	
	Vector3(float x, float y, float z):Eigen::Vector4f(x,y,z,0) {}
	Vector3(void):Eigen::Vector4f(0,0,0,0) {}
	template<typename OtherDerived>
	Vector3(const Eigen::MatrixBase<OtherDerived>& other)
			: Eigen::Vector4f(other)
	{ }
	template<typename OtherDerived>
	Vector3& operator=(const Eigen::MatrixBase <OtherDerived>& other){
		this->Eigen::Vector4f::operator=(other);
		return *this;
	}
	Vector3(const pcl::PointXYZI& p):Eigen::Vector4f( p.x, p.y, p.z,0) {}
	
	Vector3& operator=(const pcl::PointXYZI& point) {
		x() = point.x;
		y() = point.y;
		z() = point.z;
		return *this;
	}
	float x() const { return (*this)(0); }
	float y() const { return (*this)(1); }
	float z() const { return (*this)(2); }
	
	float& x() { return (*this)(0); }
	float& y() { return (*this)(1); }
	float& z() { return (*this)(2); }
};

class Angle{
public:
	Angle():
			_ang(0.0),
			_cos(1.0),
			_sin(0.0) {}
	
	Angle(float value):
			_ang(value),
			_cos(std::cos(value)),
			_sin(std::sin(value)) {}
	
	Angle( const Angle& other ):
			_ang( other._ang ),
			_cos( other._cos ),
			_sin( other._sin ) {}
	
	void operator =( const Angle& other){
		_ang = ( other._ang );
		_cos = ( other._cos );
		_sin = ( other._sin );
	}
	
	void operator +=( const float& val)   { *this = ( _ang + val) ; }
	void operator +=( const Angle& other) { *this = ( _ang + other._ang ); }
	void operator -=( const float& val)   { *this = ( _ang - val ); }
	void operator -=( const Angle& other) { *this = ( _ang - other._ang ); }
	
	Angle operator-() const
	{
		Angle out;
		out._ang = _ang;
		out._cos = _cos;
		out._sin = -(_sin);
		return out;
	}
	
	float value() const { return _ang; }
	
	float cos() const { return _cos; }
	
	float sin() const { return _sin; }

private:
	float _ang, _cos, _sin;
};


inline Vector3 rotateX(const Vector3& v,const Angle& ang)
{
	return Vector3( v.x(),
					ang.cos() * v.y() - ang.sin() * v.z(),
					ang.sin() * v.y() + ang.cos() * v.z() );
}

inline Vector3 rotateY(const Vector3& v,const Angle& ang)
{
	return Vector3( ang.cos() * v.x() + ang.sin() * v.z(),
					v.y(),
					-ang.sin() * v.x() + ang.cos() * v.z() );
}

inline Vector3 rotateZ(const Vector3& v,const Angle& ang)
{
	return Vector3( ang.cos() * v.x() - ang.sin() * v.y(),
					ang.sin() * v.x() + ang.cos() * v.y(),
					v.z() );
}

struct Twist{
	Angle rot_x;
	Angle rot_y;
	Angle rot_z;
	Vector3 pos;
};


class mapping {
public:
	//变量:
	typedef pcl::PointXYZI PointType;
	vector<VertexSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _vertices;
	vector<EdgeSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _odometryEdges;
	vector<EdgeSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _edges;
	//保存文件名

	string _outFilename = "Lidarmapping.g2o";
	//*****mapping
	const float scanPeriod = 0.1;
	
	const int stackFrameNum = 1;
	const int mapFrameNum = 5;
	
	double timeLaserCloudCornerLast = 0;
	double timeLaserCloudSurfLast = 0;
	double timeLaserCloudFullRes = 0;
	double timeLaserOdometry = 0;
	
	bool newLaserCloudCornerLast = false;
	bool newLaserCloudSurfLast = false;
	bool newLaserCloudFullRes = false;
	bool newLaserOdometry = false;
	
	int laserCloudCenWidth = 10;//10
	int laserCloudCenHeight = 5;//5
	int laserCloudCenDepth = 10;//10
	const int laserCloudWidth = 21;//21
	const int laserCloudHeight = 11;//11
	const int laserCloudDepth = 21;//21
	const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;//4851
//lidar视域范围内(FOV)的点云集索引
	int laserCloudValidInd[80000];//125;
//lidar周围的点云集索引
	int laserCloudSurroundInd[80000];//125
	
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;
//滤波用temp点
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2;
	pcl::PointCloud<PointType>::Ptr laserCloudOri;
	pcl::PointCloud<PointType>::Ptr coeffSel;
	pcl::PointCloud<PointType>::Ptr laserCloudSurround;
	pcl::PointCloud<PointType>::Ptr laserCloudSurround2;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
//这坨干啥的?
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2;
	
	Twist transformSum;//上次得到的lidar odom
	Twist transformIncre;
	Twist transformTobeMapped;
	Twist transformBefMapped;
	Twist transformAftMapped;
	
	int imuPointerFront = 0;
	int imuPointerLast = -1;
	const int imuQueLength = 200;
	
	double imuTime[200] = {0};
	float imuRoll[200] = {0};
	float imuPitch[200] = {0};
	//***
	//函数:
	mapping(){
		setup();
	};
	//1.保存每次优化后的位姿结果
	bool saveg2o(Eigen::Isometry3d curr);
	//2. 保存最终g2o文件
	bool savefile();
	//3. 设定
	void setup();
	//4. get point cloud
	void getPoint(pcl::PointCloud<PointType> corner,pcl::PointCloud<PointType> surface,pcl::PointCloud<PointType> full_cloud,Eigen::Isometry3d Initpose);
	//5.基于匀速模型，根据上次微调的结果(1)和odometry(2)这次与上次计算的结果(3)，
// 猜测一个新的世界坐标系的转换矩阵transformTobeMapped
//input:transformBefMapped transformSum transformAftMapped
//output:     transformAftMapped
	void transformAssociateToMap();
	//6.记录odometry发送的转换矩阵与mapping之后的转换矩阵，下一帧点云会使用(有IMU的话会使用IMU进行补偿)
	void transformUpdate();
	//7. 根据调整计算后的转移矩阵，将点注册到全局世界坐标系下
	void pointAssociateToMap(PointType const * const pi, PointType * const po);
	//8. 点转移到局部坐标系下
	void pointAssociateTobeMapped(PointType const * const pi, PointType * const po);
private:
	vector<int> _vertxfromnum;
	vector<int> _vertextonum;
	//存两个闭环边的tf
	vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _trans_loopclosure;
	int _vertexnum = 0;
	Eigen::Matrix<double, 6, 6> _information = Eigen::Matrix<double, 6, 6>::Identity();
};


#endif //PCD_COMPARE_MAPPING_H
