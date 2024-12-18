//
// Created by echo on 19-3-6.
//

#ifndef PCD_COMPARE_UTIL_H
#define PCD_COMPARE_UTIL_H

#include <string>
#include <chrono>
#include <iostream>
#include <string>
#include <dirent.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
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
G2O_USE_TYPE_GROUP(slam3d);
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
//新的点云类型


struct PointXYZIBT
{
	PCL_ADD_POINT4D
	PCL_ADD_INTENSITY;
	float beam;
	float pctime;
	float range;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIBT,
								   (float, x, x) (float, y, y)
										   (float, z, z) (float, intensity, intensity)
										   (float, beam, beam)
										   (float, pctime, pctime)
										   (float, range, range)
)

typedef PointXYZIBT  PointTypeBeam;
//点云类型2 光滑度的
struct PointXYZIBS
{
	PCL_ADD_POINT4D
	PCL_ADD_INTENSITY;
	float beam;
	float pctime;
	float pointType;
	float range;
	float smooth;
	float NeighborPicked;
	float curvature;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIBS,
								   (float, x, x) (float, y, y)
										   (float, z, z) (float, intensity, intensity)
										   (float, beam, beam)
										   (float, pctime, pctime)
										   (float, pointType, pointType)
										   (float, range, range)
										   (float, smooth, smooth)
										   (float, NeighborPicked, NeighborPicked)
										   (float, curvature, curvature)
)

typedef PointXYZIBS  PointTypeSm;

//todo  my pcd
 
	struct PointXYZITR {
		PCL_ADD_POINT4D
		float intensity;
		float timestamp;
		float ring;                      ///< laser ring number
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
	} EIGEN_ALIGN16;
	
 

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITR,
								  (float, x, x)(float, y, y)(float, z, z)
										  (float, intensity, intensity)(float, timestamp, timestamp)(float, ring, ring))

typedef PointXYZITR mypcd;
typedef pcl::PointCloud<mypcd> mypcdCloud;

//0.1. hesai LiDAR
namespace pandar_pointcloud
{
	struct PointXYZIT {
		PCL_ADD_POINT4D
		float intensity;
		double timestamp;
		uint16_t ring;                      ///< laser ring number
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
	} EIGEN_ALIGN16;
	
};

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZIT,
								  (float, x, x)(float, y, y)(float, z, z)
										  (float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

typedef pandar_pointcloud::PointXYZIT PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;
//0.2 vlp16
 
 
	struct PointXYZIR {
		PCL_ADD_POINT4D
		float intensity;
		float time;
		uint16_t ring;                      ///< laser ring number
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
	} EIGEN_ALIGN16;
	
 

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIR,
								  (float, x, x)(float, y, y)(float, z, z)
										  (float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))
typedef  PointXYZIR VLPPoint;
typedef pcl::PointCloud<VLPPoint> VLPPointCloud;

//0.3 robosense
struct PointRobo {
	PCL_ADD_POINT4D
	float intensity;
	float time;
	float ring;                      ///< laser ring number
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT( PointRobo,
								   (float, x, x)(float, y, y)(float, z, z)
										   (float, intensity, intensity)(float, time, time)(float, ring, ring))
typedef  PointRobo RoboPoint;
typedef pcl::PointCloud<RoboPoint> RoboPointCLoud;
//0.4 xyzirgb

struct PointXYZRGBI {
	PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
	PCL_ADD_RGB;
	PCL_ADD_INTENSITY;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZRGBI,
								   (float, x, x)(float, y, y)(float, z, z)
										   (float, intensity, intensity) (std::uint32_t, rgba, rgba))

namespace ouster_ros {

    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                          (std::uint32_t, t, t)
                                          (std::uint16_t, reflectivity, reflectivity)
                                          (std::uint8_t, ring, ring)
                                          (std::uint16_t, ambient, ambient)
                                          (std::uint32_t, range, range)
)
 
/*namespace pcl
{
	struct EIGEN_ALIGN16 _PointXYZRGBI
	{
		PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
		PCL_ADD_RGB;
		PCL_ADD_INTENSITY;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
 
	struct EIGEN_ALIGN16 PointXYZRGBI : public _PointXYZRGBI
	{
		inline PointXYZRGBI (const _PointXYZRGBI &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			rgba = p.rgba;
			intensity = p.intensity;
		}
		
		inline PointXYZRGBI ()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			r = g = b = 0;
			a = 0;
			intensity = 0.0f;
		}
		
		inline PointXYZRGBI (uint8_t _r, uint8_t _g, uint8_t _b, float _intensity)
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			r = _r;
			g = _g;
			b = _b;
			a = 0;
			intensity = _intensity;
		}
		//PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBI& p);
		
		friend std::ostream& operator << (std::ostream& os, const PointXYZRGBI& p);
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	
}*/



//0. 工具类
class util {
public:
    int test;
	std::vector<std::string> file_names_;
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    void timeCalcSet(std::string name);
    void timeCalcSet();
    void timeUsed();
    
    //每根的点云
	std::vector<pcl::PointCloud<pcl::PointXYZINormal>> beam_pcd;
    //读取pcd序列
	bool GetFileNames(const std::string directory,const std::string suffix ="pcd");
	//PointXYZINormal normal x 为线数 y 为时间
	void GetPointCloudBeam(pcl::PointCloud<pcl::PointXYZINormal> pc_in,pcl::PointCloud<pcl::PointXYZINormal>& pc_out);
	//提取边缘点
	void GetBeamEdge(pcl::PointCloud<pcl::PointXYZINormal> pc_in,pcl::PointCloud<pcl::PointXYZINormal>& pc_out);
	//格式转换
/*	Eigen::Isometry3d odometryToEigen(nav_msgs::Odometry &o, Eigen::Isometry3d &e) {
		Eigen::Isometry3d is_test;
		is_test = Eigen::Isometry3d::Identity();
		tf::Transform tf_test;
		tf::poseMsgToTF(o.pose.pose, tf_test);
		tf::transformTFToEigen(tf_test, e);
		return e;
	}*/
	
	std::vector<int> unique_element_in_vector(std::vector<int> v){
		std::vector<int>::iterator vector_iterator;
		std::sort(v.begin(),v.end());
		vector_iterator = std::unique(v.begin(),v.end());
		if(vector_iterator != v.end()){
			v.erase(vector_iterator,v.end());
		}
		return v;
	}
	//读取g2o位姿
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getEigenPoseFromg2oFile(
			std::string &g2ofilename);
	std::string name_global;
private:
    std::chrono::high_resolution_clock::time_point _now_ms;

};


//loam里面的单位
//1.vector3
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
	
	Vector3(const PointTypeSm& p):Eigen::Vector4f( p.x, p.y, p.z,0) {}
	
/*	Vector3& operator=(const pcl::PointXYZI& point) {
		x() = point.x;
		y() = point.y;
		z() = point.z;
		return *this;
	}*/
	Vector3& operator=(const PointTypeSm& point) {
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
//2.angle

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

//3.rotatex

inline Vector3 rotateX(const Vector3& v,const Angle& ang)
{
	return Vector3( v.x(),
					ang.cos() * v.y() - ang.sin() * v.z(),
					ang.sin() * v.y() + ang.cos() * v.z() );
}
//3.1 rotate y
inline Vector3 rotateY(const Vector3& v,const Angle& ang)
{
	return Vector3( ang.cos() * v.x() + ang.sin() * v.z(),
					v.y(),
					-ang.sin() * v.x() + ang.cos() * v.z() );
}
//3.2 rotate z
inline Vector3 rotateZ(const Vector3& v,const Angle& ang)
{
	return Vector3( ang.cos() * v.x() - ang.sin() * v.y(),
					ang.sin() * v.x() + ang.cos() * v.y(),
					v.z() );
}
//4. twist
struct Twist{
	Angle rot_x;
	Angle rot_y;
	Angle rot_z;
	Vector3 pos;
};

/*//3.循环进行icp
for(int i = start_pcd; i< end_pcd;i = i + step){
  //(1) 读取当前pcd trans到预测位置 (2) 得到tf 把当前帧累加()()()()
  pcl::io::loadPCDFile<pcl::PointXYZI>(file_names[i], *gicp_aft);
  std::vector<int> indices1;
  pcl::removeNaNFromPointCloud(*gicp_aft, *gicp_aft, indices1);
  //3.a降采样
  if(voxel_before){
	pcl::VoxelGrid<pcl::PointXYZI> sor1;
	sor1.setInputCloud(gicp_aft);                           //设置需要过滤的点云给滤波对象
	sor1.setLeafSize(voxel_size,voxel_size,voxel_size);     //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
	sor1.filter(*voxel);
	*gicp_aft = *voxel;
  }
  if(hgieht){
	pcl::PassThrough<pcl::PointXYZI> pass;   //1. passthrough
	pass.setInputCloud (gicp_aft);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (height_pass, height_pass_up);
	pass.filter (*voxel);
	*gicp_aft = *voxel;
  }

  if(i == start_pcd){
	*cloud_bef = *gicp_aft;
	*gicp_bef = *gicp_aft;
	*cloud_bef = *gicp_aft;
	trans_vector.push_back(Eigen::Isometry3d::Identity());
  } else{
*//*      //a. 上次得到的点云来设置初值 gicp_aft * tf  = gicp_aft_tfed
      pcl::transformPointCloud(*gicp_aft, *gicp_aft_tfed, gicp_result.matrix());
      GICP(*cloud_bef,*gicp_aft_tfed,gicp_result);
      //b. 累加位姿
      gicp_result = trans_vector.back()*gicp_result.inverse();
      trans_vector.push_back(gicp_result);
      //c. 得到真实位姿点云 (in, out, tf)
      pcl::transformPointCloud(*gicp_aft, *gicp_aft_tfed, gicp_result.matrix());
      *cloud_bef += *gicp_aft_tfed;*//*
      //a. 上次得到的点云来设置初值 gicp_aft * tf  = gicp_aft_tfed
      if(single){
        if(use_gicp){
          GICP(*gicp_aft,*gicp_bef,gicp_result,distance);
          //GICP(*gicp_aft,*cloud_bef,gicp_result_bigmap);
          //b. 累加位姿
          gicp_result = trans_vector.back()*gicp_result;
          trans_vector.push_back(gicp_result);
          //c. 得到真实位姿点云 (in, out, tf)
          pcl::transformPointCloud(*gicp_aft, *gicp_aft_tfed, gicp_result.matrix());
          cout<< gicp_result.matrix()<<endl;
          pcl::VoxelGrid<pcl::PointXYZI> sor;
          sor.setInputCloud(cloud_bef);                   //设置需要过滤的点云给滤波对象
          sor.setLeafSize(0.1,0.1,0.1);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
          sor.filter(*cloud_bef_filter);
          *cloud_bef = *cloud_bef_filter;
          *cloud_bef += *gicp_aft_tfed;
          *gicp_bef = *gicp_aft;
        }else{
          ICP(*gicp_aft,*gicp_bef,gicp_result,distance);
          //b. 累加位姿
          gicp_result = trans_vector.back()*gicp_result;
          trans_vector.push_back(gicp_result);
          //c. 得到真实位姿点云 (in, out, tf)
          pcl::transformPointCloud(*gicp_aft, *gicp_aft_tfed, gicp_result.matrix());
          pcl::VoxelGrid<pcl::PointXYZI> sor;
          sor.setInputCloud(cloud_bef);                   //设置需要过滤的点云给滤波对象
          sor.setLeafSize(0.1,0.1,0.1);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
          sor.filter(*cloud_bef_filter);
          *cloud_bef = *cloud_bef_filter;
          *cloud_bef += *gicp_aft_tfed;
          *gicp_bef = *gicp_aft;
        }

      }else{
        if(use_gicp){
          GICP(*gicp_aft,*cloud_bef,gicp_result,distance);
          //b. 累加位姿
          trans_vector.push_back(gicp_result);
          //c. 得到真实位姿点云 (in, out, tf)
          pcl::transformPointCloud(*gicp_aft, *gicp_aft_tfed, gicp_result.matrix());
          cout<< gicp_result.matrix()<<endl;
          pcl::VoxelGrid<pcl::PointXYZI> sor;
          sor.setInputCloud(cloud_bef);                   //设置需要过滤的点云给滤波对象
          sor.setLeafSize(0.1,0.1,0.1);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
          sor.filter(*cloud_bef_filter);
          *cloud_bef = *cloud_bef_filter;
          *cloud_bef += *gicp_aft_tfed;
          *gicp_bef = *gicp_aft;
        }else{
          pcl::transformPointCloud(*gicp_aft, *gicp_aft_tfed, gicp_result.matrix());
          cout<<"第一个 初始化icp的: \n"<<gicp_result.matrix()<<endl;
          gicp_result_last = gicp_result.matrix();
          ICP(*gicp_aft_tfed,*cloud_bef,gicp_result,distance);
          //c. 得到真实位姿点云 (in, out, tf)
          gicp_result_last = gicp_result_last*gicp_result;
          cout<<"第3个 总共的tf: \n"<<gicp_result_last.matrix()<<endl;
          pcl::transformPointCloud(*gicp_aft, *gicp_aft_tfed, gicp_result_last.matrix());
          cout<< gicp_result.matrix()<<endl;
          pcl::VoxelGrid<pcl::PointXYZI> sor;
          sor.setInputCloud(cloud_bef);                   //设置需要过滤的点云给滤波对象
          sor.setLeafSize(0.1,0.1,0.1);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
          sor.filter(*cloud_bef_filter);
          *cloud_bef = *cloud_bef_filter;
          *cloud_bef += *gicp_aft_tfed;
          *gicp_bef = *gicp_aft;
        }
      }
    }
  }
  //4.点云格式转换
  copyPCD(cloud_bef_filter, test);
  // 可视化 ---------

  test->points.data()->x = cloud_bef->points.data()->x;
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerGenericField <PointTypeBeam> handler(test,"intensity");

  viewer.addPointCloud<PointTypeBeam>(test, handler, "Final cloud");
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  viewer.addCoordinateSystem (10.0);
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (); }
  return(0);*/
// 可视化 ---------

#endif //PCD_COMPARE_UTIL_H
