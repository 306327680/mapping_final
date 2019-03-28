//
// Created by echo on 19-3-21.
//

#ifndef PCD_COMPARE_FP_PC_VIEWING_H
#define PCD_COMPARE_FP_PC_VIEWING_H
#include <string>
#include <chrono>
#include <iostream>
/*#include <pcl/visualization/cloud_viewer.h>*/
#include "util.h"
#include <cmath>
#include <Eigen/Eigenvalues>
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

typedef  pcl::PointXYZI PointType;

class FP_PC_viewing {
public:
	FP_PC_viewing(){
		setup();
	};
	//1.计算每个线的角度
	void calcAngle(pcl::PointCloud<PointTypeSm>::Ptr examplePC);
	//2.读取地图
	//todo 可以分割成小地图提速
	void getmap(std::string mappath);
	//3.先分割成小的再转换到当前的位姿之后 再计算x,y,z,深度
	void get_pose(Eigen::Isometry3d pose);
	//4. 计算符合条件被滤除的点
	void calcData();
	//5. get real map
	void realmap();
	//6. 清空缓存
	void clear();
	//变量
	pcl::PointCloud<PointTypeSm>::Ptr curr_frame;
	pcl::PointCloud<PointTypeSm> maptemp_beam_iso;
	pcl::PointCloud<PointTypeSm> realmap_save;
	//删除后的点云
	pcl::PointCloud<PointTypeSm> map_reduced;
private:
	pcl::PointCloud<PointType>::Ptr mappoints;
	//保存线束的角度的
	std::vector<double> pc_in_angle;
	std::vector<int> pc_in_count;
	const int N_SCAN = 16;
	//临时储存地图
	pcl::PointCloud<PointTypeSm>::Ptr maptemp;
	pcl::PointCloud<PointTypeSm>::Ptr maptemp_local;
	pcl::PointCloud<PointTypeSm>::Ptr maptemp_beam;
	pcl::PointCloud<PointTypeSm>::Ptr maptemp_filter;
	
	pcl::PointCloud<PointTypeSm> single_tmp;
	//下面是用来快速找到对比用的点的变量
	std::vector<std::vector<pcl::PointCloud<PointTypeSm>>> fast_seg;
	std::vector<std::vector<pcl::PointCloud<PointTypeSm>>> fast_seg_ring;
	std::vector<std::vector<pcl::PointCloud<PointTypeSm>>> fast_map;
	std::vector<std::vector<pcl::PointCloud<PointTypeSm>>> fast_map_10m;
	//得到当前的位姿
	Eigen::Isometry3d curr_pose;
	//地图边界
	double max_x, max_y,min_x,min_y;
	//存index
	std::vector<int> filter_index;
	//看哪个点存进去
	std::vector<bool> map_point_save;
	//函数
	void setup();
};


#endif //PCD_COMPARE_FP_PC_VIEWING_H
