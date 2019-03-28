//
// Created by echo on 19-3-21.
//

#include "FP_PC_viewing.h"
#include <pcl/filters/passthrough.h>
void FP_PC_viewing::setup() {
	mappoints.reset(new pcl::PointCloud<PointType>());
	maptemp.reset(new pcl::PointCloud<PointTypeSm>());
	maptemp_local.reset(new pcl::PointCloud<PointTypeSm>());
	maptemp_beam.reset(new pcl::PointCloud<PointTypeSm>());
	maptemp_filter.reset(new pcl::PointCloud<PointTypeSm>());
	pc_in_angle.resize(N_SCAN);
	pc_in_count.resize(N_SCAN);
	fast_seg.resize(N_SCAN);
	fast_seg_ring.resize(N_SCAN);
	for (int i = 0; i < fast_seg.size(); ++i) {
		fast_seg[i].resize(361);
		fast_seg_ring[i].resize(361);
	}
}
//1
void FP_PC_viewing::calcAngle(pcl::PointCloud<PointTypeSm>::Ptr examplePC) {
	PointTypeBeam thisPoint;
	//防止内存泄露
	
	
	for(int i=0; i<examplePC->size(); i++){
		thisPoint.x = examplePC->points[i].x;
		thisPoint.y = examplePC->points[i].y;
		thisPoint.z = examplePC->points[i].z;
		//计算一帧点云的垂直角
		double verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y))
				* 180 / M_PI;
		pc_in_angle[examplePC->points[i].beam] = verticalAngle;
		//当前水平角度
		float cur_angle = (float)-atan2(examplePC->points[i].y, examplePC->points[i].x)* 180 / M_PI;
		examplePC->points[i].pctime = cur_angle;
		//给快速分割的数据结构放入点 线束*度数
		fast_seg_ring[examplePC->points[i].beam][floor(cur_angle+180)].push_back(examplePC->points[i]);
	}
}

void FP_PC_viewing::getmap(std::string mappath) {
	fast_map.clear();
	pcl::PointCloud<pcl::PointXYZI> cloud_bef;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_aft(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile<pcl::PointXYZI>(mappath, cloud_bef);
	*cloud_aft = cloud_bef;
/*	pcl::VoxelGrid<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud_aft);
	sor.setLeafSize(0.2, 0.2, 0.2);
	sor.filter(cloud_bef);*/

	maptemp->resize(cloud_bef.size());
	map_point_save.resize(cloud_bef.size());
	for(int i; i<cloud_bef.size(); i++){
		maptemp->points[i].x = cloud_bef.points[i].x;
		maptemp->points[i].y = cloud_bef.points[i].y;
		maptemp->points[i].z = cloud_bef.points[i].z;
		maptemp->points[i].intensity = cloud_bef.points[i].intensity;
		maptemp->points[i].NeighborPicked = i; // 点云的index
		map_point_save[i] = true;
		// 计算最大最小的点
		if(maptemp->points[i].x > max_x){
			max_x = maptemp->points[i].x;
		}
		if(maptemp->points[i].y > max_y){
			max_y = maptemp->points[i].y;
		}
		if(maptemp->points[i].x < min_x){
			min_x = maptemp->points[i].x;
		}
		if(maptemp->points[i].y <min_y){
			min_y = maptemp->points[i].y;
		}
	}
	//start fast map
	fast_map.resize(ceil(max_x)-floor(min_x));
	for (int j = 0; j < fast_map.size(); ++j) {
		fast_map[j].resize(ceil(max_y)-floor(min_y));
	}
	
	for(int i; i<maptemp->size(); i++){
		fast_map[floor(maptemp->points[i].x - min_x)][floor(maptemp->points[i].y - min_y)]
				.push_back(maptemp->points[i]);
	}
}
//2
void FP_PC_viewing::get_pose(Eigen::Isometry3d pose) {
	
	double x_pose = pose(0,3);
	double y_pose = pose(1,3);
	pcl::PointCloud<PointTypeSm>::Ptr cloud_bef(new pcl::PointCloud<PointTypeSm>);
	pcl::PointCloud<PointTypeSm>::Ptr tmp(new pcl::PointCloud<PointTypeSm>);
	//普通的filter 速度慢 模块相加的会快些 1m分辨率
	int x_range =60 ,y_range =60;
	for (int i = 0; i < x_range; ++i) {
		for (int j = 0; j < y_range; ++j) {
			//防止越界
			if(floor(x_pose+i-min_x -x_range/2)>=0&&floor(x_pose+i-min_x -x_range/2)<=fast_map.size()&&
					floor(y_pose+j-min_y-y_range/2)>=0&&floor(y_pose+j-min_y-y_range/2)<=fast_map[0].size()){
				*cloud_bef += fast_map[floor(x_pose+i-min_x - x_range / 2)][floor(y_pose + j - min_y - y_range/2)];
			}
		}
	}
	//转换到当前位姿
	pcl::transformPointCloud(*cloud_bef, *maptemp_local, pose.inverse().matrix());
}
//3
void FP_PC_viewing::calcData() {
	PointTypeBeam thisPoint;
	double verticalAngle;
	double range;
	float cur_angle;
	for(int i = 0; i < maptemp_local->size(); i++){
		thisPoint.x = maptemp_local->points[i].x;
		thisPoint.y = maptemp_local->points[i].y;
		thisPoint.z = maptemp_local->points[i].z;
		verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x +
				thisPoint.y * thisPoint.y)) * 180 / M_PI;
		range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y *
				thisPoint.y + thisPoint.z * thisPoint.z);
		cur_angle = (float)-atan2(maptemp_local->points[i].y, maptemp_local->points[i].x)
					* 180 / M_PI; //当前角度
		
		maptemp_local->points[i].range = range;
		maptemp_local->points[i].curvature = verticalAngle;
		maptemp_local->points[i].pctime = cur_angle;
		maptemp_local->points[i].beam = -1;
		//存储快速大地图中分割出来的
		for(int xx = 0; xx < pc_in_angle.size(); xx++) {
			if((maptemp_local->points[i].curvature  < pc_in_angle[xx] + 0.03) &&
			   (maptemp_local->points[i].curvature  > pc_in_angle[xx] - 0.03)){
				maptemp_local->points[i].beam = xx;
				
				PointTypeSm a;
				a = maptemp_local->points[i];
				//maptemp_beam_iso.points.push_back(a);
				//快速分割用
				fast_seg[xx][floor(cur_angle + 180)].points.push_back(a);
				break;
			}
		}
	}
	//todo 如何设计数据结构,更快的比较
	int cur_ring = 0;
	pcl::PointCloud<PointTypeSm> filtered;
	for(int i = 0; i < fast_seg.size(); i++){
		for (int j = 0; j < fast_seg[i].size(); ++j) {
			for (int k = 0; k < fast_seg[i][j].size(); ++k) {
				for (int l = 0; l < fast_seg_ring[i][j].size(); ++l) {
					//0.09
					if(fabs(fast_seg_ring[i][j].points[l].pctime - fast_seg[i][j].points[k].pctime)<0.05){
						//环上的点比现在的远
						if(fast_seg_ring[i][j].points[l].range > fast_seg[i][j].points[k].range){
							//单帧debug用
					/*		map_reduced.push_back(fast_seg[i][j].points[k]);
							filter_index.push_back(fast_seg[i][j].points[k].NeighborPicked);*/
							map_point_save[fast_seg[i][j].points[k].NeighborPicked] = false;
						}
					}
				}
			}
		}
	}
	map_reduced.width = 1;
	map_reduced.height = map_reduced.points.size();
	maptemp_beam_iso.width = 1;
	maptemp_beam_iso.height = maptemp_beam_iso.points.size();
	
}
//4
void FP_PC_viewing::realmap() {
	for (int i = 0; i < maptemp->size(); ++i) {
		if(map_point_save[i]){
			realmap_save.push_back(maptemp->points[i]);
		}
	}
	realmap_save.width = 1;
	realmap_save.height = realmap_save.points.size();
}

void FP_PC_viewing::clear() {
	map_reduced.clear();
	fast_seg.clear();
	fast_seg.resize(N_SCAN);
	for (int i = 0; i < fast_seg.size(); ++i) {
		fast_seg[i].resize(361);
	}
	fast_seg_ring.clear();
	fast_seg_ring.resize(N_SCAN);
	for (int i = 0; i < fast_seg_ring.size(); ++i) {
		fast_seg_ring[i].resize(361);
	}
}
