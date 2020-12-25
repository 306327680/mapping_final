//
// Created by echo on 2020/12/9.
// 1.用途: 在2d中对地图的边界进行确认 用来切换惯导定位和LiDAR的定位. ok
// 2.用途: 在2d中对object 进行tracking 用来去除动态物体
// 3.用途: 用ground map 把这个投影到png类似的图片中进行辅助标注 ok
// 4.自动对地面的东西进行标注 normal 啥的
// 5.读取

#ifndef PCD_COMPARE_POINT_TO_GRIDMAP_H
#define PCD_COMPARE_POINT_TO_GRIDMAP_H
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/features/normal_3d.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
class point_to_gridmap {
public:
	point_to_gridmap(pcl::PointCloud<pcl::PointXYZI> pc){currentPC = pc;};
	//重置grid
	void initGrid(nav_msgs::OccupancyGridPtr grid);
	//0.计算surface normal
	void calcSurfaceNormals(pcl::PointCloud<pcl::PointXYZI> cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
	//1.计算点云大小
	void calcPcBoundary(double & xMax  ,double & yMax  ,double & xMin ,double & yMin );
	//2. 计算grid map
	void populateMap(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::vector<int> &map,double xMax, double yMax, double xMin, double yMin,double cellResolution, int xCells, int yCells);
	//3. 得到grid map
	void genOccupancyGrid(std::vector<signed char> &ocGrid, std::vector<int> &countGrid, int size , int xcells);
	//4. 更新栅格
	void updateGrid(nav_msgs::OccupancyGridPtr grid, double cellRes, int xCells, int yCells,double originX, double originY, std::vector<signed char> *ocGrid);
	//5. 保存栅格图
	void saveGridasPNG(nav_msgs::OccupancyGridPtr grid, std::string image_path);
	//a.1 反射率得到ground map
	void groundVoxelMap( nav_msgs::OccupancyGridPtr grid);
	//a.2 生成mat
	void createAlphaMat(cv::Mat &mat);
	//a.3 int转rgb
	cv::Vec3b int2RGB(int input);
	//变量
	pcl::PointCloud<pcl::PointXYZI> currentPC;
	//b.1 读取GridMap
	void readGridMap(std::string mapPath);
private:
	double cellResolution = 0.05;
};


#endif //PCD_COMPARE_POINT_TO_GRIDMAP_H
