//
// Created by echo on 2019/8/28.
//

#ifndef PORT_SIMULATOR_GROUND_SEG_H
#define PORT_SIMULATOR_GROUND_SEG_H




#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>



#define CLIP_HEIGHT 0.2 //截取掉高于雷达自身0.2米的点
#define MIN_DISTANCE 2.4
#define RADIAL_DIVIDER_ANGLE 0.18// 我们对360度进行微分，分成若干等份，每一份的角度为0.18度，这个微分的等份近似的可以看作一条射线，
//如下图所示,图中是一个激光雷达的纵截面的示意图，雷达由下至上分布多个激光器，发出如图所示的放射状激光束，这些激光束在平地上即表现为，图中的水平线即为一条射线：
#define SENSOR_HEIGHT 1.78

#define concentric_divider_distance_ 0.1 //0.1 meters default
#define min_height_threshold_ 0.05
#define local_max_slope_ 8   //max slope of the ground between points, degree
#define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2

class groundSeg
{

private:
	
	struct PointXYZIRTColor
	{
		pcl::PointXYZI point;
		
		float radius; //cylindric coords on XY Plane
		float theta;  //angle deg on XY plane
		
		size_t radial_div;     //index of the radial divsion to which this point belongs to
		size_t concentric_div; //index of the concentric division to which this points belongs to
		size_t original_index; //index of this point in the source pointcloud
	};
	typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;
	
	size_t radial_dividers_num_;
	size_t concentric_dividers_num_;
	pcl::PCDWriter writer_pcd;

	
	void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
	
	void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
	
	void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
						  PointCloudXYZIRTColor &out_organized_points,
						  std::vector<pcl::PointIndices> &out_radial_divided_indices,
						  std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);
	
	void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
					 pcl::PointIndices &out_ground_indices,
					 pcl::PointIndices &out_no_ground_indices);
	

public:
	groundSeg(){};
	~groundSeg(){};
	pcl::PointCloud<pcl::PointXYZI> point_cb(pcl::PointCloud<pcl::PointXYZI> point_in);
};

#endif //PORT_SIMULATOR_GROUND_SEG_H

