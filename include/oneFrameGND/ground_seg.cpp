//
// Created by echo on 2019/8/28.
//

#include "ground_seg.h"
//1.高度过滤
void groundSeg::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
						   const pcl::PointCloud<pcl::PointXYZI>::Ptr out) {
	pcl::ExtractIndices<pcl::PointXYZI> cliper;
	
	cliper.setInputCloud(in);
	pcl::PointIndices indices;

	for (size_t i = 0; i < in->points.size(); i++)
	{
		if (in->points[i].z > clip_height)
		{
			indices.indices.push_back(i);
		}
	}
	cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	cliper.setNegative(true); //ture to remove the indices
	cliper.filter(*out);
}

void groundSeg::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
								const pcl::PointCloud<pcl::PointXYZI>::Ptr out) {
	pcl::ExtractIndices<pcl::PointXYZI> cliper;
	cliper.setInputCloud(in);
	pcl::PointIndices indices;

	for (size_t i = 0; i < in->points.size(); i++)
	{
		double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);
		
		if (distance < min_distance)
		{
			indices.indices.push_back(i);
		}
	}
	cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	cliper.setNegative(true); //ture to remove the indices
	cliper.filter(*out);
}

pcl::PointCloud<pcl::PointXYZI> groundSeg::point_cb(pcl::PointCloud<pcl::PointXYZI> point_in) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	*current_pc_ptr = point_in;
	pcl::PointCloud<pcl::PointXYZI> pcout;
//1. 去掉高的点
	clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr);
	pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZI>);
//2.去掉车身周围的点
	remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close);
	
	PointCloudXYZIRTColor organized_points;
	std::vector<pcl::PointIndices> radial_division_indices;
	std::vector<pcl::PointIndices> closest_indices;
	std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;
	
	radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);
//3.radius表示点到lidar的水平距离(半径)，即： theta是点相对于车头正方向(即x方向)的夹角，计算公式为：
	XYZI_to_RTZColor(remove_close, organized_points,
					 radial_division_indices, radial_ordered_clouds);
	pcl::PointIndices ground_indices, no_ground_indices;
//4.判断射线中前后两点的坡度是否大于我们事先设定的坡度阈值，从而判断点是否为地面点。代码如下：
	classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	
	pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
	extract_ground.setInputCloud(remove_close);
	extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
	extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
	extract_ground.filter(*ground_cloud_ptr);
	extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
	extract_ground.filter(*no_ground_cloud_ptr);

	for (int i = 0; i < ground_cloud_ptr->size(); ++i) {
		pcl::PointXYZI temp;
		temp.x = ground_cloud_ptr->points[i].x;
		temp.y = ground_cloud_ptr->points[i].y;
		temp.z = ground_cloud_ptr->points[i].z;
		temp.intensity = ground_cloud_ptr->points[i].intensity;
		pcout.push_back(temp);
	}
/*	for (int i = 0; i < no_ground_cloud_ptr->size(); ++i) {
		pcl::PointXYZI temp;
		temp.x = no_ground_cloud_ptr->points[i].x;
		temp.y = no_ground_cloud_ptr->points[i].y;
		temp.z = no_ground_cloud_ptr->points[i].z;
		temp.intensity = 100;
		pcout.push_back(temp);
	}*/
	return pcout;
	/*writer_pcd.write("ground_cloud_ptr.pcd",*ground_cloud_ptr);
	writer_pcd.write("no_ground_cloud_ptr.pcd",*no_ground_cloud_ptr);*/
	
}

void groundSeg::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
								 groundSeg::PointCloudXYZIRTColor &out_organized_points,
								 std::vector<pcl::PointIndices> &out_radial_divided_indices,
								 std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds) {
	out_organized_points.resize(in_cloud->points.size());
	out_radial_divided_indices.clear();
	out_radial_divided_indices.resize(radial_dividers_num_);
	out_radial_ordered_clouds.resize(radial_dividers_num_);
	
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		PointXYZIRTColor new_point;
		auto radius = (float)sqrt(
				in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);//radius表示点到lidar的水平距离(半径)，即：
		auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;//theta是点相对于车头正方向(即x方向)的夹角
		if (theta < 0)
		{
			theta += 360;
		}
		//角度的微分
		auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
		//半径的微分
		auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));
		
		new_point.point = in_cloud->points[i];
		new_point.radius = radius;
		new_point.theta = theta;
		new_point.radial_div = radial_div;
		new_point.concentric_div = concentric_div;
		new_point.original_index = i;
		
		out_organized_points[i] = new_point;
		
		//radial divisions更加角度的微分组织射线
		//todo 这里有可能有问题
		out_radial_divided_indices[radial_div].indices.push_back(i);
		
		out_radial_ordered_clouds[radial_div].push_back(new_point);
		
	} //end for
	
	//将同一根射线上的点按照半径（距离）排序

	for (size_t i = 0; i < radial_dividers_num_; i++)
	{
		std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
				  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
	}
	
}

void groundSeg::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
							pcl::PointIndices &out_ground_indices, pcl::PointIndices &out_no_ground_indices) {
	out_ground_indices.indices.clear();
	out_no_ground_indices.indices.clear();

	for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
	{
		float prev_radius = 0.f;
		float prev_height = -SENSOR_HEIGHT;
		bool prev_ground = false;
		bool current_ground = false;
		for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
		{
			float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
			float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
			float current_height = in_radial_ordered_clouds[i][j].point.z;
			float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;
			
			//for points which are very close causing the height threshold to be tiny, set a minimum value
			if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
			{
				height_threshold = min_height_threshold_;
			}
			
			//check current point height against the LOCAL threshold (previous point)
			if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
			{
				//Check again using general geometry (radius from origin) if previous points wasn't ground
				if (!prev_ground)
				{
					if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
					{
						current_ground = true;
					}
					else
					{
						current_ground = false;
					}
				}
				else
				{
					current_ground = true;
				}
			}
			else
			{
				//check if previous point is too far from previous one, if so classify again
				if (points_distance > reclass_distance_threshold_ &&
					(current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
				{
					current_ground = true;
				}
				else
				{
					current_ground = false;
				}
			}
			
			if (current_ground)
			{
				out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
				prev_ground = true;
			}
			else
			{
				out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
				prev_ground = false;
			}
			
			prev_radius = in_radial_ordered_clouds[i][j].radius;
			prev_height = in_radial_ordered_clouds[i][j].point.z;
		}
	}
}
