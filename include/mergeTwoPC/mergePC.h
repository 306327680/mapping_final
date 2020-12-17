//
// Created by echo on 2020/9/16.
//

#ifndef PCD_COMPARE_MERGEPC_H
#define PCD_COMPARE_MERGEPC_H


#include "tools/util.h"
#include <pcl/point_cloud.h>			/* pcl::PointCloud */
#include <pcl/point_types.h>			/* pcl::PointXYZ */
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

class mergePC {
public:
	mergePC(){};
	VLPPointCloud process(VLPPointCloud left, VLPPointCloud right);
	void readYaml(std::string path);
	
	std::vector<Eigen::Matrix4d> TBL;
private:
	VLPPointCloud left;
	VLPPointCloud right;

};


#endif //PCD_COMPARE_MERGEPC_H
