//
// Created by echo on 2020/4/26.
//

#include "GPS_loop_mapping.h"

void
GPS_loop_mapping::GPSandPose(std::string lidar_pose, std::string gps_constraint, Eigen::Isometry3d extrinsic_matrix) {
	ceres::examples::MapOfPoses poses;//这个项目中定义的map pose
	ceres::examples::VectorOfConstraints constraints;//这个项目中定义的 约束向量
	ceres::examples::ReadG2oFile(lidar_pose, &poses, &constraints);//读取g2o文件
	
	ceres::Problem problem;
	BuildOptimizationProblem(constraints, &poses, &problem);
	SolveOptimizationProblem(&problem);
	OutputPoses("poses_optimized.txt", poses);
}
