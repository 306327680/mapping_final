//
// Created by echo on 2020/1/17.
//

#include "Calibration6DOF.h"

void Calibration6DOF::CalibrateGNSSLiDAR(std::vector<Eigen::Matrix4d> gps_poses, std::vector<Eigen::Matrix4d> LiDAR_poses,
									Eigen::Isometry3d &T_lidar2INS ) {
 
	ceres::Problem calibration;//1.0 设定problem
	ceres::LossFunction *loss_function1 = new ceres::CauchyLoss(0.1); //2.1 设定 loss function
	
	Eigen::Matrix4d gps_inc,Lidar_inc;
	for (int i = 0; i < gps_poses.size()-1; ++i) {
		//计算增量
		gps_inc = gps_poses[i].inverse()* gps_poses[i+1];
		Lidar_inc = LiDAR_poses[i].inverse()*LiDAR_poses[i+1];
		//把增量放进去优化
		ceres::CostFunction* cost_function =  Calibration::Create(gps_inc,Lidar_inc); //只有这里用到了自己构造的结构体 在结构体里面确定 error 和 jacobian
		calibration.AddResidualBlock(cost_function,
								     loss_function1 /* CauchyLoss loss */,
								  extrinsic_param);
	}
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &calibration, &summary);
	std::cout << summary.FullReport() << "\n";
	
	cv::Mat R_vec = (cv::Mat_<double>(3,1) << extrinsic_param[0],extrinsic_param[1],extrinsic_param[2]);//数组转cv向量
	cv::Mat R_cvest;
	Rodrigues(R_vec,R_cvest);//罗德里格斯公式，旋转向量转旋转矩阵
	Eigen::Matrix<double,3,3> R_est;
	cv2eigen(R_cvest,R_est);//cv矩阵转eigen矩阵
	Eigen::Vector3d t_est(extrinsic_param[3],extrinsic_param[4],extrinsic_param[5]);
	Eigen::Isometry3d T_i(R_est);//构造变换矩阵与输出
	T_i.pretranslate(t_est);
	std::cout<<"T increase \n"<<T_i.matrix()<<std::endl;
}
