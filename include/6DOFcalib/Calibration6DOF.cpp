//
// Created by echo on 2020/1/17.
//
//todo 现在只是测试版,回头应该要有严格的时间戳和数量对齐的操作

#include "Calibration6DOF.h"

void Calibration6DOF::CalibrateGNSSLiDAR(std::vector<Eigen::Matrix4d> gps_poses, std::vector<Eigen::Matrix4d> LiDAR_poses,
									Eigen::Isometry3d &T_lidar2INS ) {
 
	ceres::Problem calibration;//1.0 设定problem
	ceres::LossFunction *loss_function1 = new ceres::CauchyLoss(0.1); //2.1 设定 loss function
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization(); //四元数方式的优化
    calibration.AddParameterBlock(para_q, 4, q_parameterization);
    calibration.AddParameterBlock(para_t, 3);

	Eigen::Isometry3d gps_inc,Lidar_inc,gps_abs,Lidar_abs;
	//todo 这里为啥输出的不一样
    for (int j = 0; j < LiDAR_poses.size(); ++j) {
        //std::cout<<LiDAR_poses[j].matrix()<<std::endl;
    }
    if (LiDAR_poses.size() ==  gps_poses.size()){
        std::cout<< "same size "<<std::endl;
    } else{
        std::cout<<LiDAR_poses.size()<< " not same size "<<gps_poses.size()<<std::endl;
    }
	for (int i = 0; i < gps_poses.size(); ++i) {
		//计算增量
		gps_inc = gps_poses[i].inverse()* gps_poses[i+1];
		Lidar_inc = LiDAR_poses[i].inverse()*LiDAR_poses[i+1];
        gps_abs = gps_poses[0].inverse()* gps_poses[i+1];
        Lidar_abs = LiDAR_poses[0].inverse()*LiDAR_poses[i+1];

        Eigen::Matrix<double,7,1> gps_inc_qt,Lidar_inc_qt,gps_abs_qt,Lidar_abs_qt;
        Eigen::Quaterniond quaternion(gps_inc.rotation().matrix());
        gps_inc_qt[0] = quaternion.x();
        gps_inc_qt[1] = quaternion.y();
        gps_inc_qt[2] = quaternion.z();
        gps_inc_qt[3] = quaternion.w();
        gps_inc_qt[4] = gps_inc(0,3);
        gps_inc_qt[5] = gps_inc(1,3);
        gps_inc_qt[6] = gps_inc(2,3);
        quaternion =Eigen::Quaterniond(Lidar_inc.rotation().matrix());
        Lidar_inc_qt[0] = quaternion.x();
        Lidar_inc_qt[1] = quaternion.y();
        Lidar_inc_qt[2] = quaternion.z();
        Lidar_inc_qt[3] = quaternion.w();
        Lidar_inc_qt[4] = Lidar_inc(0,3);
        Lidar_inc_qt[5] = Lidar_inc(1,3);
        Lidar_inc_qt[6] = Lidar_inc(2,3);
        quaternion =Eigen::Quaterniond(gps_abs.rotation().matrix());
        gps_abs_qt[0] = quaternion.x();
        gps_abs_qt[1] = quaternion.y();
        gps_abs_qt[2] = quaternion.z();
        gps_abs_qt[3] = quaternion.w();
        gps_abs_qt[4] = gps_abs(0,3);
        gps_abs_qt[5] = gps_abs(1,3);
        gps_abs_qt[6] = gps_abs(2,3);
        quaternion =Eigen::Quaterniond(Lidar_abs.rotation().matrix());
        Lidar_abs_qt[0] = quaternion.x();
        Lidar_abs_qt[1] = quaternion.y();
        Lidar_abs_qt[2] = quaternion.z();
        Lidar_abs_qt[3] = quaternion.w();
        Lidar_abs_qt[4] = Lidar_abs(0,3);
        Lidar_abs_qt[5] = Lidar_abs(1,3);
        Lidar_abs_qt[6] = Lidar_abs(2,3);

		//把增量放进去优化
		ceres::CostFunction* cost_function =  Calibration::Create(gps_inc_qt,Lidar_inc_qt,gps_abs_qt,Lidar_abs_qt); //只有这里用到了自己构造的结构体 在结构体里面确定 error 和 jacobian
        calibration.AddResidualBlock(cost_function, loss_function1, para_q, para_t);//这里分成两个了
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 20000;
/*    options.function_tolerance = 1e-20;
    options.min_line_search_step_size = 1e-20;*/

	ceres::Solver::Summary summary;
	ceres::Solve(options, &calibration, &summary);
	std::cout << summary.FullReport() << "\n";
    std::cout << para_q[0] <<" "<< para_q[1] <<" "<< para_q[2] <<" "<< para_q[3]<< std::endl;
    std::cout << para_t[0] <<" "<< para_t[1] <<" "<< para_t[2]  << std::endl;

	//处理结果
    Eigen::Quaternion<double> q_extrinsic{para_q[3], para_q[0], para_q[1], para_q[2]};
    Eigen::Vector3d t_extrinsic{ para_t[0],  para_t[1],  para_t[2]};
    T_lidar2INS.setIdentity();
    T_lidar2INS.rotate(q_extrinsic);
    T_lidar2INS.pretranslate(t_extrinsic);

}
