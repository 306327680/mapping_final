//
// Created by echo on 2020/1/17.
//
//todo 现在只是测试版,回头应该要有严格的时间戳和数量对齐的操作

#include "Calibration6DOF.h"
//优化的方法直接标定3DOF-6DOF外参
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

void
Calibration6DOF::CalibrateGNSSLiDARICP(std::vector<Eigen::Matrix4d> gps_poses, std::vector<Eigen::Matrix4d> LiDAR_poses,
									   Eigen::Isometry3d &T_lidar2INS, Eigen::Vector3d arm ) {

	std::vector<cv::Point3f> pts1;
	std::vector<cv::Point3f> pts2;
	for (int j = 0; j < gps_poses.size(); ++j) {
		cv::Point3f temp;
		temp.x = gps_poses[j](0,3);
		temp.y = gps_poses[j](1,3);
		temp.z = gps_poses[j](2,3);
		pts2.push_back(temp);
		temp.x = LiDAR_poses[j](0,3);
		temp.y = LiDAR_poses[j](1,3);
		temp.z = LiDAR_poses[j](2,3);
		pts1.push_back(temp);
	}
	cv::Mat R;
	cv::Mat t;
	Eigen::Isometry3d se3;
	
	cv::Point3f p1, p2;     // center of mass
	int N = pts1.size();
	for (int i = 0; i < N; i++) {
		p1 += pts1[i];
		p2 += pts2[i];
	}
	p1 = cv::Point3f(cv::Vec3f(p1) / N);
	p2 = cv::Point3f(cv::Vec3f(p2) / N);
	std::vector<cv::Point3f> q1(N), q2(N); // remove the center
	for (int i = 0; i < N; i++) {
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}
	
	// compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; i++) {
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	Eigen::Matrix3d cc = W.transpose()*W;
	
	Eigen::EigenSolver<Eigen::Matrix3d> es(cc);
/*	std::cout<<"eigenvalue:\n"<<es.eigenvalues()<<"\n"<<std::endl;
	std::cout<<"eigenvector:\n"<<es.eigenvectors()<<"\n"<<std::endl;*/
	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	/*std::cout << "W=\n" << W << std::endl;
	std::cout << "U=\n" << U << std::endl;
	std::cout << "V=\n" << V << std::endl;*/
	
	Eigen::Matrix3d R_ = U * (V.transpose());
	if (R_.determinant() < 0) {
		R_ = -R_;
	}
	Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
	
	// convert to cv::Mat
	R = (cv::Mat_<double>(3, 3) <<
								R_(0, 0), R_(0, 1), R_(0, 2),
			R_(1, 0), R_(1, 1), R_(1, 2),
			R_(2, 0), R_(2, 1), R_(2, 2)
	);
	t = (cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
	se3 = Eigen::Isometry3d::Identity();
	se3.rotate(R_);
	se3.pretranslate(t_);
	T_lidar2INS = se3;
}
