//
// Created by echo on 2020/9/5.
//

#include "DAICP.h"

Eigen::Isometry3d DAICP::solveICP(pcl::PointCloud<pcl::PointXYZI>::Ptr target, pcl::PointCloud<pcl::PointXYZI> source,
								  Eigen::Isometry3d init_pose) {
	//1.转换点云
	pcl::PointCloud<pcl::PointXYZI> tfed_source;
	pcl::transformPointCloud(source, tfed_source, init_pose.matrix());
	T = Eigen::Isometry3d::Identity();
	//2.计算最近的点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	target_ptr = target;
	timeCalcSet("kdtreeLast");
	kdtreeLast->setInputCloud(target_ptr); // time 0.0380341 i+15 52% wholemap 1.13772
	timeUsed();
	//进行10次迭代	//3.构建约束 + LM优化 *******wholemap 30 : 0.0135034
	
	for (int i = 0; i < 30; ++i) {
		//std::cout<<"\n \n current interation: "<<i<<std::endl;
		//solveOneLMNumericDiff(target,source);
		solveOneSVD(target_ptr,source);//更新T
		/*	if(solveOneLM(target,source)){
				continue;
			} else{
				std::cout<<"times: "<<i<<std::endl;
				break;
			}*/
	}
	
	//4.保存result
/*	pcl::PointCloud<pcl::PointXYZI> result;
	pcl::transformPointCloud(source, result, T.matrix());
	pcl::io::savePCDFile("result.pcd",result); //保存最近点*/
	std::cout<<"Final T\n"<<T.matrix()<<std::endl;
	return T;
}

void DAICP::solveOneSVD(pcl::PointCloud<pcl::PointXYZI>::Ptr target, pcl::PointCloud<pcl::PointXYZI> source) {
	pcl::PointXYZI pointSel; // 要搜索的点
	std::vector<int> pointSearchInd;//搜索最近点的id
	std::vector<float> pointSearchSqDis;//最近点的距离
	
	pcl::PointCloud<pcl::PointXYZI> Temp_pc;
	std::vector<cv::Point3f> vp1,vp2;
	pcl::transformPointCloud(source, Temp_pc, T.matrix()); 			//*******把初始点云转换到上次的结果的位置 5.8488e-05
	
	Eigen::Matrix3d information_ = Eigen::Matrix3d::Identity(); //设置information
	
	//for 0.00478317  i+1 65% 的时间 i+5 30% i+15 %14.2 whole map :0.000538211
	//i+15 try2: wholemap 0.000326884 local map:0. 00034949
	//i+1 try2: wholemap  0.00460973 local map:0.00477633
	//用了94%的时间
	timeCalcSet("for");
	for (int i = 0; i < Temp_pc.size(); i = i+1) {
		
		pointSel = Temp_pc[i];
		kdtreeLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis); //	1.294e-06
		
		vp1.push_back(cv::Point3f(pointSel.x,pointSel.y,pointSel.z)) ;			//当前扫描点 8.2e-08
		vp2.push_back(cv::Point3f(target->points[pointSearchInd.back()].x,
								  target->points[pointSearchInd.back()].y,
								  target->points[pointSearchInd.back()].z)) ; //地图点
		
	}
	timeUsed();
	cv::Mat R, t;
	Eigen::Isometry3d se3;
	
	solveOneLMSVD(vp1,vp2,R,t,se3); //4.865e-06
	std::cout<<se3.matrix()<<std::endl;
	T = T*se3.inverse();
}

bool
DAICP::solveOneLMSVD(const std::vector<cv::Point3f> &pts1, const std::vector<cv::Point3f> &pts2, cv::Mat &R, cv::Mat &t,
					 Eigen::Isometry3d &se3) {
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
	std::cout<<"eigenvalue:\n"<<es.eigenvalues()<<"\n"<<std::endl;
	std::cout<<"eigenvector:\n"<<es.eigenvectors()<<"\n"<<std::endl;
	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	std::cout << "W=\n" << W << std::endl;
	std::cout << "U=\n" << U << std::endl;
	std::cout << "V=\n" << V << std::endl;
	
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
/*	std::cout<<"orign:\n"<<R<<"\n"<<t<<std::endl;
	std::cout<<"result: \n"<<se3.matrix()<<std::endl;*/
	
	return false;
}
