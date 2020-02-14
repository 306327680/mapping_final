//
// Created by echo on 2020/1/17.
//用于标定LiDAR-GNSS 的外参 还有其他6DOF-6DOF 包含两个外参的情况
//

#ifndef PCD_COMPARE_CALIBRATION6DOF_H
#define PCD_COMPARE_CALIBRATION6DOF_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <ceres/ceres.h>
//#include <pcl/registration/ndt.h>      				//NDT(正态分布)配准类头文件
//#include <pcl/filters/approximate_voxel_grid.h>     //滤波类头文件  （使用体素网格过滤器处理的效果比较好）
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/console/parse.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/filters/extract_indices.h>

class Calibration6DOF {
public:
	//构建 ceres的 误差函数
	struct Calibration
	{//输入的变量是两次的增量, 优化量是外参
		Calibration(Eigen::Matrix4d T_g0_g1, Eigen::Matrix4d T_l0_l1) : T_g0_g1(T_g0_g1), T_l0_l1(T_l0_l1) {
		}
		
		template <typename T>
		bool operator()(const T *q,  T *residual) const //q是 double[]
		{
			//todo 这里的旋转平移的公式不多对,需要改成欧拉角的
			Eigen::Matrix<T, 4, 4> T_w_l0,T_w_g1_est,T_w_g1,error_term,isometry,translate;
			
			cv::Mat R_vec = (cv::Mat_<T>(3,1) << q[0],q[1],q[2]);//数组转cv向量
			cv::Mat R_cvest;
			Rodrigues(R_vec,R_cvest);//罗德里格斯公式，旋转向量转旋转矩阵
			
			Eigen::Matrix<T,3,3> R_est;
			cv2eigen(R_cvest,R_est);//cv矩阵转eigen矩阵
			Eigen::Vector3d t_est;//(q[3],q[4],q[5]);
			cv::Mat t_vec = (cv::Mat_<T>(3,1) << q[3],q[4],q[5]);
			cv2eigen(t_vec,t_est);
			
			Eigen::Isometry3d T_i(R_est);//构造变换矩阵与输出
			T_i.pretranslate(t_est); //pretranslate 应该就是加上
			T_w_l0 = T_i.matrix(); //1.赋值给初始雷达的位姿
			//2.gps的预测
			T_w_g1_est = (T_i*T_l0_l1*T_i.inverse()).matrix();
			T_w_g1 = T_w_g1;
			//两个结果的误差
			error_term = T_w_g1_est*T_w_g1.inverse();
			//残差为点点距离
			residual[0] = q[0];
			//residual[0] = ceres::sqrt(error_term(0,3)*error_term(0,3)+error_term(1,3)*error_term(1,3)+error_term(2,3)*error_term(2,3));
			return true;
		}
		
		static ceres::CostFunction *Create(const Eigen::Matrix4d origin_pt, const Eigen::Matrix4d target_pt){
			return (new ceres::AutoDiffCostFunction<Calibration, 1, 6>(new Calibration(origin_pt, target_pt))); //残差设置为1维,外参设为6维
		}
		
		Eigen::Matrix4d T_g0_g1; //T_w_G1_est 的RT
		Eigen::Matrix4d T_l0_l1; //T_w_G1 的 RT
	};
	Calibration6DOF(){};
	//1. 输入: 雷达和INS时间戳对齐后的位姿 输出:(1). 雷达到gnss的外参  感觉一个就行
	void CalibrateGNSSLiDAR(std::vector<Eigen::Matrix4d> gps_poses,std::vector<Eigen::Matrix4d> LiDAR_poses,
			Eigen::Isometry3d & T_lidar2INS);
	//2. 变量
	double extrinsic_param[6]={0,0,0,0,0,0};
private:
	Eigen::Isometry3d T_w_G1,T_w_G0,T_G0_L0,T_L0_L1,T_G0_G1,T_w_G1_est;
};


#endif //PCD_COMPARE_CALIBRATION6DOF_H
