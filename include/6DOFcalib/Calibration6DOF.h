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
		Calibration(Eigen::Matrix<double,7,1>  T_g0_g1, Eigen::Matrix<double,7,1>  T_l0_l1,
		        const Eigen::Matrix<double,7,1>  gps_abs, const Eigen::Matrix<double,7,1>  lidar_abs)
		        : T_g0_g1(T_g0_g1), T_l0_l1(T_l0_l1) ,T_g0_gn(gps_abs),T_l0_ln(lidar_abs){//相邻的增量 和 绝对位置
		}

		template <typename T>
		bool operator()(const T *q, const T *t , T *residual) const //q是 double[]
		{
            Eigen::Quaternion<T> q_extrinsic{q[3], q[0], q[1], q[2]};
            Eigen::Matrix<T, 3, 1> t_extrinsic{ t[0],  t[1],  t[2]};

            Eigen::Quaternion<T> q_g{T(T_g0_g1[3]), T(T_g0_g1[0]), T(T_g0_g1[1]), T(T_g0_g1[2])};//w ,x ,y ,z
            Eigen::Quaternion<T> q_l{T(T_l0_l1[3]), T(T_l0_l1[0]), T(T_l0_l1[1]), T(T_l0_l1[2])};
            Eigen::Quaternion<T> q_g_n{T(T_g0_gn[3]), T(T_g0_gn[0]), T(T_g0_gn[1]), T(T_g0_gn[2])};//w ,x ,y ,z
            Eigen::Quaternion<T> q_l_n{T(T_l0_ln[3]), T(T_l0_ln[0]), T(T_l0_ln[1]), T(T_l0_ln[2])};

            Eigen::Matrix<T, 3, 1> t_g{ T(T_g0_g1[4]),  T(T_g0_g1[5]),  T(T_g0_g1[6])};
            Eigen::Matrix<T, 3, 1> t_l{ T(T_l0_l1[4]),  T(T_l0_l1[5]),  T(T_l0_l1[6])};
            Eigen::Matrix<T, 3, 1> t_g_n{ T(T_g0_gn[4]),  T(T_g0_gn[5]),  T(T_g0_gn[6])};
            Eigen::Matrix<T, 3, 1> t_l_n{ T(T_l0_ln[4]),  T(T_l0_ln[5]),  T(T_l0_ln[6])};

            Eigen::Matrix<T, 3, 1> T_w_g1_zero, T_w_g1_x, T_w_g1_y, T_w_g1_z;
            Eigen::Matrix<T, 3, 1> T_w_g1_est_zero, T_w_g1_est_x, T_w_g1_est_y, T_w_g1_est_z;
            //设置4个向量
            Eigen::Matrix<T, 3, 1> zero{T(0), T(0), T(0)};
            Eigen::Matrix<T, 3, 1> x{T(1), T(0), T(0)};
            Eigen::Matrix<T, 3, 1> y{T(0), T(1), T(0)};
            Eigen::Matrix<T, 3, 1> z{T(0), T(0), T(1)};
            //误差
            Eigen::Matrix<T, 3, 1> e_zero{T(0), T(0), T(0)};
            Eigen::Matrix<T, 3, 1> e_x{T(1), T(0), T(0)};
            Eigen::Matrix<T, 3, 1> e_y{T(0), T(1), T(0)};
            Eigen::Matrix<T, 3, 1> e_z{T(0), T(0), T(1)};

            //变换1 相邻两镇之间的 误差
            T_w_g1_zero = q_g * zero + t_g;
            T_w_g1_x = q_g * x + t_g;
            T_w_g1_y = q_g * y + t_g;
            T_w_g1_z = q_g * z + t_g;
            //变换2

            T_w_g1_est_zero =  q_extrinsic * zero + t_extrinsic;
            T_w_g1_est_x =  q_extrinsic * x + t_extrinsic;
            T_w_g1_est_y =  q_extrinsic * y + t_extrinsic;
            T_w_g1_est_z =  q_extrinsic * z + t_extrinsic;

            T_w_g1_est_zero =  q_l * T_w_g1_est_zero + t_l;
            T_w_g1_est_x =  q_l * T_w_g1_est_x + t_l;
            T_w_g1_est_y =  q_l * T_w_g1_est_y + t_l;
            T_w_g1_est_z =  q_l * T_w_g1_est_z + t_l;

            T_w_g1_est_zero =  q_extrinsic.inverse() * T_w_g1_est_zero - t_extrinsic;
            T_w_g1_est_x =  q_extrinsic.inverse() * T_w_g1_est_x - t_extrinsic;
            T_w_g1_est_y =  q_extrinsic.inverse() * T_w_g1_est_y - t_extrinsic;
            T_w_g1_est_z =  q_extrinsic.inverse() * T_w_g1_est_z - t_extrinsic;
            //error
            e_zero = T_w_g1_est_zero - T_w_g1_zero;
            e_x = T_w_g1_est_x - T_w_g1_x;
            e_y = T_w_g1_est_y - T_w_g1_y;
            e_z = T_w_g1_est_z - T_w_g1_z;

            residual[0] = ceres::sqrt(e_zero(0,0)*e_zero(0,0) + e_zero(1,0)*e_zero(1,0) + e_zero(2,0)*e_zero(2,0));
            residual[1] = ceres::sqrt(e_x(0,0)*e_x(0,0) + e_x(1,0)*e_x(1,0) + e_x(2,0)*e_x(2,0));
            residual[2] = ceres::sqrt(e_y(0,0)*e_y(0,0) + e_y(1,0)*e_y(1,0) + e_y(2,0)*e_y(2,0));
            residual[3] = ceres::sqrt(e_z(0,0)*e_z(0,0) + e_z(1,0)*e_z(1,0) + e_z(2,0)*e_z(2,0));
            //第二部分残差 整体的残差
            //变换1
            T_w_g1_zero = q_g_n * zero + t_g_n;
            T_w_g1_x    = q_g_n * x + t_g_n;
            T_w_g1_y    = q_g_n * y + t_g_n;
            T_w_g1_z    = q_g_n * z + t_g_n;
            //变换2

            T_w_g1_est_zero =  q_extrinsic * zero + t_extrinsic;
            T_w_g1_est_x    =  q_extrinsic * x + t_extrinsic;
            T_w_g1_est_y    =  q_extrinsic * y + t_extrinsic;
            T_w_g1_est_z    =  q_extrinsic * z + t_extrinsic;

            T_w_g1_est_zero =  q_l_n * T_w_g1_est_zero + t_l_n;
            T_w_g1_est_x    =  q_l_n * T_w_g1_est_x + t_l_n;
            T_w_g1_est_y    =  q_l_n * T_w_g1_est_y + t_l_n;
            T_w_g1_est_z    =  q_l_n * T_w_g1_est_z + t_l_n;

            T_w_g1_est_zero =  q_extrinsic.inverse() * T_w_g1_est_zero - t_extrinsic;
            T_w_g1_est_x =  q_extrinsic.inverse() * T_w_g1_est_x - t_extrinsic;
            T_w_g1_est_y =  q_extrinsic.inverse() * T_w_g1_est_y - t_extrinsic;
            T_w_g1_est_z =  q_extrinsic.inverse() * T_w_g1_est_z - t_extrinsic;
            //error
            e_zero  = T_w_g1_est_zero - T_w_g1_zero;
            e_x     = T_w_g1_est_x    - T_w_g1_x;
            e_y     = T_w_g1_est_y    - T_w_g1_y;
            e_z     = T_w_g1_est_z    - T_w_g1_z;

            residual[4] = ceres::sqrt(e_zero(0,0)*e_zero(0,0) + e_zero(1,0)*e_zero(1,0) + e_zero(2,0)*e_zero(2,0));
            residual[5] = ceres::sqrt(e_x(0,0)*e_x(0,0) + e_x(1,0)*e_x(1,0) + e_x(2,0)*e_x(2,0));
            residual[6] = ceres::sqrt(e_y(0,0)*e_y(0,0) + e_y(1,0)*e_y(1,0) + e_y(2,0)*e_y(2,0));
            residual[7] = ceres::sqrt(e_z(0,0)*e_z(0,0) + e_z(1,0)*e_z(1,0) + e_z(2,0)*e_z(2,0));

			return true;
		}
		
		static ceres::CostFunction *Create(const Eigen::Matrix<double,7,1>  origin_pt, const Eigen::Matrix<double,7,1>  target_pt,
                                           const Eigen::Matrix<double,7,1>  gps_abs, const Eigen::Matrix<double,7,1>  lidar_abs){
			return (new ceres::AutoDiffCostFunction<Calibration, 8, 4, 3>(new Calibration(origin_pt, target_pt, gps_abs, lidar_abs))); //残差设置为4维,外参设为4元数和t
		}

		Eigen::Matrix<double,7,1> T_g0_g1;
		Eigen::Matrix<double,7,1> T_l0_l1;
        Eigen::Matrix<double,7,1> T_g0_gn;
        Eigen::Matrix<double,7,1> T_l0_ln;

	};
	Calibration6DOF(){};
	//1. 输入: 雷达和INS时间戳对齐后的位姿 输出:(1). 雷达到gnss的外参  感觉一个就行
	void CalibrateGNSSLiDAR(std::vector<Eigen::Matrix4d> gps_poses,std::vector<Eigen::Matrix4d> LiDAR_poses,
			Eigen::Isometry3d & T_lidar2INS);
	//2. 外参
	double extrinsic_param[7]={0,0,0,0,0,0};
	//3. aloam的外参
    double para_q[4] = {0, 0, 0, 1};
    double para_t[3] = {0, 0, 0};
private:
	Eigen::Isometry3d T_w_G1,T_w_G0,T_G0_L0,T_L0_L1,T_G0_G1,T_w_G1_est;
};


#endif //PCD_COMPARE_CALIBRATION6DOF_H
//
/*
T_w_g1_est_zero =  q_l_n * zero + t_l_n;
T_w_g1_est_x    =  q_l_n * x + t_l_n;
T_w_g1_est_y    =  q_l_n * y + t_l_n;
T_w_g1_est_z    =  q_l_n * z + t_l_n;

T_w_g1_est_zero =  q_extrinsic * T_w_g1_est_zero + t_extrinsic;
T_w_g1_est_x    =  q_extrinsic * T_w_g1_est_x + t_extrinsic;
T_w_g1_est_y    =  q_extrinsic * T_w_g1_est_y + t_extrinsic;
T_w_g1_est_z    =  q_extrinsic * T_w_g1_est_z + t_extrinsic;*/
