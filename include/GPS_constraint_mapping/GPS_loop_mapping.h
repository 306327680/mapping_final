//
// Created by echo on 2020/4/26.
//

#ifndef PCD_COMPARE_GPS_LOOP_MAPPING_H
#define PCD_COMPARE_GPS_LOOP_MAPPING_H


#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <istream>
#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include "g2oIO/PoseGraphIO.h"
#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//下面是ceres自己的东西
namespace ceres {
	namespace examples {
		
		//########读取g2o文件
		// Reads a single pose from the input and inserts it into the map. Returns false
// if there is a duplicate entry.
		template <typename Pose, typename Allocator>
		bool ReadVertex(std::ifstream* infile,
						std::map<int, Pose, std::less<int>, Allocator>* poses) {
			int id;
			Pose pose;
			*infile >> id >> pose;
			
			// Ensure we don't have duplicate poses.
			if (poses->find(id) != poses->end()) {
				LOG(ERROR) << "Duplicate vertex with ID: " << id;
				return false;
			}
			(*poses)[id] = pose;
			
			return true;
		}

// Reads the contraints between two vertices in the pose graph
		template <typename Constraint, typename Allocator>
		void ReadConstraint(std::ifstream* infile,
							std::vector<Constraint, Allocator>* constraints) {
			Constraint constraint;
			*infile >> constraint;
			
			constraints->push_back(constraint);
		}

// Reads a file in the g2o filename format that describes a pose graph
// problem. The g2o format consists of two entries, vertices and constraints.
//
// In 2D, a vertex is defined as follows:
//
// VERTEX_SE2 ID x_meters y_meters yaw_radians
//
// A constraint is defined as follows:
//
// EDGE_SE2 ID_A ID_B A_x_B A_y_B A_yaw_B I_11 I_12 I_13 I_22 I_23 I_33
//
// where I_ij is the (i, j)-th entry of the information matrix for the
// measurement.
//
//
// In 3D, a vertex is defined as follows:
//
// VERTEX_SE3:QUAT ID x y z q_x q_y q_z q_w
//
// where the quaternion is in Hamilton form.
// A constraint is defined as follows:
//
// EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12 I_13 ... I_16 I_22 I_23 ... I_26 ... I_66 // NOLINT
//
// where I_ij is the (i, j)-th entry of the information matrix for the
// measurement. Only the upper-triangular part is stored. The measurement order
// is the delta position followed by the delta orientation.
		template <typename Pose, typename Constraint, typename MapAllocator,typename VectorAllocator>
		
		bool ReadG2oFile(const std::string& filename,
						 std::map<int, Pose, std::less<int>, MapAllocator>* poses,
						 std::vector<Constraint, VectorAllocator>* constraints) {
			CHECK(poses != NULL);
			CHECK(constraints != NULL);
			
			poses->clear();
			constraints->clear();
			
			std::ifstream infile(filename.c_str());
			if (!infile) {
				return false;
			}
			
			std::string data_type;
			while (infile.good()) {
				// Read whether the type is a node or a constraint.
				infile >> data_type;
				if (data_type == Pose::name()) {
					if (!ReadVertex(&infile, poses)) {
						return false;
					}
				} else if (data_type == Constraint::name()) {
					ReadConstraint(&infile, constraints);
				} else {
					LOG(ERROR) << "Unknown data type: " << data_type;
					return false;
				}
				
				// Clear any trailing whitespace from the line.
				infile >> std::ws;
			}
			return true;
		}
		//************ 设定数据格式
		//pq
		struct Pose3d {//pose
			Eigen::Vector3d p;
			Eigen::Quaterniond q;
			// The name of the data type in the g2o file format.
			static std::string name() {
				return "VERTEX_SE3:QUAT";
			}
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
		
		//运算符重载
		inline std::istream& operator>>(std::istream& input, Pose3d& pose) {
			input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >>
				  pose.q.y() >> pose.q.z() >> pose.q.w();
			// Normalize the quaternion to account for precision loss due to serialization.
			pose.q.normalize();
			return input;
		}
		//
		typedef std::map<int, Pose3d, std::less<int>,Eigen::aligned_allocator<std::pair<const int, Pose3d> > > MapOfPoses;
// 两个顶点之间的位姿图约束,约束是指begin到end的
// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
		struct Constraint3d {
			int id_begin;
			int id_end;
			
			// The transformation that represents the pose of the end frame E w.r.t. the
			// begin frame B. In other words, it transforms a vector in the E frame to
			// the B frame.
			Pose3d t_be;//SE3
			
			// The inverse of the covariance matrix for the measurement. The order of the
			// entries are x, y, z, delta orientation.
			Eigen::Matrix<double, 6, 6> information;//信息矩阵
			
			// The name of the data type in the g2o file format. g2o中的数据文件类型的名字
			static std::string name() {
				return "EDGE_SE3:QUAT";
			}
			
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
		//传入t和imformation???
		inline std::istream& operator>>(std::istream& input, Constraint3d& constraint) {
			Pose3d& t_be = constraint.t_be;
			input >> constraint.id_begin >> constraint.id_end >> t_be;
			
			for (int i = 0; i < 6 && input.good(); ++i) {
				for (int j = i; j < 6 && input.good(); ++j) {
					input >> constraint.information(i, j);
					if (i != j) {
						constraint.information(j, i) = constraint.information(i, j);
					}
				}
			}
			return input;
		}
		
		typedef std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d> >
				VectorOfConstraints;
		
		//位姿图的误差项
		class PoseGraph3dErrorTerm {
		public:
			//用来赋值 Create 里面的 放到了这里
			PoseGraph3dErrorTerm(const Pose3d& t_ab_measured,const Eigen::Matrix<double, 6, 6>& sqrt_information)
					: t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}
			
			template <typename T>
			//前四个是 ceres要调整的, 最后一个是残差项,通过调整 前面4个使得residuals_ptr最小
			bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
							const T* const p_b_ptr, const T* const q_b_ptr,
							T* residuals_ptr) const {
				Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_a(p_a_ptr);
				Eigen::Map<const Eigen::Quaternion<T> > q_a(q_a_ptr);
				
				Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_b(p_b_ptr);
				Eigen::Map<const Eigen::Quaternion<T> > q_b(q_b_ptr);
				
				// Compute the relative transformation between the two frames.
				//两针之间的相对旋转
				Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();//四元数的共轭 x,y,z 都负
				Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;
				
				// Represent the displacement between the two frames in the A frame. 两针之间位移
				Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);
				
				// Compute the error between the two orientation estimates. measure和estimate朝向之间的error conjugate 共轭
				Eigen::Quaternion<T> delta_q = t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();
				
				// Compute the residuals.
				// [ position         ]   [ delta_p          ]
				// [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
				//计算残差
				Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
				residuals.template block<3, 1>(0, 0) = T(100000.0)*(p_ab_estimated - t_ab_measured_.p.template cast<T>());//前三位是位移残差
				residuals.template block<3, 1>(3, 0) = T(200000.0) * delta_q.vec();//这三位是旋转残差
				
				// Scale the residuals by the measurement uncertainty.
				residuals.applyOnTheLeft(sqrt_information_.template cast<T>());//测量不确定性去scale这些measurement
				
				return true;
			}
			
			static ceres::CostFunction* Create(
					const Pose3d& t_ab_measured,
					const Eigen::Matrix<double, 6, 6>& sqrt_information) {
				//残差6位 tq tq 输入两个位姿
				return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
			}
			
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		private:
			// The measurement for the position of B relative to A in the A frame.
			const Pose3d t_ab_measured_;
			// The square root of the measurement information matrix.
			const Eigen::Matrix<double, 6, 6> sqrt_information_;
		};
		
		
		//***gps的约束
		//输入: 1.gps测量值 2.当前位姿
		//误差: 当前位姿的杆臂值坐标-gps测量值
		class PoseGraphGPSErrorTerm {
		public:
			//用来赋值 Create 里面的 放到了这里
			PoseGraphGPSErrorTerm(const Eigen::Vector3d& t_ab_measured,const Eigen::Matrix<double, 6, 6>& sqrt_information)
					: t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}
			
			template <typename T>
			//前四个是 ceres要调整的, 最后一个是残差项,通过调整 前面4个使得residuals_ptr最小
			bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
							T* residuals_ptr) const {
				Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_a(p_a_ptr);
				Eigen::Map<const Eigen::Quaternion<T> > q_a(q_a_ptr);
				Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(residuals_ptr);
				//当前的
				residuals(0,0) = p_a(0,0) - t_ab_measured_(0);
				residuals(1,0) = p_a(1,0) - t_ab_measured_(1);
				residuals(2,0) = T(5.0)*(p_a(2,0) - t_ab_measured_(2));
				
				return true;
			}
			
			static ceres::CostFunction* Create(
					const Eigen::Vector3d& t_ab_measured,
					const Eigen::Matrix<double, 6, 6>& sqrt_information) {
				//残差3位 tq   输入1个位姿
				//残差
				return new ceres::AutoDiffCostFunction<PoseGraphGPSErrorTerm, 3, 3, 4>(new PoseGraphGPSErrorTerm(t_ab_measured, sqrt_information));
			}
			
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		private:
			// The measurement for the position of B relative to A in the A frame.
			const Eigen::Vector3d t_ab_measured_;
			// The square root of the measurement information matrix.
			const Eigen::Matrix<double, 6, 6> sqrt_information_;
		};
	}  // namespace examples
}  // namespace ceres



class GPS_loop_mapping {
public:
	GPS_loop_mapping(){};
	//功能1. 把闭环的位姿放进去,所有的,不需要lidar位姿downsample,输入lla的坐标.和激光旋转过去的坐标
	void GPSandPose(std::string lidar_pose,std::string gps_constraint,Eigen::Isometry3d extrinsic_matrix);
 
	template <typename Pose, typename Constraint, typename MapAllocator,typename VectorAllocator>
 	void G2OandPCD(std::string lidar_pose,std::string gps_constraint,
 			std::map<int, Pose, std::less<int>, MapAllocator>* poses,
				   std::vector<Constraint, VectorAllocator>* constraints);//通过闭环的点云构建 poses(顶点)和constraints(边)
	//这个函数是为了找到gps的值和pose的index的关系,之后可以加入方差矩阵传递参数
	void RelationG2OGPS(std::string lidar_pose,std::string gps_constraint,std::vector<std::pair<int,Eigen::Vector3d>>& relation);
	void RelationG2OGPS(std::string lidar_pose,std::string gps_constraint,std::vector<std::pair<int,Eigen::Vector3d>>& relation,std::vector<Eigen::Vector3d> & conv_gps);
// 1.构建优化问题
	void BuildOptimizationProblem(const ceres::examples::VectorOfConstraints& constraints,ceres::examples::MapOfPoses* poses, ceres::Problem* problem) ;
	//2.加入gps约束的问题
	void BuildGPSOptimizationProblem(const ceres::examples::VectorOfConstraints& constraints,
			ceres::examples::MapOfPoses* poses, ceres::Problem* problem,	std::vector<std::pair<int,Eigen::Vector3d>> relation) ;
	
// Returns true if the solve was successful.
	bool SolveOptimizationProblem(ceres::Problem* problem) {
		std::cout<<" Solving the problem"<<std::endl;
		CHECK(problem != NULL);
		
		ceres::Solver::Options options;
		options.max_num_iterations = 50000;
		options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
		
		ceres::Solver::Summary summary;
		ceres::Solve(options, problem, &summary);
		
		std::cout << summary.FullReport() << '\n';
		
		return summary.IsSolutionUsable();
	}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
	bool OutputPoses(const std::string& filename, const ceres::examples::MapOfPoses& poses) {
		std::fstream outfile;
		outfile.open(filename.c_str(), std::istream::out);
		if (!outfile) {
			LOG(ERROR) << "Error opening the file: " << filename;
			return false;
		}
		for (std::map<int, ceres::examples::Pose3d, std::less<int>,Eigen::aligned_allocator<std::pair<const int, ceres::examples::Pose3d> > >::const_iterator poses_iter = poses.begin();poses_iter != poses.end(); ++poses_iter) {
			//poses_iter这个是迭代的东西
			const std::map<int, ceres::examples::Pose3d, std::less<int>,Eigen::aligned_allocator<std::pair<const int, ceres::examples::Pose3d> > >::value_type& pair = *poses_iter;
			outfile << pair.first << " " << pair.second.p.transpose() << " "
					<< pair.second.q.x() << " " << pair.second.q.y() << " "
					<< pair.second.q.z() << " " << pair.second.q.w() << '\n';
		}
		return true;
	}
	void poseTF(ceres::examples::MapOfPoses &input,Eigen::Isometry3d trans);

private:
	pcl::PointCloud<pcl::PointXYZI> gps;
};
#endif //PCD_COMPARE_GPS_LOOP_MAPPING_H
