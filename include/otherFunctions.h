//
// Created by echo on 19-5-24.
//

#ifndef PCD_COMPARE_MAIN_H
#define PCD_COMPARE_MAIN_H

#define PCL_NO_PRECOMPILE
#include <iostream>
#include <pcl/io/pcd_io.h>//不能两次引用头文件
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <dirent.h>
#include <pcl/filters/voxel_grid.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include "distortion/beam_separate.h"
#include "mapping/lmOptimizationSufraceCorner.h"
#include "oneFrameGND/ground_seg.h"
#include "spline/experiment.h"
#include "g2oIO/PoseGraphIO.h"
#include "DataIO/ReadBag.h"
#include "GPS/gpsTools.h"
#include "registration/registration.h"
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/cloud_viewer.h>
#include "ndt_omp/include/pclomp/ndt_omp.h"
#include <ndt_omp/include/pclomp/gicp_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ros/ros.h>
#include <6DOFcalib/Calibration6DOF.h>
#include <nav_msgs/Path.h>
#include <string>
#include <iostream>
#include "imgAddColor2Lidar/imgAddColor2Lidar.h"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Isometry3d)
using namespace g2o;
class main_function {
public:
	
	main_function(){};
	std::string LiDAR_type = "Velodyne";
//extern const std::string LiDAR_type = "Hesai";
// 1. 参数初始化
	bool tensorvoting = true;

	std::vector<std::string> file_names_;
	std::vector<std::string> PNG_file_names_;
	std::string filename;
	std::string filepath = "/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_pcd";
	Eigen::Isometry3d curICP = Eigen::Isometry3d::Identity();
	int cur_id = 0;
	int start_id = 0;//设置开始结束的点
	int end_id = 2;
//打开g2o文件路径
	std::string g2o_path = "null";
	int past = 0;
//存g2o路径
	std::string save_g2o_path = "/home/echo/small_program/test.g2o";
//存点云的路径
	std::string save_pcd_path = "/home/echo/map.pcd";
	std::string save_color_pcd_path = "/home/echo/map_color.pcd";
//存储g2o为iso3d
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector;

//0.设置起始结束的pcd
	void setStartEnd() {
		std::cout << "设置起始pcd" << std::endl;
		cin >> start_id;
		std::cout << "start: " << start_id << std::endl;
		std::cout << "设置结束pcd" << std::endl;
		cin >> end_id;
		std::cout << "end: " << end_id << std::endl;
	}

//1. 设置输入模式: 1.g2o+pcd的传统模式(pcd+g2o路径) 2.point to plane ICP (需要提供pcd路径)
	int status = 0;

//2. 输入模式的函数
	int getParam(int argc, char **argv) ;

// 3. 得到名称
	bool GetFileNames(const std::string directory, const std::string suffix) ;
	bool GetPNGFileNames(const std::string directory, const std::string suffix) ;
//4. 排序
	bool FindFileseq(int64_t seq);

//5. 保存g2o
	void saveFile(std::string outFilename, std::vector<VertexSE3 *> vertices,
				  std::vector<EdgeSE3 *> edges) {
		ofstream fileOutputStream;
		fileOutputStream.open(outFilename.c_str());
		std::string vertexTag = Factory::instance()->tag(vertices[0]);
		std::string edgeTag = Factory::instance()->tag(edges[0]);
		ostream &fout = outFilename != "-" ? fileOutputStream : cout;
		for (size_t i = 0; i < vertices.size(); ++i) {
			VertexSE3 *v = vertices[i];
			fout << vertexTag << " " << v->id() << " ";
			v->write(fout);
			fout << endl;
		}
		
		for (size_t i = 0; i < edges.size(); ++i) {
			EdgeSE3 *e = edges[i];
			VertexSE3 *from = static_cast<VertexSE3 *>(e->vertex(0));
			VertexSE3 *to = static_cast<VertexSE3 *>(e->vertex(1));
			fout << edgeTag << " " << from->id() << " " << to->id() << " ";
			e->write(fout);
			fout << endl;
		}
	}

//6 .读取g2o文件
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getEigenPoseFromg2oFile(
			std::string g2ofilename) {
		cout << "Reading the g2o file ~" << endl;
		std::ifstream fin(g2ofilename);
		cout << "G2o opened" << endl;
		if (!fin) {
			std::cerr << "file " << g2ofilename << " does not exist." << std::endl;
			std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT;     //todo
			vT.resize(1);
			vT[0] = Eigen::Matrix4d::Identity(4, 4);
			return vT;
		}
		
		
		std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT;
		while (!fin.eof()) {
			std::string name;
			fin >> name;
			if (name == "VERTEX_SE3:QUAT") {
				g2o::VertexSE3 v;
				
				int index = 0;
				fin >> index;
				v.setId(index);
				v.read(fin);
				Eigen::Isometry3d T = v.estimate();
				vT.push_back(T);
			} else if (name == "EDGE_SE3:QUAT") {
				continue;
			} else
				continue;
			if (!fin.good()) break;
		}
		std::cout << "read total " << vT.size() << " vertices\n";
		return vT;
	}

//7. 读trans和附近pcd拼成点云
//7.1.trans vector 7.2.查找的位置 7.3.点云读取的路径
	void genlocalmap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
					 std::string filepath, pcl::PointCloud<pcl::PointXYZI> &bigmap) {
		Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
		//1.1提取地面,准备产生地面的地图
		PlaneGroundFilter filter;
		
		bool kitti = true;
		featureExtraction Feature;
		if (kitti) {
//        Eigen::AngleAxisd odom_angle2 (M_PI/2, Eigen::Vector3d (1,0,0));
//        Eigen::AngleAxisd odom_angle3 (M_PI/2, Eigen::Vector3d (0,0,1));
//        pcd_rotate = odom_angle2*pcd_rotate;
//        pcd_rotate = odom_angle3*pcd_rotate;
		}
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_add(
				new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(
				new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_aft(
				new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(
				new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rot(
				new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<PointTypeBeam>::Ptr test(
				new pcl::PointCloud<PointTypeBeam>);
		pcl::PointCloud<PointTypeBeam>::Ptr pcout(
				new pcl::PointCloud<PointTypeBeam>);
		pcl::PointCloud<PointTypeSm> cornerPointsSharp;
		pcl::PointCloud<PointTypeSm> cornerPointsLessSharp;
		pcl::PointCloud<PointTypeSm> surfPointsFlat;
		pcl::PointCloud<PointTypeSm> surfPointsLessFlat;
		Eigen::Isometry3d out3d = Eigen::Isometry3d::Identity();
		pcl::PointCloud<PointTypeSm>::Ptr segmentedCloud(new pcl::PointCloud<PointTypeSm>);
		cout << "start interation" << endl;
		ofstream fout("log.txt");
		pcl::PCDWriter writer;
		float min_intensity;
		float max_intensity;
		util tools;
		//1.遍历所有点
		for (int i = 1; i < file_names_.size(); i++) {
			if (i > start_id && i < end_id) {
				//1.1设置起始和结束的点
				std::vector<int> indices1;
				//读点云
				pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], *cloud_bef);
				pcl::removeNaNFromPointCloud(*cloud_bef, *cloud_rot, indices1);
				
				*cloud_bef = *cloud_rot;
				//调整pandar的反射率
				min_intensity = cloud_bef->points[0].intensity;
				max_intensity = 0;
				for (int k = 0; k < cloud_bef->size(); ++k) {
					
					if (cloud_bef->points[k].intensity > max_intensity) {
						max_intensity = cloud_bef->points[k].intensity;
					}
					if (cloud_bef->points[k].intensity < min_intensity) {
						min_intensity = cloud_bef->points[k].intensity;
					}
				}
				for (int k = 0; k < cloud_bef->size(); ++k) {
					cloud_bef->points[k].intensity = (cloud_bef->points[k].intensity) * 1e43;
				}
				//结束调整反射率
				//开始采集感兴趣的区域
				if (tensorvoting) {
					pcl::PassThrough<pcl::PointXYZI> pass;   //1. passthrough
					pass.setInputCloud(cloud_bef);
					pass.setFilterFieldName("x");
					pass.setFilterLimits(-25, 25);
					pass.filter(*cloud_rot);
					pass.setInputCloud(cloud_rot);
					pass.setFilterFieldName("y");
					pass.setFilterLimits(-50, 50);
					pass.filter(*cloud_bef);
					pass.setInputCloud(cloud_bef);
					pass.setFilterFieldName("z");
					pass.setFilterLimits(-10, -1.8);
					pass.filter(*cloud_rot);
				}
				
				//计算两帧的增量
				*cloud_bef = *cloud_rot;
				out3d = trans_vector[i].matrix();
				out3d = trans_vector[i - 1].inverse().matrix() * out3d.matrix();
				
				fout << i << endl;
				fout << "out3d.matrix()" << endl;
				fout << out3d.matrix() << endl;
				//去畸变
				Feature.checkorder(cloud_bef, test);
				Feature.adjustDistortion(test, pcout, out3d);
				/*	//保存每贞
				 * std::stringstream path;
					path<<"map/"<<i<<".pcd";
					writer.write<PointTypeBeam>(path.str(),*pcout, true);
					  std::stringstream path1;
					  path1<<"map/"<<i<<"_.pcd";
					  writer.write<pcl::PointXYZI>(path1.str(),*cloud_bef, true);*/
				Feature.calculateSmoothness(pcout, segmentedCloud);
				Feature.calcFeature(segmentedCloud);
				//tmp用来转换格式
				tmp->clear();
				tmp->resize(pcout->size());
				for (int j = 0; j < pcout->size(); ++j) {
					tmp->points[j].x = pcout->points[j].x;
					tmp->points[j].y = pcout->points[j].y;
					tmp->points[j].z = pcout->points[j].z;//14.40
					tmp->points[j].intensity = pcout->points[j].intensity;
				}
				//todo 改成 第一步提取index 先,之后记录index,最后滤除
				//***这里是地面滤波器
				filter.point_cb(*tmp);
				*tmp = *filter.g_ground_pc;
				pcl::VoxelGrid<pcl::PointXYZI> sor;
				/*   sor.setInputCloud(tmp);                   //设置需要过滤的点云给滤波对象
				   sor.setLeafSize(0.1, 0.1, 0.1);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
				   sor.filter(*cloud_aft);*/
				//new
				*cloud_bef = *tmp;
				pcl::transformPointCloud(*cloud_bef, *cloud_aft, trans_vector[i].matrix());
				*cloud_add += *cloud_aft;
				
				if (!tensorvoting) {
					if (i % 100 == 50) {
						sor.setInputCloud(cloud_add);                   //设置需要过滤的点云给滤波对象
						sor.setLeafSize(0.05, 0.05, 0.05);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
						sor.filter(*cloud_aft);
						*cloud_add = *cloud_aft;
						std::cout.width(3);//i的输出为3位宽
						int percent = 0;
						int size_all = 0;
						size_all = static_cast<int>(file_names_.size());
						percent = i * 100 / size_all;
						std::cout << percent << "%" << std::endl;
						std::cout << "\b\b\b";//回删三个字符，使数字在原地变化
					}
				}
				int percent = 0;
				int size_all = 0;
				size_all = end_id - start_id;
				percent = i * 100 / size_all;
				std::cout << percent << "%" << std::endl;
				std::cout << "\b \b \b ";//回删三个字符，使数字在原地变化
			}
			
		}
		fout.close();
		cout << "end interation" << endl;
		cout << "max intenstity: " << max_intensity << " min intensity: " << min_intensity << std::endl;
		//转换回(0,0,0,0,0,0)
		pcl::transformPointCloud(*cloud_add, *cloud_aft, trans_vector[0].inverse().matrix());
		// 全点云时候应当加
		if (!tensorvoting) {
			pcl::VoxelGrid<pcl::PointXYZI> sor;
			sor.setInputCloud(cloud_aft);
			sor.setLeafSize(0.2, 0.2, 0.2);
			sor.filter(*cloud_add);
		}
		// pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		// outrem.setInputCloud(cloud_add);
		// outrem.setRadiusSearch(0.5);
		// outrem.setMinNeighborsInRadius (5);
		// // 应用滤波器
		// outrem.filter (*cloud_aft);
		//这里就是整
		bigmap.clear();
		for (int l = 0; l < cloud_add->size(); ++l) {
			if (cloud_add->points[l].intensity > (float) 14.4860 && cloud_add->points[l].intensity < (float) 14.96) {
				bigmap.push_back(cloud_add->points[l]);
			}
		}
		*cloud_add = bigmap;
		cout << "map saving" << endl;
		writer.write<pcl::PointXYZI>("final_map.pcd", *cloud_add, true);
	}

//8. 用来测试插值的 输入: 位姿vector
	void testspline(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector) {
		Eigen::Isometry3d lastodom, latest_odom, lidar_latest, temp_new;
		PoseGraphIO saveGraph;
		for (int j = 0; j < trans_vector.size() - 3; ++j) {
			Eigen::Isometry3d t1, t2, t3, t4;
			t1 = trans_vector[j];
			t2 = trans_vector[j + 1];
			t3 = trans_vector[j + 2];
			t4 = trans_vector[j + 3];
			double num = 3;//两个位姿之间插几个
			for (double i = 0; i < num; ++i) {
				SplineFusion sf;
				saveGraph.insertPose(sf.cumulativeForm(t1, t2, t3, t4, i / num));
			}
		}
		//保存g2o
		saveGraph.saveGraph("/home/echo/test_ws/spline_g2o/a.g2o");
	}

//9. feature map loam 特针点的地图
	void genfeaturemap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
					   std::string filepath, pcl::PointCloud<pcl::PointXYZI> &bigmap) {
		//0.初始化参数
		Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
		featureExtraction Feature;
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(
				new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> cloud_rot_pc;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rot(
				new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<PointTypeBeam>::Ptr test(
				new pcl::PointCloud<PointTypeBeam>);
		pcl::PointCloud<PointTypeBeam>::Ptr pcout(
				new pcl::PointCloud<PointTypeBeam>);
		
		pcl::PointCloud<PointTypeSm> cornerPointsSharp, wholemap;
		pcl::PointCloud<PointTypeSm> cornerPointsLessSharp;
		pcl::PointCloud<PointTypeSm> surfPointsFlat;
		pcl::PointCloud<PointTypeSm> surfPointsLessFlat;
		pcl::PointCloud<PointTypeSm> tfedPC;
		Eigen::Isometry3d out3d = Eigen::Isometry3d::Identity();
		pcl::PointCloud<PointTypeSm>::Ptr segmentedCloud(new pcl::PointCloud<PointTypeSm>);
		cout << "start interation" << endl;
		pcl::PCDWriter writer;
		//todo 这里改过了,还没改回来接口
		//groundSeg gs;
		
		//	1.迭代添加点云 大循环********
		for (int i = 0; i < file_names_.size(); i++) {
			if (i > start_id && i < end_id) {
				
				std::cout << "remains: " << end_id - i << std::endl;
				std::vector<int> indices1;
				//读点云
				pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], *cloud_bef);
				pcl::removeNaNFromPointCloud(*cloud_bef, *cloud_rot, indices1);
				out3d = trans_vector[i].matrix();
				out3d = trans_vector[i - 1].inverse().matrix() * out3d.matrix();
				pcl::copyPointCloud(*cloud_rot, cloud_rot_pc);
				//pcl::copyPointCloud(gs.point_cb(cloud_rot_pc),*cloud_bef);
				
				Feature.checkorder(cloud_bef, test);
				Feature.adjustDistortion(test, pcout, out3d);//输入点云 输出点云 相对tf
				Feature.calculateSmoothness(pcout, segmentedCloud);
				Feature.calcFeature(segmentedCloud);
				
				//把3m外的 特征点提取
				PointTypeSm temp;
				pcl::transformPointCloud(*Feature.cornerPointsSharp, tfedPC, trans_vector[i].matrix());
				for (int k = 0; k < tfedPC.size(); ++k) {
					if (tfedPC.points[k].range > 3) {//过滤附近的点用的
						temp = tfedPC.points[k];
						temp.pointType = 1.0;
						cornerPointsSharp.points.push_back(temp);
					}
				}
				
				pcl::transformPointCloud(*Feature.cornerPointsLessSharp, tfedPC, trans_vector[i].matrix());
				for (int k = 0; k < tfedPC.size(); ++k) {
					if (tfedPC.points[k].range > 3) {
						temp = tfedPC.points[k];
						temp.pointType = 2.0;
						cornerPointsLessSharp.points.push_back(temp);
					}
				}
				
				pcl::transformPointCloud(*Feature.surfPointsFlat, tfedPC, trans_vector[i].matrix());
				for (int k = 0; k < tfedPC.size(); ++k) {
					if (tfedPC.points[k].range > 3) {
						temp = tfedPC.points[k];
						temp.pointType = 3.0;
						surfPointsFlat.points.push_back(temp);
					}
				}
				int wtf;
				wtf = surfPointsLessFlat.size();
				pcl::transformPointCloud(*Feature.surfPointsLessFlat, tfedPC, trans_vector[i].matrix());
				for (int k = 0; k < tfedPC.size(); ++k) {
					if (tfedPC.points[k].range > 3) {
						temp = tfedPC.points[k];
						temp.pointType = 4.0;
						surfPointsLessFlat.points.push_back(temp);
					}
				}
				tfedPC.clear();
				cloud_bef->clear();
			}
		}
		wholemap += cornerPointsSharp;
		wholemap += cornerPointsLessSharp;
		wholemap += surfPointsFlat;
		wholemap += surfPointsLessFlat;
		
		writer.write<PointTypeSm>("cornerPointsSharp.pcd", cornerPointsSharp, true);
		writer.write<PointTypeSm>("cornerPointsLessSharp.pcd", cornerPointsLessSharp, true);
		writer.write<PointTypeSm>("surfPointsFlat.pcd", surfPointsFlat, true);
		writer.write<PointTypeSm>("surfPointsLessFlat.pcd", surfPointsLessFlat, true);
		writer.write<PointTypeSm>("wholemap.pcd", wholemap, true);
		pcl::PointXYZI curPC;
		
		cout << "end interation" << endl;
		//转换回(0,0,0,0,0,0)
//	pcl::transformPointCloud(*cloud_add, *cloud_aft, trans_vector[0].inverse().matrix());
	}

//10.0 线性去畸变的接口
	void simpleDistortion(mypcdCloud input, Eigen::Matrix4f increase, pcl::PointCloud<pcl::PointXYZI> &output) {
		output.clear();
/*	for (int i = 0; i < input.size(); ++i) {
		pcl::PointCloud<pcl::PointXYZI> onepointcloud,onepointcloudtfed;
		pcl::PointXYZI onepoint;
		onepoint.x = input[i].x;
		onepoint.y = input[i].y;
		onepoint.z = input[i].z;
		onepoint.intensity = input[i].intensity;
		onepointcloud.push_back(onepoint);
		pcl::transformPointCloud(onepointcloud,onepointcloudtfed,increase*10*input[i].timestamp);
		output.push_back(onepointcloudtfed[0]);
	}
	printf("current input: %d after distortion: %d \n",input.size(),output.size());*/
		pcl::PointCloud<PointTypeBeam>::Ptr test(new pcl::PointCloud<PointTypeBeam>);
		pcl::PointCloud<PointTypeBeam>::Ptr pcout(new pcl::PointCloud<PointTypeBeam>);
		
		featureExtraction Feature;
		Eigen::Isometry3d se3;
		se3 = increase.cast<double>();
		PointTypeBeam temp;
		for (int i = 0; i < input.size(); ++i) {
			temp.x = input[i].x;
			temp.y = input[i].y;
			temp.z = input[i].z;
			temp.intensity = input[i].intensity;
			temp.pctime = input[i].timestamp * 10;
			temp.beam = input[i].ring;
			test->push_back(temp);
		}
		Feature.adjustDistortion(test, pcout, se3);
		pcl::copyPointCloud(*pcout, output);
	}


//11.1 omp NDT 配准
	pcl::PointCloud<pcl::PointXYZI>::Ptr align(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration,
											   const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud,
											   const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud) {
		registration->setInputTarget(target_cloud);
		registration->setInputSource(source_cloud);
		pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
		
		auto t2 = ros::WallTime::now();
		for (int i = 0; i < 10; i++) {
			registration->align(*aligned);
		}
		auto t3 = ros::WallTime::now();
		std::cout << "10times: " << (t3 - t2).toSec() * 1000 << "[msec]" << std::endl;
		std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;
		return aligned;
	}

//12.1.1 距离滤波
	void pointCloudRangeFilter(pcl::PointCloud<pcl::PointXYZI> &input, float range) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
		for (int i = 0; i < input.size(); ++i) {
			if (sqrtf(input[i].x * input[i].x + input[i].y + input[i].y) < range) {
				filtered->push_back(input[i]);
			}
		}
		
		//加一个
		pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;     //创建滤波器对象
		sor.setInputCloud(filtered);                      //设置待滤波的点云
		sor.setMeanK(50);                                //设置在进行统计时考虑的临近点个数
		sor.setStddevMulThresh(1.0);                //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
		sor.filter(input);
		
	}


//13. 转换一个点的坐标
	void transformOnePoint(Eigen::Matrix4f t, pcl::PointXYZI &input) {
		Eigen::Matrix<float, 4, 1> temp_point, result;
		temp_point(0, 0) = input.x;
		temp_point(1, 0) = input.y;
		temp_point(2, 0) = input.z;
		temp_point(3, 0) = 1;
		result = t * temp_point;
		input.x = result(0, 0);
		input.y = result(1, 0);
		input.z = result(2, 0);
	}


//14. tools 建立局部地图 //a. 挑选关键帧(有必要?) //b. 维持队列长度 //d. downsample 15 帧大概1.2 s
	
	pcl::PointCloud<pcl::PointXYZI>
	lidarLocalMap(std::vector<Eigen::Matrix4f> &poses, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clouds,
				  int buffer_size) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> map_temp;
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_temp_ptr(new pcl::PointCloud<pcl::PointXYZI>);
		std::vector<Eigen::Matrix4f> poses_tmp;
		std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds_tmp;
		Eigen::Matrix4f tf_all = Eigen::Matrix4f::Identity();
		pcl::VoxelGrid<pcl::PointXYZI> sor;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor1;
 
		//最新帧降采样
		*map_temp_ptr = clouds.back();
		sor.setInputCloud(map_temp_ptr);
		//sor.setLeafSize(0.05f, 0.05f, 0.05f);//室内
		sor.setLeafSize(0.3f, 0.3f, 0.1f); //外面
		sor.filter(clouds.back());
		*map_temp_ptr = clouds.back();
		sor1.setInputCloud(map_temp_ptr);
		sor1.setMeanK(50);
		sor1.setStddevMulThresh(1.0);
		sor1.filter(clouds.back());
		
		//note 15帧64线 0.02 大概225520个点 //10帧试试
		if (poses.size() >= buffer_size) {
			for (int i = poses.size() - buffer_size; i < poses.size(); i++) {
				pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
				//下面的if 保证了 clouds 里面都是 降采样过的点云
				*map_ptr += map_temp;
				poses_tmp.push_back(poses[i]);
				clouds_tmp.push_back(clouds[i]);
			}
			poses = poses_tmp;
			clouds = clouds_tmp;
			
		} else if (poses.size() > 1) {//第一开始点云不多的情况 不要第一个帧,因为没有去畸变
			for (int i = 1; i < poses.size(); i++) {
				pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
				*map_ptr += map_temp;
			}
		} else {
			pcl::transformPointCloud(clouds[0], *map_ptr, poses[0]);
		}
		
		pcl::transformPointCloud(*map_ptr, map_temp, poses.back().inverse());
		*map_ptr = map_temp;
 
		sor.setInputCloud(map_ptr);
		sor.setLeafSize(0.25f, 0.25, 0.1f);
		sor.filter(map_temp);

/*	pcl::UniformSampling<pcl::PointXYZI> filter;
	pcl::PointCloud<int> keypointIndices;
	filter.setInputCloud(map_ptr);
	filter.setRadiusSearch(0.15f); //2cm 测距精度 015 haixing
	filter.compute(keypointIndices);
	pcl::copyPointCloud(*map_ptr, keypointIndices.points, map_temp);*/
		//pointCloudRangeFilter(map_temp,75); //距离滤波可以去了
 
		std::cout << " 局部地图大小: " << map_temp.size() << std::endl;
		return map_temp;
	}
//15. 对4个位姿的点进行连续去畸变 自己维护一下序列()
//input 4个位姿 从t-1 到t+2 输入 t时刻的点云
//输出 continusTime 去过畸变的点云
	
	pcl::PointCloud<pcl::PointXYZI>
	continusTimeDistrotion(std::vector<Eigen::Matrix4f> &poses, std::vector<mypcdCloud> &clouds) {
		Eigen::Isometry3d t1, t2, t3, t4, current_trans;
		pcl::PointCloud<pcl::PointXYZI> result;
		pcl::PointXYZI temp;
		std::vector<Eigen::Matrix4f> poses_tmp;
		std::vector<mypcdCloud> clouds_tmp;
		double num = 3;//两个位姿之间插几个
		SplineFusion sf;
		
		//维持队列
		if (poses.size() == clouds.size()) {
			if (poses.size() < 4) {
				printf("等待4次配准结果\n");
				return result;
			}
			for (int j = poses.size() - 4; j < poses.size(); ++j) {
				poses_tmp.push_back(poses[j]);
				clouds_tmp.push_back(clouds[j]);
			}
			poses = poses_tmp;
			clouds = clouds_tmp;
		} else {
			printf("pose clouds 不相等\n");
			return result;
		}
		
		//转存位姿点
		t1 = poses[0].matrix().cast<double>();
		t2 = poses[1].matrix().cast<double>();
		t3 = poses[2].matrix().cast<double>();
		t4 = poses[3].matrix().cast<double>();
		
		for (int i = 0; i < clouds[1].size(); ++i) {
			current_trans = sf.cumulativeForm(t1, t2, t3, t4, clouds[clouds.size() - 3][i].timestamp * 10); //10为 10 帧每秒
			temp.x = clouds[1][i].x;
			temp.y = clouds[1][i].y;
			
			temp.z = clouds[1][i].z;
			temp.intensity = clouds[1][i].intensity;
			transformOnePoint(current_trans.matrix().cast<float>(), temp);
			result.push_back(temp);
		}
		printf("continue-time 完成\n");
		return result;
	}


//功能1 用g2o pose 建图/**/ 就是用一一对应的g2o文件 进行拼图操作
	int g2omapping() {
		//得到所有的位姿向量
		trans_vector = getEigenPoseFromg2oFile(g2o_path);
		//生成局部地图
/*	lmOptimizationSufraceCorner lm;
	std::vector<double>  vector;
	Eigen::Isometry3d se3;
	//测试 rpy->se3
	for (int i = 0; i < trans_vector.size(); ++i) {
		std::cout<<trans_vector[i].matrix()<<std::endl;
		lm.eigen2RPYXYZ(trans_vector[i],vector);
		for (int j = 0; j < vector.size(); ++j) {
			std::cout<<vector[j]<<std::endl;
		}
		lm.RPYXYZ2eigen(vector,se3);
		std::cout<<i<<std::endl;
	}*/
		//样条插值
		testspline(trans_vector);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
		cout << "trans_vector.size() : " << trans_vector.size() << " file_names_.size() : " << file_names_.size()
			 << endl;
		if (trans_vector.size() == file_names_.size()) {
			//genfeaturemap(trans_vector,filepath,*cloud1);
			genlocalmap(trans_vector, filepath, *cloud1);
		} else {
			cout << "!!!!! PCD & g2o does not have same number " << endl;
			return 0;
		}
	}
//16 LiDAR 局部地图生成(){} 通过距离来
	pcl::PointCloud<pcl::PointXYZI>
	lidarLocalMapDistance(std::vector<Eigen::Matrix4f> &poses, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clouds,
				  double distiance, int buffer_size,bool & local_map_updated,	pcl::PointCloud<pcl::PointXYZI> last_local_map) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> map_temp;
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_temp_ptr(new pcl::PointCloud<pcl::PointXYZI>);
		std::vector<Eigen::Matrix4f> poses_tmp;
		std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds_tmp;
		Eigen::Matrix4f tf_all = Eigen::Matrix4f::Identity();
		pcl::VoxelGrid<pcl::PointXYZI> sor;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor1;
 		//存放两帧的位姿
		Eigen::Matrix4f pose_latest;
		Eigen::Matrix4f pose_last;
		Eigen::Matrix4f pose_diff;
		double distance_calc;
		//0. 得到这一阵位姿和上一阵位姿
		if(poses.size()>1){
			pose_latest =  poses.back();
			pose_last = poses.at(poses.size()-2);
			pose_diff = pose_latest*pose_last.inverse();
			distance_calc = sqrtf(pose_diff(0,3)*pose_diff(0,3)+
								  pose_diff(1,3)*pose_diff(1,3)+
								  pose_diff(2,3)*pose_diff(2,3));
		}else{
			distance_calc = 100;
		}
		std::cout<<"distance_calc: "<<distance_calc<<std::endl;
		//满足运动距离
		if(distance_calc>distiance){
			local_map_updated = true;
			//1 最新帧降采样
			*map_temp_ptr = clouds.back();
			sor.setInputCloud(map_temp_ptr);
			//sor.setLeafSize(0.05f, 0.05f, 0.05f);//室内
			sor.setLeafSize(0.3f, 0.3f, 0.1f); //外面
			sor.filter(clouds.back());
			*map_temp_ptr = clouds.back();
			sor1.setInputCloud(map_temp_ptr);
			sor1.setMeanK(50);
			sor1.setStddevMulThresh(1.0);
			sor1.filter(clouds.back());
			
			//2. note 15帧64线 0.02 大概225520个点 //10帧试试
			if (poses.size() >= buffer_size) {
				for (int i = poses.size() - buffer_size; i < poses.size(); i++) {
					pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
					//下面的if 保证了 clouds 里面都是 降采样过的点云
					*map_ptr += map_temp;
					poses_tmp.push_back(poses[i]);
					clouds_tmp.push_back(clouds[i]);
				}
				poses = poses_tmp;
				clouds = clouds_tmp;
				
			} else if (poses.size() > 1) {//第一开始点云不多的情况 不要第一个帧,因为没有去畸变
				for (int i = 1; i < poses.size(); i++) {
					pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
					*map_ptr += map_temp;
				}
			} else {
				pcl::transformPointCloud(clouds[0], *map_ptr, poses[0]);
			}
			//3. 地图转换到当前的最后一帧的位姿
			pcl::transformPointCloud(*map_ptr, map_temp, poses.back().inverse());
			*map_ptr = map_temp;
			//4.降采样
			sor.setInputCloud(map_ptr);
			sor.setLeafSize(0.25f, 0.25, 0.1f);
			sor.filter(map_temp);
			std::cout << " 局部地图大小: " << map_temp.size() << std::endl;
			return map_temp;
		}else{
			local_map_updated = false;
			std::cout<<"false"<<std::endl;
			//2. 这个只保留前面的点云和位姿
			if (poses.size() >= buffer_size) {
				for (int i = 0; i < buffer_size; i++) {
					pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
					//下面的if 保证了 clouds 里面都是 降采样过的点云
					*map_ptr += map_temp;
					poses_tmp.push_back(poses[i]);
					clouds_tmp.push_back(clouds[i]);
				}
				poses = poses_tmp;
				clouds = clouds_tmp;
				//3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
				sor.setInputCloud(map_ptr);
				sor.setLeafSize(0.25f, 0.25, 0.1f);
				sor.filter(map_temp);
				pcl::transformPointCloud(map_temp, *map_ptr, pose_latest.inverse());
				return *map_ptr;
				
			} else if (poses.size() > 1) {//第一开始点云不多的情况 不要第一个帧,因为没有去畸变
				for (int i = 1; i < poses.size(); i++) {
					pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
					*map_ptr += map_temp;
				}
				//3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
				pcl::transformPointCloud(*map_ptr, map_temp, pose_latest.inverse());
				*map_ptr = map_temp;
				//4.降采样
				sor.setInputCloud(map_ptr);
				sor.setLeafSize(0.25f, 0.25, 0.1f);
				sor.filter(map_temp);
				return map_temp;
			} else {
				pcl::transformPointCloud(clouds[0], *map_ptr, poses[0]);
				//3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
				pcl::transformPointCloud(*map_ptr, map_temp, pose_latest.inverse());
				*map_ptr = map_temp;
				//4.降采样
				sor.setInputCloud(map_ptr);
				sor.setLeafSize(0.25f, 0.25, 0.1f);
				sor.filter(map_temp);
				return map_temp;
			}
			
		}
	}
	ros::Time fromPath2Time(std::string s);

//6.2 建图前端 点面icp
	int point2planeICP(){
		//g2o结果存储
		PoseGraphIO g2osaver;
		pcl::PointCloud<pcl::PointXYZI> tfed;
		pcl::PointCloud<pcl::PointXYZRGB> tfed_color;
		pcl::PointCloud<pcl::PointXYZI> cloud_continus_time_T_world;
		pcl::PointCloud<pcl::PointXYZI> cloud_continus_time_T_LiDAR;
		pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> nonan;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef_ds(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> cloud_map_ds;
		mypcdCloud xyzItimeRing; //现在改了之后的点
		VLPPointCloud xyzirVLP;
		RoboPointCLoud xyzirRobo;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai(new pcl::PointCloud<pcl::PointXYZI>);//io 进来的点
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_local_map(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_to_pub(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZI>);				//线性去畸变的图
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_map_color(new 	pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map_continus_time(new pcl::PointCloud<pcl::PointXYZI>);//连续时间的
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr filter1(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr raw(new pcl::PointCloud<pcl::PointXYZINormal>);
		bool bperal_edge = false;
		//ros debug
		//todo 这里可以去掉ros
		ros::NodeHandle node;
		ros::NodeHandle privateNode("~");
		sensor_msgs::PointCloud2 to_pub_frame;
		sensor_msgs::PointCloud2 to_pub_frame_linear;
		pcl::PCLPointCloud2 pcl_frame;
		ros::Publisher test_frame;
		ros::Publisher continue_frame;
		ros::Publisher test_frame_linear;
		ros::Publisher path_publish;
		test_frame = node.advertise<sensor_msgs::PointCloud2>("/local_map", 5);
		continue_frame = node.advertise<sensor_msgs::PointCloud2>("/continue_frame", 5);
		test_frame_linear = node.advertise<sensor_msgs::PointCloud2>("/current_frame_linear", 5);
		path_publish = node.advertise<nav_msgs::Path>("/lidar_path", 5);
		//todo end 这里可以去掉ros
		//存tf的
		std::vector<Eigen::Matrix4f> poses;
		std::vector<pcl::PointCloud<pcl::PointXYZI>>  clouds;
		//每两帧之间的变换
		std::vector<Eigen::Matrix4f> icp_result;
		Eigen::Matrix4f current_scan_pose = Eigen::Matrix4f::Identity();
		//滤波相关
 
		pcl::PointCloud<int> keypointIndices;
		pcl::UniformSampling<pcl::PointXYZI> filter_us;
		//车速
		float curr_speed = 0;
		//continus-time distortion
		std::vector<Eigen::Matrix4f>  poses_distortion;
		std::vector<mypcdCloud> clouds_distortion_origin;
//class
		util tools,tools2;
		registration icp;
		//位姿存成csv
		CSVio csvio;//位姿csv
		imgAddColor2Lidar a;//投影点云
		a.readExInt("/home/echo/fusion_ws/src/coloured_cloud/ex_params.txt");
		//投影相关
		pcl::PointCloud<pcl::PointXYZRGB> tosave;
		cv::Mat mat;VLPPointCloud cloudin;

/*		icp.setParam("/media/echo/DataDisc/3_program/mapping/cfg/icp.yaml");*/

		pcl::PCDWriter writer;
		bool first_cloud = true;
		bool distortion = true;
		std::cout<<file_names_ .size()<<std::endl;
		std::cout<<start_id<<" "<<end_id<<std::endl;
		bool VLP = true;
		std::string LiDAR_type = "VLP";
		bool local_map_updated = true; //todo 加入地图更新判断 1100-3000
		std::vector<nav_msgs::Odometry> odoms;//当前结果转成odom 存储
		nav_msgs::Odometry current_odom;
	
		//开始迭代
		if(file_names_ .size() != PNG_file_names_ .size()){
			std::cout<<"wrong size, pcd: "<<file_names_.size()<<" PNG "<<PNG_file_names_ .size()<<std::endl;
			return(0);
		}
		for(int i = 0;  i <file_names_ .size();i++){
			tools2.timeCalcSet("total");
			if (i>start_id && i<end_id) {
				//存储时间戳
				ros::Time cur_time;
				cur_time = fromPath2Time(file_names_[i]);
				//0. 读取不同pcd类型
				//存储完成
				if(LiDAR_type == "VLP"){//判断用的是不是vlp的,用的话进行转换
					pcl::io::loadPCDFile<VLPPoint>(file_names_[i+1], xyzirVLP);
					//滤波
					VLPPointCloud::Ptr xyzirVLP_ptr(new VLPPointCloud);
					VLPPointCloud::Ptr xyzirVLP_ds_ptr(new VLPPointCloud);
					pcl::copyPointCloud(xyzirVLP,*xyzirVLP_ptr);
			 
					pcl::StatisticalOutlierRemoval<VLPPoint> sor;
					sor.setInputCloud (xyzirVLP_ptr);
					sor.setMeanK (50);
					sor.setStddevMulThresh (2);
					sor.filter (*xyzirVLP_ds_ptr);
					pcl::copyPointCloud(*xyzirVLP_ds_ptr, xyzirVLP);
					
					xyzItimeRing.clear();
					for (int j = 0; j < xyzirVLP.size(); ++j) {
						mypcd temp;
						temp.x = xyzirVLP[j].x;
						temp.y = xyzirVLP[j].y;
						temp.z = xyzirVLP[j].z;
						temp.intensity = xyzirVLP[j].intensity;
						temp.timestamp = xyzirVLP[j].time;
						temp.ring = xyzirVLP[j].ring;
						xyzItimeRing.push_back(temp);
					}
				}else if(LiDAR_type == "Hesai"){
					pcl::io::loadPCDFile<mypcd>(file_names_[i], xyzItimeRing);
				} else if(LiDAR_type == "robo"){
					pcl::io::loadPCDFile<RoboPoint>(file_names_[i], xyzirRobo);
					xyzItimeRing.clear();
					for (int j = 0; j < xyzirRobo.size(); ++j) {
						mypcd temp;
						temp.x = xyzirRobo[j].x;
						temp.y = xyzirRobo[j].y;
						temp.z = xyzirRobo[j].z;
						temp.intensity = xyzirRobo[j].intensity;
						temp.timestamp = xyzirRobo[j].time;
						temp.ring = xyzirRobo[j].ring;
						xyzItimeRing.push_back(temp);
					}
				}else{
					std::cout<<"unknown pcd type"<<std::endl;
				}
				pcl::copyPointCloud(xyzItimeRing,*cloud_hesai);
				//0. 读取完毕
				//1. 这里用pcl的 plane to plane icp
				if(first_cloud){
					//1.1 第一帧 不进行计算
					std::cout<<"first cloud" <<std::endl;
					pcl::copyPointCloud(*cloud_hesai,*cloud_local_map);
					poses.push_back(Eigen::Matrix4f::Identity());
					clouds.push_back(*cloud_local_map);
					first_cloud = false;
					g2osaver.insertPose(Eigen::Isometry3d::Identity());
				} else{
					//todo 有个bug,就是第二次去畸变没有用完全的tf去畸变 没问题
					//2.1 加个去畸变
					simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef); //T_l-1_l
					//2.1.1 设为普通icp********
					icp.transformation = Eigen::Matrix4f::Identity();
					//2.2 输入的也应该降采样
 
					//(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
					
					//2.3 ICP
					tools.timeCalcSet("第一次ICP用时     ");
					icp.SetNormalICP(); //设定scan-scan icp参数
					tfed = icp.normalIcpRegistration(cloud_bef,*cloud_local_map);
					icp_result.push_back(icp.increase);//T_l_l+1
					
					//2.3.1 再次去畸变 再次减小范围
					simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
					//pointCloudRangeFilter(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
					tools.timeUsed();
					
					tools.timeCalcSet("局部地图用时    ");
					//2.3.2.1 局部地图生成
					//*cloud_local_map = lidarLocalMap(poses,clouds,50);  //生成局部地图****
					*cloud_local_map = lidarLocalMapDistance(poses,clouds,0.5,20 ,local_map_updated,*cloud_local_map);  //生成局部地图****
					tools.timeUsed();
					
					//2.3.2 ******再次icp           *********** &&&&这个当做 lidar mapping
					tools.timeCalcSet("第二次ICP用时    ");
					icp.SetPlaneICP();	//设定点面 icp参数
					tfed = icp.normalIcpRegistrationlocal(cloud_bef,*cloud_local_map);
					icp_result.back() = icp_result.back()*icp.pcl_plane_plane_icp->getFinalTransformation(); //第一次结果*下一次去畸变icp结果
					
					//2.3.3 再次去畸变 再次减小范围
					simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
					//pointCloudRangeFilter(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
					tools.timeUsed();
					//2.4 speed
					curr_speed = sqrt(icp_result.back()(0,3)*icp_result.back()(0,3)+icp_result.back()(1,3)*icp_result.back()(1,3))/0.1;
					// 2.5 点云投影
					mat = cv::imread(PNG_file_names_[i]);
					tosave  = a.pclalignImg2LiDAR(mat,*cloud_bef);
					//2.6 存储结果
					*local_map_to_pub = *cloud_local_map;
					*cloud_local_map = *cloud_bef; 	//下一帧匹配的target是上帧去畸变之后的结果
					//可以用恢复出来的位姿 tf 以前的点云
					clouds.push_back(*cloud_bef);
					std::stringstream pcd_save;
					pcd_save<<"dist_pcd/"<<i<<".pcd";
					writer.write(pcd_save.str(),*cloud_bef, true);
					//生成地图
					Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();
					//试一下这样恢复出来的位姿
					for (int k = 0; k < icp_result.size(); ++k) {
						current_pose *= icp_result[k];
					}
					//存储这次结果
					poses_distortion.push_back(current_pose.matrix());
					poses.push_back(current_pose.matrix());
					g2osaver.insertPose(Eigen::Isometry3d(current_pose.matrix().cast<double>()));
					clouds_distortion_origin.push_back(xyzItimeRing);
					std::cout<<"*****上次点云ID: "<<i<<" ***** speed: "<<3.6*curr_speed<<" km/h ******** \n\n\n"<<std::endl;
					//存一下'csvio
					Eigen::Isometry3d se3_save;
					csvio.LiDARsaveOnePose(Eigen::Isometry3d(current_pose.matrix().cast<double>()),cur_time);//转csv用的
					//运行最终的去畸变
					if(1){ //存大点云
						std::cout<<"全局坐标 \n"<<current_pose.matrix()<<std::endl;
						tfed = *cloud_bef;
						cloud_bef->clear();
						for (int j = 0; j < tfed.size(); ++j) {//距离滤波
							if(sqrt(tfed[j].x*tfed[j].x+tfed[j].y*tfed[j].y)>10){
								cloud_bef->push_back(tfed[j]);
							}
						}
						pcl::transformPointCloud(*cloud_bef,tfed,current_pose);
						pcl::transformPointCloud(tosave,tfed_color,current_pose);
						*cloud_map_color += tfed_color;
						*cloud_map += tfed;
						//存的点云缩小点,每50帧存一下结果;
						if(i%100==0){
							pcl::PointCloud<int> keypointIndices;
							filter_us.setInputCloud(cloud_map);
							filter_us.setRadiusSearch(0.02f);
							filter_us.compute(keypointIndices);
							pcl::copyPointCloud(*cloud_map, keypointIndices.points, cloud_map_ds);
							*cloud_map = cloud_map_ds;
							writer.write("cloud_map.pcd",*cloud_map, true);
							writer.write("cloud_map_color.pcd",*cloud_map_color, true);
						}
						
						/*			tools.timeCalcSet("连续时间去畸变用时:    ");
									cloud_continus_time_T_world = continusTimeDistrotion(poses_distortion,clouds_distortion_origin);//这里放的是最新的一帧和位姿
									*cloud_map_continus_time += cloud_continus_time_T_world;
												if (poses_distortion.size() == 4){ //进行了连续时间的去畸变就替换这个 转换到最新的T的坐标系下面
													pcl::transformPointCloud(cloud_continus_time_T_world,cloud_continus_time_T_LiDAR,current_pose.matrix().inverse());
													clouds[clouds.size()-3] = cloud_continus_time_T_LiDAR;
												}
									tools.timeUsed();
									//todo 这里可以去掉ros
									pcl::toPCLPointCloud2(*cloud_map_continus_time, pcl_frame);
									pcl_conversions::fromPCL(pcl_frame, to_pub_frame);
									to_pub_frame.header.frame_id = "/map";
									continue_frame.publish(to_pub_frame);
					*/
						pcl::toPCLPointCloud2(tfed, pcl_frame);
						pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
						to_pub_frame_linear.header.frame_id = "/map";
						test_frame_linear.publish(to_pub_frame_linear);
						
						pcl::toPCLPointCloud2(*local_map_to_pub, pcl_frame);
						pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
						to_pub_frame_linear.header.frame_id = "/map";
						test_frame.publish(to_pub_frame_linear);
						tools2.timeUsed();
						
						//	todo 这里可以去掉ros
					}else{//存每一帧 防止内存爆炸
						cloud_continus_time_T_world = continusTimeDistrotion(poses_distortion,clouds_distortion_origin);//这里放的是最新的一帧和位姿
						std::stringstream pcd_save;
						pcd_save<<"tfed_pcd/"<<i<<".pcd";
						if(cloud_continus_time_T_world.size()>0){
							writer.write(pcd_save.str(),cloud_continus_time_T_world, true);
						}
					}
				}
			}
		}
		std::cout<<save_g2o_path<<"\n"<<save_pcd_path<<std::endl;
		g2osaver.saveGraph(save_g2o_path);
		writer.write(save_pcd_path,*cloud_map, true);
		writer.write(save_color_pcd_path,*cloud_map_color, true);
/*	writer.write("distro_final.pcd",*cloud_map_continus_time, true);*/
		csvio.LiDARsaveAll("wtf");
		return(0);
	}


// 功能3 设置road curb 的mapping
	void traversableMapping(){
		//点云缓冲
		pcl::PointCloud<pcl::PointXYZI> cloud_bef;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rot(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr filter(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr raw(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_aft(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_add(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr whole_map(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr raw_tf(new pcl::PointCloud<pcl::PointXYZI>);
		util tools;
		pcl::PCDWriter writer;
		//0. 得到每个点的位姿
		
		trans_vector = getEigenPoseFromg2oFile(g2o_path);
		//1.遍历点云
		if(trans_vector.size() == file_names_.size()){
			for(int i = 1;  i <file_names_ .size();i++){
				if (i>start_id && i<end_id) {
					cloud_bef.clear();
					cloud_aft->clear();
					result->clear();
					raw->clear();
					filter->clear();
					raw_tf->clear();
					pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], cloud_bef);
					
					std::vector<int> indices1;
					std::cout<<"size: "<<cloud_bef.size()/32.0<<std::endl;
					//1.读取每个点来的时间
					for (int j = 0; j < cloud_bef.size(); ++j) {
						pcl::PointXYZINormal aa;
						aa.x = cloud_bef[j].x;
						aa.y = cloud_bef[j].y;
						aa.z = cloud_bef[j].z;
						aa.intensity = cloud_bef[j].intensity;
						aa.normal_y = j%(cloud_bef.size()/32);
						raw->push_back(aa);
					}
					pcl::removeNaNFromPointCloud(*raw, *filter, indices1);
					tools.GetPointCloudBeam(*filter,*result);
					tools.GetBeamEdge(*filter,*result);
					
					pcl::transformPointCloud(*result, *cloud_aft, trans_vector[i].matrix());
					pcl::transformPointCloud(cloud_bef, *raw_tf, trans_vector[i].matrix());
					
					*cloud_add += *cloud_aft;
					*whole_map += *raw_tf;
				}
			}
			cout<<cloud_add->size()<<endl;
			writer.write("test.pcd",*cloud_add, false);
			writer.write("wholemap.pcd",*whole_map, false);
		} else{
			cout<<"!!!!! PCD & g2o does not have same number "<<endl;
		}
	}

// 功能4 使用encoder 和GPS LiDAR 去建图
	void encoderMapping(){
		pcl::PointCloud<pcl::PointXYZI> encoder_pcd;
		pcl::PointCloud<pcl::PointXYZI> gps_pcd;
		pcl::PCDWriter writer;
		//读取bag的程序
		ReadBag rb;
		//rb.getPath("/media/echo/DataDisc/9_rosbag/shandong_park/2019-08-14-16-41-05.bag");
		rb.getPath("/media/echo/DataDisc/9_rosbag/test_gps_lidar_calibration/2019-10-23-18-48-24.bag");
		//测试: 存储GPS + Encoder 到点云
		writer.write("/home/echo/encoder.pcd",rb.encoder_pcd);
		writer.write("/home/echo/gps_pcd.pcd",rb.gps_pcd);
	}

//功能5.1 LiDAR + GNSS mapping 杆臂值 标定
//程序准备设计的方案: 1. 读取一个bag 取其中的一段时间 eg 60s 600 帧进行 odom的计算
//2. 读取gps 转成lla
//3. 时间戳对齐
//4. 设置两个优化变量 一个是两个传感器的外参 一个是旋转的角度,就是雷达初始时刻相对于正北的朝向;
//5.
// /media/echo/DataDisc/9_rosbag/rsparel_64_ins 这个好像能work gps时间戳不太对 就用时间戳减1s
	void LiDARGNSScalibration (std::string lidar_g2o,std::string gps_pcd){
		pcl::PCDWriter writer;
		ReadBag rb;
		Calibration6DOF calibrate; //用来标定外参的
		std::vector<std::pair<Eigen::Isometry3d,double>>  gps_pose ;
		Eigen::Vector3d lla_origin;
		pcl::PointCloud<pcl::PointXYZ>::Ptr gps_position(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ> lidar_position,lidar_position_tfed;
		pcl::PointCloud<pcl::PointXYZ> gps_position_save;
		//计算外参要用的
		std::vector<Eigen::Matrix4d> gps_poses;std::vector<Eigen::Matrix4d> LiDAR_poses;Eigen::Isometry3d  T_lidar2INS;
		//处于计算时间考虑,分步执行
		//step 1: 把pcd计算出里程计
		//step 2: 把gps的位置转化为pcd
		//rb.gnssPCDExtrinsicParameters("/media/echo/DataDisc/9_rosbag/zed_pandar64_ins/Hesai_back_afternoon_2.bag",gps_pose,lla_origin);
		//测试: 存储GPS + Encoder 到点云
		//测试 step 1. 读取g2o的 位姿
		//测试 step 2. 读取gps位姿的pcd
		pcl::io::loadPCDFile<pcl::PointXYZ>(gps_pcd, *gps_position); //gnssPCDExtrinsicParameters 函数得到的gnss轨迹
		trans_vector = getEigenPoseFromg2oFile(lidar_g2o);//通过功能2 得到的位置
		//测试step 3. 虚拟出两个数据来计算出两个T 第一个T LiDAR到gnss中心的位姿 另一个是LiDAR到 LLA坐标的位姿
		
		//可视化一下当前的时间戳的对应关系.
		pcl::visualization::PCLVisualizer vis("gps2lidar");
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> aligned_handler(gps_position, "intensity");
		vis.addPointCloud(gps_position, aligned_handler, "map");
		for (int i = 0; i < trans_vector.size(); ++i) {
			pcl::PointXYZ temp;
			temp.x = trans_vector[i](0,3);
			temp.y = trans_vector[i](1,3);
			temp.z = trans_vector[i](2,3);
			vis.addLine<pcl::PointXYZ>(gps_position->points[i*5], temp, i*2%255, (i*3+100)%255, 0, std::to_string(i));
			lidar_position.push_back(temp);
			gps_position_save.push_back(gps_position->points[i*5]);
		}
		//vis.spin();
		//0.格式转化
		//这一部分产生一一对应的关系
		//todo 这里gps的位置比较多, 所以以 惯导的数量为基础 惯导 50hz lidar 10 hz
		for (int j = 200; j < trans_vector.size() ; ++j) {
			Eigen::Matrix4d temp;
			temp.setIdentity();
			temp(0,3) = gps_position->points[j*5].x;
			temp(1,3) = gps_position->points[j*5].y;
			temp(2,3) = gps_position->points[j*5].z;
			gps_poses.push_back(temp);
		}
		for (int k = 200; k < trans_vector.size(); ++k) {
			LiDAR_poses.push_back(trans_vector[k].matrix());
		}
		//1.计算lidar到gnss 外参
/*    for (int l = 0; l < LiDAR_poses.size(); ++l) {
        std::cout<<LiDAR_poses[l].matrix()<<std::endl;
    }*/
		calibrate.CalibrateGNSSLiDAR(gps_poses,LiDAR_poses,T_lidar2INS);
		pcl::transformPointCloud(lidar_position,lidar_position_tfed,T_lidar2INS.inverse().matrix());
		writer.write("route/lidar_position.pcd",lidar_position_tfed);
		writer.write("route/raw_lidar_position.pcd",lidar_position);
		writer.write("route/gps_position_save.pcd",gps_position_save);
		std::cout<<"标定结果:\n"<<T_lidar2INS.inverse().matrix()<<std::endl;
	}
//功能5.2 LiDAR + GNSS mapping 建图
	void LiDARGNSSMapping(){
	
	}
//功能6. NDT mapping
	void NDTmapping(){
		//	local variables
		pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr Final_map (new pcl::PointCloud<pcl::PointXYZI>());
		bool first_cloud = true;
		pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
		pcl::UniformSampling<pcl::PointXYZI> filter;
		filter.setRadiusSearch(0.1f); //0.1米
		mypcdCloud xyzItimeRing; //现在改了之后的点
		pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp
				(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
		//配置ndt
		ndt_omp->setResolution(1.0);
		Eigen::Matrix4d total_t = Eigen::Matrix4d::Identity();
		//loop
		for(int i = 0;  i <file_names_ .size();i++){
			if (i>start_id && i<end_id) {
				pcl::io::loadPCDFile<mypcd>(file_names_[i], xyzItimeRing);
				//这里放去畸变
				//复制到这
				pcl::copyPointCloud(xyzItimeRing,*source_cloud);
				if(first_cloud){
					first_cloud = false;
					pcl::copyPointCloud(xyzItimeRing,*target_cloud);
				}else{
					//0 显示配准的帧
					std::cout<<"from->to: "<<i<<"->"<<i-1<<std::endl;
					//1 降采样
					pcl::PointCloud<int> keypointIndices;
					
					//1.1 target 降采样
					filter.setInputCloud(target_cloud);
					filter.compute(keypointIndices);
					pcl::copyPointCloud(*target_cloud, keypointIndices.points, *downsampled);
					*target_cloud = *downsampled;
					//1.2 source降采样
					filter.setInputCloud(source_cloud);
					filter.compute(keypointIndices);
					pcl::copyPointCloud(*source_cloud, keypointIndices.points, *downsampled);
					*source_cloud = *downsampled;
					//2 配置线程数
					std::vector<int> num_threads = {1, omp_get_max_threads()};
					for(int n : num_threads) {
						ndt_omp->setNumThreads(n);
						ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
						aligned = align(ndt_omp, target_cloud, source_cloud);
					}
					//*target_cloud += *aligned;
					std::cout<<"target size: "<<target_cloud->size()<<std::endl;
					total_t *= ndt_omp->getFinalTransformation().cast<double>();
					std::cout<<total_t.matrix()<<std::endl;
					*target_cloud = *source_cloud;
					pcl::transformPointCloud(*source_cloud,*downsampled,total_t);
					*Final_map += *downsampled;
				}
			}
		}
		// visulization
		pcl::visualization::PCLVisualizer vis("vis");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_handler(Final_map, 255.0, 255.0, 0.0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> target_handler(target_cloud, 255.0, 0.0, 0.0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_handler(source_cloud, 0.0, 255.0, 0.0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> aligned_handler(aligned, 0.0, 0.0, 255.0);
		vis.addPointCloud(Final_map, map_handler, "target1");
		vis.addPointCloud(target_cloud, target_handler, "target");
		vis.addPointCloud(source_cloud, source_handler, "source");
		vis.addPointCloud(aligned, aligned_handler, "aligned");
		vis.spin();
	}
//功能7 读取hesaipcd
	void readAndSaveHesai(std::string path) {
		ReadBag a;
		//a.readHesai(path);
		a.readVLP16("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_fov.bag",
					"/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_pcd");
		//a.readTopRobosense("/media/echo/DataDisc/9_rosbag/9_huawei_jialuowuliu/2020-04-09-11-44-45.bag","/home/echo/2_huawei");
/*		std::vector<std::pair<Eigen::Isometry3d,double>>  gps_pose ;
		Eigen::Vector3d lla_origin;
		a.readcamera("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_fov.bag","/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/pic");
		a.gnssPCDExtrinsicParameters("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_fov.bag",gps_pose,lla_origin);
	}*/
	}
//功能8 用来测试模块好使不
	void testFunction(){
		pcl::PCDWriter writer;
		imgAddColor2Lidar a;
		a.readExInt("/home/echo/fusion_ws/src/coloured_cloud/ex_params.txt");
		pcl::PointCloud<pcl::PointXYZRGB> tosave;
		cv::Mat mat;VLPPointCloud cloudin;
		mat = cv::imread("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/pic/1586507671.986369610.png");
		pcl::io::loadPCDFile<VLPPoint>("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/small_pcd/1586507671.84489393.pcd", cloudin);
		GetFileNames("/media/echo/DataDisc/9_rosbag/8_imu_camera_rtk_vlp/pic","png");
		tosave  = a.pclalignImg2LiDAR(mat,cloudin);
		pcl::PointXYZRGB a1;
		writer.write("/home/echo/fusion_ws/result.pcd",tosave, true);
	}
//功能9 普通的16线建图
	void rslidarmapping(){
		registration icp;
		pcl::PointCloud<pcl::PointXYZI> xyzItimeRing;
		for(int i = 0;  i <file_names_ .size();i++){
			pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], xyzItimeRing);
		}
	}
};


#endif //PCD_COMPARE_MAIN_H
