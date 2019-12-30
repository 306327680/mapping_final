//
// Created by echo on 19-5-24.
//

#ifndef PCD_COMPARE_MAIN_H
#define PCD_COMPARE_MAIN_H

#include <iostream>
#include <pcl/io/pcd_io.h>
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
// 1. 参数初始化
bool tensorvoting = true;
using namespace g2o;
std::vector<std::string> file_names_;
std::string filename;
std::string filepath = "/media/echo/35E4DE63622AD713/test/";
Eigen::Isometry3d curICP = Eigen::Isometry3d::Identity();
int cur_id = 0;
int start_id = 0;//设置开始结束的点
int end_id = 2;
//打开g2o文件路径
std::string g2o_path = "null";
int past = 0;
//存g2o路径
std::string save_g2o_path = "/home/echo/small_program/test.g2o";
//存储g2o为iso3d
std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector;




// 2. 得到名称
bool GetFileNames(const std::string directory,const std::string suffix){
	file_names_.clear();
	DIR *dp;
	struct dirent *dirp;
	dp = opendir(directory.c_str());
	if (!dp) {
		std::cerr<<"cannot open directory:"<<directory<<std::endl;
		return false;
	}
	std::string file;
	while(dirp = readdir(dp)){
		file = dirp->d_name;
		if (file.find(".")!=std::string::npos){
			file = directory + "/" + file;
			if (suffix == file.substr(file.size() - suffix.size())){
				file_names_.push_back(file);
			}
		}
	}
	closedir(dp);
	std::sort(file_names_.begin(),file_names_.end());
	
	if(file_names_.empty()){
		std::cerr<<"directory:"<<directory<<"is empty"<<std::endl;
		return false;
	}
	std::cerr<<"路径: "<<directory<<" 有"<<file_names_.size()<<"个pcd文件"<<std::endl;
	return true;
}
//3. 排序
bool FindFileseq(int64_t seq){
	int64_t idx_file = seq;
	if (idx_file > file_names_.size()-1) {
		return INT64_MAX;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	uint64_t idx_seperator = file_names_[idx_file].find_last_of("/");
	filename = file_names_[idx_file].substr(idx_seperator + 1);
	std::cout<<"PCD file is "<<filename<<" seq is"<<idx_file<<std::endl;
	return true;
}

//4. 保存g2o
void saveFile(std::string outFilename, std::vector<VertexSE3*> vertices,
			  std::vector<EdgeSE3*> edges){
	ofstream fileOutputStream;
	fileOutputStream.open(outFilename.c_str());
	std::string vertexTag = Factory::instance()->tag(vertices[0]);
	std::string edgeTag = Factory::instance()->tag(edges[0]);
	ostream& fout = outFilename != "-" ? fileOutputStream : cout;
	for (size_t i = 0; i < vertices.size(); ++i)
	{
		VertexSE3* v = vertices[i];
		fout << vertexTag << " " << v->id() << " ";
		v->write(fout);
		fout << endl;
	}
	
	for (size_t i = 0; i < edges.size(); ++i) {
		EdgeSE3* e = edges[i];
		VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
		VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
		fout << edgeTag << " " << from->id() << " " << to->id() << " ";
		e->write(fout);
		fout << endl;
	}
}

//5 .读取g2o文件
std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getEigenPoseFromg2oFile(
		std::string &g2ofilename) {
	cout<<"Reading the g2o file ~"<<endl;
	std::ifstream fin(g2ofilename);
	cout<<"G2o opened"<<endl;
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

//6. 读trans和附近pcd拼成点云
//1.trans vector 2.查找的位置 3.点云读取的路径
void genlocalmap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
				 std::string filepath,pcl::PointCloud<pcl::PointXYZI>& bigmap){
	Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
	//1.1提取地面,准备产生地面的地图
	PlaneGroundFilter filter;
	
	bool kitti = true;
	featureExtraction Feature;
	if (kitti){
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
	pcl::PointCloud<PointTypeBeam>::Ptr test (
			new pcl::PointCloud<PointTypeBeam>);
	pcl::PointCloud<PointTypeBeam>::Ptr pcout(
			new pcl::PointCloud<PointTypeBeam>);
	pcl::PointCloud<PointTypeSm> cornerPointsSharp;
	pcl::PointCloud<PointTypeSm> cornerPointsLessSharp;
	pcl::PointCloud<PointTypeSm> surfPointsFlat;
	pcl::PointCloud<PointTypeSm> surfPointsLessFlat;
	Eigen::Isometry3d out3d = Eigen::Isometry3d::Identity();
	pcl::PointCloud<PointTypeSm>::Ptr segmentedCloud(new pcl::PointCloud<PointTypeSm>);
	cout<<"start interation"<<endl;
	ofstream fout("log.txt");
	pcl::PCDWriter writer;
	float min_intensity;
	float max_intensity;
	util tools;
	//1.遍历所有点
	for(int i = 1;  i <file_names_ .size();i++){
		if (i>start_id && i<end_id) {
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
				
				if(cloud_bef->points[k].intensity>max_intensity){
					max_intensity = cloud_bef->points[k].intensity;
				}
				if(cloud_bef->points[k].intensity<min_intensity){
					min_intensity = cloud_bef->points[k].intensity;
				}
			}
			for (int k = 0; k < cloud_bef->size(); ++k) {
				cloud_bef->points[k].intensity = (cloud_bef->points[k].intensity)*1e43;
			}
			//结束调整反射率
			//开始采集感兴趣的区域
			if(tensorvoting){
				pcl::PassThrough<pcl::PointXYZI> pass;   //1. passthrough
				pass.setInputCloud (cloud_bef);
				pass.setFilterFieldName ("x");
				pass.setFilterLimits (-25, 25);
				pass.filter (*cloud_rot);
				pass.setInputCloud (cloud_rot);
				pass.setFilterFieldName ("y");
				pass.setFilterLimits (-50, 50);
				pass.filter (*cloud_bef);
				pass.setInputCloud (cloud_bef);
				pass.setFilterFieldName ("z");
				pass.setFilterLimits (-10, -1.8);
				pass.filter (*cloud_rot);
			}
			
			//计算两帧的增量
			*cloud_bef = *cloud_rot;
			out3d = trans_vector[i].matrix();
			out3d =  trans_vector[i-1].inverse().matrix() * out3d.matrix();
			
			fout <<i<< endl;
			fout <<"out3d.matrix()"<< endl;
			fout <<out3d.matrix()<< endl;
			//去畸变
			Feature.checkorder(cloud_bef,test);
			Feature.adjustDistortion(test,pcout,out3d);
			/*	//保存每贞
			 * std::stringstream path;
				path<<"map/"<<i<<".pcd";
				writer.write<PointTypeBeam>(path.str(),*pcout, true);
				  std::stringstream path1;
				  path1<<"map/"<<i<<"_.pcd";
				  writer.write<pcl::PointXYZI>(path1.str(),*cloud_bef, true);*/
			Feature.calculateSmoothness(pcout,segmentedCloud);
			Feature.calcFeature(segmentedCloud);
			//tmp用来转换格式
			tmp->clear();
			tmp->resize(pcout->size());
			for (int j = 0; j < pcout->size(); ++j) {
				tmp->points[j].x = pcout->points[j].x;
				tmp->points[j].y = pcout->points[j].y;
				tmp->points[j].z = pcout->points[j].z;//14.40
				tmp->points[j].intensity = pcout->points[j].intensity; }
			//todo 改成 第一步提取index 先,之后记录index,最后滤除
			//这里是地面滤波器
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
			
			if(!tensorvoting){
				if (i % 100 == 50){
					sor.setInputCloud(cloud_add);                   //设置需要过滤的点云给滤波对象
					sor.setLeafSize(0.05, 0.05, 0.05);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
					sor.filter(*cloud_aft);
					*cloud_add = *cloud_aft;
					std::cout.width(3);//i的输出为3位宽
					int percent = 0;
					int size_all = 0;
					size_all = static_cast<int>(file_names_ .size());
					percent =i*100/size_all;
					std::cout << percent << "%"<<std::endl;
					std::cout << "\b\b\b";//回删三个字符，使数字在原地变化
				}
			}
			int percent = 0;
			int size_all = 0;
			size_all = end_id - start_id;
			percent =i*100/size_all;
			std::cout << percent << "%"<<std::endl;
			std::cout << "\b \b \b ";//回删三个字符，使数字在原地变化
		}

	}
	fout.close();
	cout<<"end interation"<<endl;
	cout<<"max intenstity: "<<max_intensity<<" min intensity: "<<min_intensity<<std::endl;
	//转换回(0,0,0,0,0,0)
	pcl::transformPointCloud(*cloud_add, *cloud_aft, trans_vector[0].inverse().matrix());
	// 全点云时候应当加
	if(!tensorvoting){
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
		if(cloud_add->points[l].intensity>(float)14.4860 && cloud_add->points[l].intensity<(float)14.96){
			bigmap.push_back(cloud_add->points[l]);
		}
	}
	*cloud_add = bigmap ;
	cout<<"map saving"<<endl;
	writer.write<pcl::PointXYZI>("final_map.pcd",*cloud_add, true);
}

//7. 用来测试插值的 输入: 位姿vector
void testspline(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector){
	Eigen::Isometry3d lastodom,latest_odom,lidar_latest,temp_new;
	PoseGraphIO saveGraph;
	for (int j = 0; j < trans_vector.size()-3; ++j) {
		Eigen::Isometry3d t1,t2,t3,t4;
		t1 = trans_vector[j];
		t2 = trans_vector[j+1];
		t3 = trans_vector[j+2];
		t4 = trans_vector[j+3];
		double num =3;//两个位姿之间插几个
		for (double i = 0; i < num; ++i) {
			SplineFusion sf;
			saveGraph.insertPose(sf.cumulativeForm(t1,t2,t3,t4,i/num));
		}
	}
	//保存g2o
	saveGraph.saveGraph("/home/echo/test_ws/spline_g2o/a.g2o");
}

//8 feature map loam 特针点的地图
void genfeaturemap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
				   std::string filepath,pcl::PointCloud<pcl::PointXYZI>& bigmap){
	//0.初始化参数
	Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
	featureExtraction Feature;
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(
			new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI> cloud_rot_pc;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rot(
			new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<PointTypeBeam>::Ptr test (
			new pcl::PointCloud<PointTypeBeam>);
	pcl::PointCloud<PointTypeBeam>::Ptr pcout(
			new pcl::PointCloud<PointTypeBeam>);
	
	pcl::PointCloud<PointTypeSm> cornerPointsSharp,wholemap;
	pcl::PointCloud<PointTypeSm> cornerPointsLessSharp;
	pcl::PointCloud<PointTypeSm> surfPointsFlat;
	pcl::PointCloud<PointTypeSm> surfPointsLessFlat;
	pcl::PointCloud<PointTypeSm> tfedPC;
	Eigen::Isometry3d out3d = Eigen::Isometry3d::Identity();
	pcl::PointCloud<PointTypeSm>::Ptr segmentedCloud(new pcl::PointCloud<PointTypeSm>);
	cout<<"start interation"<<endl;
	pcl::PCDWriter writer;
	//todo 这里改过了,还没改回来接口
	//groundSeg gs;
	
	//	1.迭代添加点云 大循环********
	for(int i = 0;  i <file_names_ .size();i++){
		if (i>start_id && i<end_id) {
			
			std::cout<<"remains: "<<end_id-i<<std::endl;
			std::vector<int> indices1;
			//读点云
			pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], *cloud_bef);
			pcl::removeNaNFromPointCloud(*cloud_bef, *cloud_rot, indices1);
			out3d = trans_vector[i].matrix();
			out3d =  trans_vector[i-1].inverse().matrix() * out3d.matrix();
			pcl::copyPointCloud(*cloud_rot,cloud_rot_pc);
			//pcl::copyPointCloud(gs.point_cb(cloud_rot_pc),*cloud_bef);
			
			Feature.checkorder(cloud_bef,test);
			Feature.adjustDistortion(test,pcout,out3d);//输入点云 输出点云 相对tf
			Feature.calculateSmoothness(pcout,segmentedCloud);
			Feature.calcFeature(segmentedCloud);
			
			//把3m外的 特征点提取
			PointTypeSm temp;
			pcl::transformPointCloud(*Feature.cornerPointsSharp, tfedPC, trans_vector[i].matrix());
			for (int k = 0; k < tfedPC.size() ; ++k) {
				if(tfedPC.points[k].range>3){//过滤附近的点用的
					temp = tfedPC.points[k];
					temp.pointType = 1.0;
					cornerPointsSharp.points.push_back(temp) ;
				}
			}
			
			pcl::transformPointCloud(*Feature.cornerPointsLessSharp, tfedPC, trans_vector[i].matrix());
			for (int k = 0; k < tfedPC.size() ; ++k) {
				if(tfedPC.points[k].range>3){
					temp = tfedPC.points[k];
					temp.pointType = 2.0;
					cornerPointsLessSharp.points.push_back(temp) ;
				}
			}
			
			pcl::transformPointCloud(*Feature.surfPointsFlat, tfedPC, trans_vector[i].matrix());
			for (int k = 0; k < tfedPC.size() ; ++k) {
				if(tfedPC.points[k].range>3){
					temp = tfedPC.points[k];
					temp.pointType = 3.0;
					surfPointsFlat.points.push_back(temp) ;
				}
			}
			int wtf;
			wtf = surfPointsLessFlat.size();
			pcl::transformPointCloud(*Feature.surfPointsLessFlat, tfedPC, trans_vector[i].matrix());
			for (int k = 0; k < tfedPC.size() ; ++k) {
				if(tfedPC.points[k].range>3){
					temp = tfedPC.points[k];
					temp.pointType = 4.0;
					surfPointsLessFlat.points.push_back(temp) ;
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
	
	writer.write<PointTypeSm>("cornerPointsSharp.pcd",cornerPointsSharp, true);
	writer.write<PointTypeSm>("cornerPointsLessSharp.pcd",cornerPointsLessSharp, true);
	writer.write<PointTypeSm>("surfPointsFlat.pcd",surfPointsFlat, true);
	writer.write<PointTypeSm>("surfPointsLessFlat.pcd",surfPointsLessFlat, true);
	writer.write<PointTypeSm>("wholemap.pcd",wholemap, true);
	pcl::PointXYZI curPC;
	
	cout<<"end interation"<<endl;
	//转换回(0,0,0,0,0,0)
//	pcl::transformPointCloud(*cloud_add, *cloud_aft, trans_vector[0].inverse().matrix());
}

//6.1 omp NDT 配准
pcl::PointCloud<pcl::PointXYZI>::Ptr align(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration,
		const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud,
		const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud )
{
	registration->setInputTarget(target_cloud);
	registration->setInputSource(source_cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
	
	auto t2 = ros::WallTime::now();
	for(int i=0; i<10; i++) {
		registration->align(*aligned);
	}
	auto t3 = ros::WallTime::now();
	std::cout << "10times: " << (t3 - t2).toSec() * 1000 << "[msec]" << std::endl;
	std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;
	return aligned;
}

#endif //PCD_COMPARE_MAIN_H
