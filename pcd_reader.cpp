// written by echo
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter.h>
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
#include "beam_separate.h"

//G2O_USE_TYPE_GROUP(slam2d);

using namespace g2o;
std::vector<std::string> file_names_;
std::string filename;
std::string filepath = "/media/echo/35E4DE63622AD713/test/";
Eigen::Isometry3d curICP = Eigen::Isometry3d::Identity();
int cur_id = 0;
//打开g2o文件路径
std::string g2o_path = "null";
int past = 0;
//存g2o路径
std::string save_g2o_path = "/home/echo/small_program/test.g2o";
//存储g2o为iso3d
std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector;

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
  return true;
}

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

//读取g2o文件
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

/*#define PointCloud_T  pcl::PointCloud<pcl::PointXYZ>

#define PointCloudPtr_T pcl::PointCloud<pcl::PointXYZ>::Ptr
#define PointCloudConstPtr_T pcl::PointCloud<pcl::PointXYZ>::ConstPtr
PointCloud_T GICP(const PointCloud_T & cloud_source,
                     const PointCloud_T & cloud_target,
                            Eigen::Isometry3d & icp_matrix) {
    PointCloud_T result;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp; //创建ICP对象，用于ICP配准
    //gicp.setRotationEpsilon()

    gicp.setTransformationEpsilon(0.01);
    gicp.setMaximumIterations (100);
    gicp.setInputSource(PointCloudConstPtr_T(new PointCloud_T(cloud_source)));
    gicp.setInputTarget(PointCloudConstPtr_T(new PointCloud_T(cloud_target)));
    gicp.align(result);
    Eigen::Matrix4f tf_s2t = gicp.getFinalTransformation();
    icp_matrix = tf_s2t.cast<double>();
    std::cout<<tf_s2t<<std::endl;
    return result; // source上应用这个矩阵，就可以转过去了
}*/


//(1). 读trans和附近pcd拼成点云
//1.trans vector 2.查找的位置 3.点云读取的路径
void genlocalmap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
                 std::string filepath,pcl::PointCloud<pcl::PointXYZI>& bigmap){
    Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
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

    for(int i = 1;  i <file_names_ .size();i++){
//      if(i>1273 || i <933 ){ //fsk big map
      if (i>4000 && i<7000) {
        std::vector<int> indices1;
        //读点云
        pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], *cloud_bef);
        pcl::removeNaNFromPointCloud(*cloud_bef, *cloud_rot, indices1);
        *cloud_bef = *cloud_rot;
        
        out3d = trans_vector[i].matrix();
        out3d =  trans_vector[i-1].inverse().matrix() * out3d.matrix();
		  
        fout <<i<< endl;
        fout <<"out3d.matrix()"<< endl;
        fout <<out3d.matrix()<< endl;
        Feature.checkorder(cloud_bef,test);
        Feature.adjustDistortion(test,pcout,out3d);
	/*	std::stringstream path;
		path<<"map/"<<i<<".pcd";
        writer.write<PointTypeBeam>(path.str(),*pcout, true);
		  std::stringstream path1;
		  path1<<"map/"<<i<<"_.pcd";
		  writer.write<pcl::PointXYZI>(path1.str(),*cloud_bef, true);*/
		Feature.calculateSmoothness(pcout,segmentedCloud);
		Feature.calcFeature(segmentedCloud);
		
        tmp->clear();
        tmp->resize(pcout->size());
        for (int j = 0; j < pcout->size(); ++j) {
          tmp->points[j].x = pcout->points[j].x;
          tmp->points[j].y = pcout->points[j].y;
          tmp->points[j].z = pcout->points[j].z;
          tmp->points[j].intensity = pcout->points[j].intensity; }

        pcl::VoxelGrid<pcl::PointXYZI> sor;
     /*   sor.setInputCloud(tmp);                   //设置需要过滤的点云给滤波对象
        sor.setLeafSize(0.1, 0.1, 0.1);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
        sor.filter(*cloud_aft);*/
        //new
        *cloud_bef = *tmp;
        pcl::transformPointCloud(*cloud_bef, *cloud_aft, trans_vector[i].matrix());
        *cloud_add += *cloud_aft;
	/*	  pcl::transformPointCloud(*Feature.cornerPointsSharp, surfPointsLessFlat, trans_vector[i].matrix());
		  for (int k = 0; k < surfPointsLessFlat.size() ; ++k) {
		  	if(surfPointsLessFlat.points[k].range>3){
				cornerPointsSharp.points.push_back(surfPointsLessFlat.points[k]) ;
		  	}
		  }
		  pcl::transformPointCloud(*Feature.cornerPointsLessSharp, surfPointsLessFlat, trans_vector[i].matrix());
		  for (int k = 0; k < surfPointsLessFlat.size() ; ++k) {
			  if(surfPointsLessFlat.points[k].range>3){
			  cornerPointsLessSharp.points.push_back(surfPointsLessFlat.points[k]) ;
		  		}
		  }
		  pcl::transformPointCloud(*Feature.cornerPointsSharp, surfPointsLessFlat, trans_vector[i].matrix());
		  for (int k = 0; k < surfPointsLessFlat.size() ; ++k) {
			  if(surfPointsLessFlat.points[k].range>3){
		  		surfPointsFlat.points.push_back(surfPointsLessFlat.points[k]) ;
			  }
		  }*/
		  
        if (i % 100 == 50){
          sor.setInputCloud(cloud_add);                   //设置需要过滤的点云给滤波对象
          sor.setLeafSize(0.2, 0.2, 0.2);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
          sor.filter(*cloud_aft);
          *cloud_add = *cloud_aft;
/*          std::cout.width(3);//i的输出为3位宽
			int percent = 0;
			int size_all = 0;
			size_all = static_cast<int>(file_names_ .size());
			percent =i*100/size_all;
			std::cout << percent << "%"<<std::endl;
			std::cout << "\b\b\b\b";//回删三个字符，使数字在原地变化*/
        }
          cloud_aft->clear();
          cloud_bef->clear();
      }
    }
    fout.close();
/*	writer.write<PointTypeSm>("cornerPointsSharp.pcd",cornerPointsSharp, true);
	writer.write<PointTypeSm>("cornerPointsLessSharp.pcd",cornerPointsLessSharp, true);
	writer.write<PointTypeSm>("surfPointsFlat.pcd",surfPointsFlat, true);*/
    cout<<"end interation"<<endl;
    //转换回(0,0,0,0,0,0)
    pcl::transformPointCloud(*cloud_add, *cloud_aft, trans_vector[0].inverse().matrix());
    // 全点云时候应当加
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud_aft);
  	sor.setLeafSize(0.2, 0.2, 0.2);
    sor.filter(*cloud_add);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // outrem.setInputCloud(cloud_add);
    // outrem.setRadiusSearch(0.5);
    // outrem.setMinNeighborsInRadius (5);
    // // 应用滤波器
    // outrem.filter (*cloud_aft);
    bigmap = *cloud_add;
}

int main(int argc,char** argv){

  if(argc != 3 ){
    std::cout<<"argument error! argument number has to be 3! The first one shuold be pcd path the second should be g2o path"<<std::endl;
    std::cout<<"./pcd_reader /media/echo/35E4DE63622AD713/fushikang/loop_pcd_single /media/echo/35E4DE63622AD713/fushikang/lihaile.g2o "<<std::endl;
    return(-1);
  }

  filepath = argv[1];
  g2o_path = argv[2];
  GetFileNames(filepath,"pcd");
  trans_vector = getEigenPoseFromg2oFile(g2o_path);
  //生成局部地图
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
  cout<<"trans_vector.size() : "<<trans_vector.size() <<" file_names_.size() : "<< file_names_.size()<<endl;
  if(trans_vector.size() == file_names_.size()){
      genlocalmap(trans_vector,filepath,*cloud1);
  } else{
      cout<<"!!!!! PCD & g2o does not have same number "<<endl;
      return 0;
  }

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI>("final_map.pcd",*cloud1, true);

  return(0);
}
