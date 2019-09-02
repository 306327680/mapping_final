// written by echo
//

#include "otherFunctions.h"
//G2O_USE_TYPE_GROUP(slam2d);

void genfeaturemap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
				 std::string filepath,pcl::PointCloud<pcl::PointXYZI>& bigmap){
	//0.初始化参数
	Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
	featureExtraction Feature;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(
			new pcl::PointCloud<pcl::PointXYZI>);
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
	
	//	1.迭代添加点云
	for(int i = 0;  i <file_names_ .size();i++){
		if (i>start_id && i<end_id) {
			std::vector<int> indices1;
			//读点云
			pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], *cloud_bef);
			pcl::removeNaNFromPointCloud(*cloud_bef, *cloud_rot, indices1);
			
			*cloud_bef = *cloud_rot;

			out3d = trans_vector[i].matrix();
			out3d =  trans_vector[i-1].inverse().matrix() * out3d.matrix();
			
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
					std::cout<<k<<std::endl;
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


int main(int argc,char** argv){

  if(argc != 3 && argc != 5 ){
    std::cout<<"argument error! argument number has to be 3! The first one shuold be pcd path the second should be g2o path"<<std::endl;
    std::cout<<"./pcd_reader /media/echo/35E4DE63622AD713/fushikang/loop_pcd_single /media/echo/35E4DE63622AD713/fushikang/lihaile.g2o "<<std::endl;
    return(-1);
  }
	
  filepath = argv[1];
  g2o_path = argv[2];
  if (argc == 5){
	  start_id = atoi(argv[3]);
	  end_id = atoi(argv[4]);
  }
  std::cout<<"start_id "<<start_id<<" end_id "<<end_id;
  GetFileNames(filepath,"pcd");
  trans_vector = getEigenPoseFromg2oFile(g2o_path);
  //生成局部地图
	lmOptimizationSufraceCorner lm;
	std::vector<double>  vector;
	Eigen::Isometry3d se3;
	for (int i = 0; i < trans_vector.size(); ++i) {
		std::cout<<trans_vector[i].matrix()<<std::endl;
		lm.eigen2RPYXYZ(trans_vector[i],vector);
		for (int j = 0; j < vector.size(); ++j) {
			std::cout<<vector[j]<<std::endl;
		}
		lm.RPYXYZ2eigen(vector,se3);
		std::cout<<se3.matrix()<<std::endl;
	}
 /* pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
  cout<<"trans_vector.size() : "<<trans_vector.size() <<" file_names_.size() : "<< file_names_.size()<<endl;
  if(trans_vector.size() == file_names_.size()){
	  genfeaturemap(trans_vector,filepath,*cloud1);
//      genlocalmap(trans_vector,filepath,*cloud1);
  } else{
      cout<<"!!!!! PCD & g2o does not have same number "<<endl;
      return 0;
  }*/

  return(0);
}
