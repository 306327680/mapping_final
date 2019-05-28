// written by echo
//

#include "main.h"
//G2O_USE_TYPE_GROUP(slam2d);

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
	pcl::PointCloud<PointTypeSm> cornerPointsSharp;
	pcl::PointCloud<PointTypeSm> cornerPointsLessSharp;
	pcl::PointCloud<PointTypeSm> surfPointsFlat;
	pcl::PointCloud<PointTypeSm> surfPointsLessFlat;
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
			pcl::transformPointCloud(*Feature.cornerPointsSharp, surfPointsLessFlat, trans_vector[i].matrix());
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
			}
			cloud_bef->clear();
		}
	}

	writer.write<PointTypeSm>("cornerPointsSharp.pcd",cornerPointsSharp, true);
	writer.write<PointTypeSm>("cornerPointsLessSharp.pcd",cornerPointsLessSharp, true);
	writer.write<PointTypeSm>("surfPointsFlat.pcd",surfPointsFlat, true);
	writer.write<PointTypeSm>("surfPointsLessFlat.pcd",surfPointsLessFlat, true);
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
  cout<<"trans_vector.size() : "<<trans_vector.size() <<" file_names_.size() : "<< file_names_.size()<<endl;
  if(trans_vector.size() == file_names_.size()){
	  genfeaturemap(trans_vector,filepath,*cloud1);
//      genlocalmap(trans_vector,filepath,*cloud1);
  } else{
      cout<<"!!!!! PCD & g2o does not have same number "<<endl;
      return 0;
  }

  return(0);
}
