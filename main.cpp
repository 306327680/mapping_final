// written by echo
//

#include "otherFunctions.h"
//G2O_USE_TYPE_GROUP(slam2d);
//生成所有的特征地图
//todo ground seg的地方还不能用

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

//设置输入模式: 1.g2o+pcd的传统模式(pcd+g2o路径) 2.point to plane ICP (需要提供pcd路径)
int status = 0;
//输入模式的函数
int getParam(int argc,char** argv){
	std::cout<<"设置建图模式: 1.g2o+pcd的传统模式(pcd+g2o路径) 2.point to plane ICP (需要提供pcd路径)"<<std::endl;
	cin >> status;
	std::cout <<"status: " <<status<<std::endl;
	if(status==1){
		if(argc != 3 && argc != 5 ){
			std::cout<<"argument error! argument number has to be 3/5! The first one should be pcd path the second should be g2o path"<<std::endl;
			std::cout<<"./pcd_reader /media/echo/35E4DE63622AD713/fushikang/loop_pcd_single /media/echo/35E4DE63622AD713/fushikang/lihaile.g2o "<<std::endl;
			std::cout<<"/media/echo/DataDisc/1_pcd_file/pku_bin /media/echo/DataDisc/2_g2o/pku/4edges_out.g2o 500 12210"<<std::endl;
			return(-1);
		}
		filepath = argv[1];
		g2o_path = argv[2];
		if (argc == 5){
			start_id = atoi(argv[3]);
			end_id = atoi(argv[4]);
		}
		std::cout<<"start_id "<<start_id<<" end_id "<<end_id;
	}
	if(status==2){
		if(argc != 2 ){
			std::cout<<"argument error! argument number has to be 2! The first one should be pcd path"<<std::endl;
			std::cout<<"e.g.: \n ./pcd_reader /media/echo/35E4DE63622AD713/fushikang/loop_pcd_single"<<std::endl;
			std::cout<<"输入pcd路径: "<<std::endl;
			cin >> filepath;
			cout<<filepath<<endl;
		} else{
			filepath = argv[1];
		}
	}
}

int g2omapping(){
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
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
	cout<<"trans_vector.size() : "<<trans_vector.size() <<" file_names_.size() : "<< file_names_.size()<<endl;
	if(trans_vector.size() == file_names_.size()){
		//genfeaturemap(trans_vector,filepath,*cloud1);
		genlocalmap(trans_vector,filepath,*cloud1);
	} else{
		cout<<"!!!!! PCD & g2o does not have same number "<<endl;
		return 0;
	}
}
int point2planeICP(){
	cout<<file_names_.back()<<endl;
	return(0);
}


int main(int argc,char** argv){

	//获得参数
  getParam(argc,argv);
  //得到所有的pcd名字
  GetFileNames(filepath,"pcd");
  
  switch(status)
  {
  	case 1 :
  		  g2omapping();
  		  cout << "g2o mapping start！" << endl;
  		  break;
  	case 2 :
  	 	  cout << "point to plane ICP start:" << endl;
		  point2planeICP();
		  cout << "point to plane ICP finish!" << endl;
  	 	  break;
	default :
	 	cout << "无效输入" << endl;
  }
  return(0);
}
