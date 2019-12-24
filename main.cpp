// written by echo
//

#include "otherFunctions.h"
//G2O_USE_TYPE_GROUP(slam2d);
//生成所有的特征地图
//todo ground seg的地方还不能用



//设置输入模式: 1.g2o+pcd的传统模式(pcd+g2o路径) 2.point to plane ICP (需要提供pcd路径)
int status = 0;
//输入模式的函数
int getParam(int argc,char** argv){
	std::cout<<"设置建图模式: 1.g2o+pcd的传统模式(pcd+g2o路径) 2.point to plane ICP (需要提供pcd路径) 3.bpreal ground mapping (pcd+g2o路径)"
	<<"4. 使用编码器和GPS LiDAR 建图"<<"5. LiDAR gps 外参标定"<<"6. NDT maping"<<std::endl;
	cin >> status;
	std::cout <<"status: " <<status<<std::endl;
	if(status==1||status==3){
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
//功能1 用g2o pose 建图
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
	//样条插值
	testspline(trans_vector);
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
//设置起始结束的pcd
void setStartEnd(){
	std::cout<<"设置起始pcd"<<std::endl;
	cin >> start_id;
	std::cout <<"start: " <<start_id<<std::endl;
	std::cout<<"设置结束pcd"<<std::endl;
	cin >> end_id;
	std::cout <<"end: " <<end_id<<std::endl;
}
void pointToPlaneICP(pcl::PointCloud<pcl::PointXYZI> pcin,Eigen::Isometry3d& tf){

}

//功能2建图前端 点面icp
int point2planeICP(){
	//点云缓冲
	std::vector<pcl::PointCloud<pcl::PointXYZI>> localmap;
	pcl::PointCloud<pcl::PointXYZI> tfed;
	pcl::PointCloud<pcl::PointXYZI> nonan;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(new pcl::PointCloud<pcl::PointXYZI>);
//	PPointCloud::Ptr cloud_hesai(new PPointCloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_local_map(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr filter(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr raw(new pcl::PointCloud<pcl::PointXYZINormal>);
	bool bperal_edge = false;
	featureExtraction Feature;//去畸变
	util tools;
	registration icp;
	icp.setParam("/media/echo/DataDisc/3_program/mapping/cfg/icp.yaml");
	icp.SetNormalICP();
	pcl::PCDWriter writer;
 	bool first_cloud = true;
 	std::cout<<file_names_ .size()<<std::endl;
	std::cout<<start_id<<" "<<end_id<<std::endl;
	//去畸变
	pcl::PointCloud<PointTypeBeam>::Ptr test (
			new pcl::PointCloud<PointTypeBeam>);
	pcl::PointCloud<PointTypeBeam>::Ptr pcout(
			new pcl::PointCloud<PointTypeBeam>);
	
	
	for(int i = 1;  i <file_names_ .size();i++){
		if (i>start_id && i<end_id) {
			pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], *cloud_hesai);
			std::vector<int> indices1;
			pcl::removeNaNFromPointCloud(*cloud_hesai, nonan, indices1);
			pcl::copyPointCloud(nonan,*cloud_hesai);
			std::cout<<"scan size: "<<cloud_bef->size() <<std::endl;
			if(bperal_edge){
/*				//转换格式
				for (int j = 0; j < cloud_bef.size(); ++j) {
					pcl::PointXYZINormal aa;
					aa.x = cloud_bef[j].x;
					aa.y = cloud_bef[j].y;
					aa.z = cloud_bef[j].z;
					aa.intensity = cloud_bef[j].intensity;
					//得到延迟时间
					aa.normal_y = j%(cloud_bef.size()/32);
					raw->push_back(aa);
				}
				//下面的代码测试 提取线的边缘
				pcl::removeNaNFromPointCloud(*raw, *filter, indices1);
				tools.GetPointCloudBeam(*filter,*result);
				tools.GetBeamEdge(*filter,*result);*/
			}else{
				//这里用pcl的 plane to plane icp
				
				if(first_cloud){
					std::cout<<"first cloud" <<std::endl;
					pcl::copyPointCloud(*cloud_hesai,*cloud_local_map);
					localmap.push_back(*cloud_local_map);
					first_cloud = false;
				} else{
					//todo加个去畸变
					pcl::copyPointCloud(*cloud_hesai,*cloud_bef);
					tfed = icp.normalIcpRegistration(cloud_bef,*cloud_local_map);
					localmap.push_back(tfed);
					//下面是生成5帧的local map
					cloud_local_map->clear();
					if(localmap.size()>1){
						std::vector<pcl::PointCloud<pcl::PointXYZI>> localmap_temp;
						localmap_temp.push_back(localmap[1]);
			/*			localmap_temp.push_back(localmap[2]);
						localmap_temp.push_back(localmap[3]);
						localmap_temp.push_back(localmap[4]);
						localmap_temp.push_back(localmap[5]);*/
						localmap = localmap_temp;
					}
					for (int j = 0; j < localmap.size(); ++j) {
						*cloud_local_map += localmap[j];
					}
					*cloud_map += tfed;
					std::cout<<"scan: "<<i<<" map size: "<<cloud_local_map->size() <<std::endl;
				}
			}
		}
	}
 
	writer.write("test2.pcd",*cloud_map, true);
	return(0);
}
//读取hesaipcd
void readAndSaveHesai(std::string path){
	ReadBag a;
	a.readHesai(path);
	
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

//功能5.1 LiDAR + GNSS mapping 标定
void LiDARGNSScalibration (){
	pcl::PCDWriter writer;
	ReadBag rb;
	rb.gnssLiDARExtrinsicParameters("/media/echo/DataDisc/9_rosbag/test_gps_lidar_calibration/2019-10-23-18-48-24.bag");
	//测试: 存储GPS + Encoder 到点云
	writer.write("/home/echo/encoder.pcd",rb.encoder_pcd);
	writer.write("/home/echo/gps_pcd.pcd",rb.gps_pcd);
}
//功能5.2 LiDAR + GNSS mapping 建图
void LiDARGNSSMapping(){

}
//功能6. NDT mapping
void NDTmapping(){

}
//intensity 的 edge 可不可以检测的到
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
		  setStartEnd();
  	 	  cout << "point to plane ICP start:" << endl;
		  //readAndSaveHesai("/media/echo/DataDisc/9_rosbag/rsparel_64_ins/2019-11-06-20-43-12_0.bag");
		  point2planeICP();
		  cout << "point to plane ICP finish!" << endl;
  	 	  break;
  	case 3 :
  		  traversableMapping();
		  cout << "traversable Mapping finish!" << endl;
		  break;
	case 4 :
		encoderMapping();
		break;
  	case 5:
  		//Calibrating the extrinsic parameters
		LiDARGNSScalibration();
		LiDARGNSSMapping();
		break;
	case 6:
		NDTmapping();
		break;
	default :
	 	cout << "无效输入" << endl;
  }
  return(0);
}
