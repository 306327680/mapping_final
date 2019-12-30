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
	if(status==2||status==6){
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
void simpleDistortion(mypcdCloud input,Eigen::Matrix4f increase,pcl::PointCloud<pcl::PointXYZI>& output){
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
	pcl::PointCloud<PointTypeBeam>::Ptr test (new pcl::PointCloud<PointTypeBeam>);
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
		temp.pctime = input[i].timestamp*10;
		temp.beam = input[i].ring;
		test->push_back(temp);
	}
	
	Feature.adjustDistortion(test,pcout,se3);
	pcl::copyPointCloud(*pcout,output);
}
//功能2建图前端 点面icp
int point2planeICP(){
	//点云缓冲
	std::vector<pcl::PointCloud<pcl::PointXYZI>> localmap;
	pcl::PointCloud<pcl::PointXYZI> tfed;
	pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI> nonan;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(new pcl::PointCloud<pcl::PointXYZI>);
	mypcdCloud xyzItimeRing; //现在改了之后的点
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai(new pcl::PointCloud<pcl::PointXYZI>);//io 进来的点
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_local_map(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr filter(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr raw(new pcl::PointCloud<pcl::PointXYZINormal>);
	bool bperal_edge = false;
	//存tf的
	std::vector<Eigen::Matrix4f> poses;
	//每两帧之间的变换
	std::vector<Eigen::Matrix4f> icp_result;
	Eigen::Matrix4f current_scan_pose = Eigen::Matrix4f::Identity();
//class
	util tools;
	registration icp;
	icp.setParam("/media/echo/DataDisc/3_program/mapping/cfg/icp.yaml");
	icp.SetNormalICP();
	pcl::PCDWriter writer;
 	bool first_cloud = true;
 	bool distortion = true;
 	std::cout<<file_names_ .size()<<std::endl;
	std::cout<<start_id<<" "<<end_id<<std::endl;
	
	for(int i = 1;  i <file_names_ .size();i++){
		if (i>start_id && i<end_id) {
			std::cout<<"*************点云ID: "<<i<<std::endl;
			pcl::io::loadPCDFile<mypcd>(file_names_[i], xyzItimeRing);
			pcl::copyPointCloud(xyzItimeRing,*cloud_hesai);
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
				//1. 这里用pcl的 plane to plane icp
				if(first_cloud){
					//1.1 第一帧 不进行计算
					std::cout<<"first cloud" <<std::endl;
					pcl::copyPointCloud(*cloud_hesai,*cloud_local_map);
					localmap.push_back(*cloud_local_map);
					first_cloud = false;
				} else{
					//2.1 加个去畸变
					
					if(distortion){
						simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
//						std::cout<<"畸变用的位移:\n"<<icp.increase.inverse()<<std::endl;
					}else{
						pcl::copyPointCloud(*cloud_hesai,*cloud_bef);
					}
					
					//2.2 输入的也应该降采样
					pcl::UniformSampling<pcl::PointXYZI> filter;
					pcl::PointCloud<int> keypointIndices;
					filter.setInputCloud(cloud_bef);
					filter.setRadiusSearch(0.05f); //0.1米
					filter.compute(keypointIndices);
					pcl::copyPointCloud(*cloud_bef, keypointIndices.points, *cloud_filtered);
					*cloud_bef = *cloud_filtered;
					keypointIndices.clear();
					//2.3 进行point 2 plane ICP **********
					tfed = icp.normalIcpRegistration(cloud_bef,*cloud_local_map);
					icp_result.push_back(icp.increase);
 					//todo 不应该这样,应该都放到000附近
					//2.3.1 再次去畸变
					simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
					pcl::transformPointCloud(*cloud_bef,tfed,icp.transformation.matrix());//去了
					//2.3.2 再次icp           ***********
					tfed = icp.normalIcpRegistrationlocal(cloud_bef,*cloud_local_map);
					poses.push_back(icp.transformation.matrix());
					icp_result.back() = icp_result.back()*icp.pcl_plane_plane_icp->getFinalTransformation(); //第一次结果*下一次去畸变icp结果
					//2.4把最后去畸变结果的点云放进去
					localmap.push_back(tfed);
					//2.5下面维护 5帧的local map
					cloud_local_map->clear();
					if(localmap.size()>1){
						std::vector<pcl::PointCloud<pcl::PointXYZI>> localmap_temp;
						for (int j = 1; j < localmap.size(); ++j) {
							localmap_temp.push_back(localmap[j]);
						}
						localmap = localmap_temp;
					}
					for (int j = 0; j < localmap.size(); j=j+1) {
						*cloud_local_map += localmap[j];
					}
					//map filter start
					filter.setInputCloud(cloud_local_map);
					filter.setRadiusSearch(0.05f); //0.1米
					filter.compute(keypointIndices);
					pcl::copyPointCloud(*cloud_local_map, keypointIndices.points, *filteredCloud);
					*cloud_local_map = *filteredCloud;
					//filter end
					
					//生成地图
					Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();
					//试一下这样恢复出来的位姿
					for (int k = 0; k < icp_result.size(); ++k) {
						current_pose *= icp_result[k];
					}
					std::cout<<"恢复位姿\n"<<current_pose.matrix()<<std::endl;
					pcl::transformPointCloud(*cloud_hesai,tfed,current_pose);
					*cloud_map += tfed;
					std::cout<<"scan: "<<i<<" map size: "<<cloud_local_map->size() <<std::endl;
				}
			}
		}
	}
	writer.write("testdistro.pcd",*cloud_map, true);
	//可视化一下
	pcl::visualization::PCLVisualizer vis("PlICP");
 
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> aligned_handler(cloud_map, "intensity");
	vis.addPointCloud(cloud_map, aligned_handler, "map");
	vis.spin();
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
  	 	  //读bag的功能测试ok
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
		setStartEnd();
		NDTmapping();
		break;
	default :
	 	cout << "无效输入" << endl;
  }
  return(0);
}
