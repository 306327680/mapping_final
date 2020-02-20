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
//功能1 用g2o pose 建图*******888888
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
//6.1.2 转换一个点的坐标
void transformOnePoint(Eigen::Matrix4f t,pcl::PointXYZI & input){
	Eigen::Matrix<float,4,1> temp_point,result;
	temp_point(0,0) = input.x;
	temp_point(1,0) = input.y;
	temp_point(2,0) = input.z;
	temp_point(3,0) = 1;
	result = t*temp_point;
	input.x = result(0,0);
	input.y = result(1,0);
	input.z = result(2,0);
}
//6.1.1 对4个位姿的点进行连续去畸变 自己维护一下序列()
//input 4个位姿 从t-1 到t+2 输入 t时刻的点云
//输出 continusTime 去过畸变的点云
pcl::PointCloud<pcl::PointXYZI> continusTimeDistrotion(std::vector<Eigen::Matrix4f> & poses, std::vector<mypcdCloud> & clouds){
	Eigen::Isometry3d t1,t2,t3,t4,current_trans;
	pcl::PointCloud<pcl::PointXYZI> result;
	pcl::PointXYZI temp;
	std::vector<Eigen::Matrix4f> poses_tmp;
	std::vector<mypcdCloud> clouds_tmp;
	double num =3;//两个位姿之间插几个
	SplineFusion sf;

	//维持队列
	if(poses.size() == clouds.size()){
		if(poses.size()<4){
			printf("等待4次配准结果\n");
			return result;
		}
		for (int j = poses.size()-4; j < poses.size(); ++j) {
			poses_tmp.push_back(poses[j]);
			clouds_tmp.push_back(clouds[j]);
		}
		poses = poses_tmp;
		clouds = clouds_tmp;
	} else{
		printf("pose clouds 不相等\n");
		return result;
	}

	//转存位姿点
	t1 = poses[0].matrix().cast<double>();
	t2 = poses[1].matrix().cast<double>();
	t3 = poses[2].matrix().cast<double>();
	t4 = poses[3].matrix().cast<double>();
		
	for (int i = 0; i < clouds[1].size(); ++i) {
		current_trans = sf.cumulativeForm(t1,t2,t3,t4,clouds[clouds.size()-3][i].timestamp*10); //10为 10 帧每秒
		temp.x = clouds[1][i].x;
		temp.y = clouds[1][i].y;
		
		temp.z = clouds[1][i].z;
		temp.intensity = clouds[1][i].intensity;
		transformOnePoint(current_trans.matrix().cast<float>(),temp);
		result.push_back(temp);
		
	}
	printf("continue-time 完成\n");
	return result;
}
//6.1 tools 建立局部地图
//a. 挑选关键帧(有必要?)
//b. 维持队列长度
//d. downsample
// 15 帧大概1.2 s
pcl::PointCloud<pcl::PointXYZI> lidarLocalMap(std::vector<Eigen::Matrix4f> & poses,std::vector<pcl::PointCloud<pcl::PointXYZI>> & clouds){
	pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI> map_temp;
	std::vector<Eigen::Matrix4f>  poses_tmp;
	std::vector<pcl::PointCloud<pcl::PointXYZI>>  clouds_tmp;
	Eigen::Matrix4f tf_all = Eigen::Matrix4f::Identity();
	//局部地图采用 continues time 的方式生成 先只在10帧上做
	
	//note 15帧64线 0.02 大概225520个点 //10帧试试
	if (poses.size()>=15){
		for (int i = poses.size() - 15; i <poses.size(); i++) {
			pcl::transformPointCloud(clouds[i],map_temp,poses[i]);
			*map_ptr += map_temp;
			poses_tmp.push_back(poses[i]);
			clouds_tmp.push_back(clouds[i]);
		}
		poses = poses_tmp;
		clouds = clouds_tmp;
	}else if(poses.size()>1){//第一开始点云不多的情况 不要第一个帧
		for (int i = 1 ; i <poses.size() ; i++) {
			pcl::transformPointCloud(clouds[i],map_temp,poses[i]);
			*map_ptr += map_temp;
		}
	} else{
		pcl::transformPointCloud(clouds[0],*map_ptr,poses[0]);
	}

	pcl::transformPointCloud(*map_ptr,map_temp,poses.back().inverse());
	*map_ptr = map_temp;
	//显示看一下
	util tools;
	tools.timeCalcSet("降采样的时间");
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	sor.setInputCloud(map_ptr);
	sor.setLeafSize(0.15f, 0.15f, 0.15f);
	sor.filter(map_temp);
	
/*	pcl::UniformSampling<pcl::PointXYZI> filter;
	pcl::PointCloud<int> keypointIndices;
	filter.setInputCloud(map_ptr);
	filter.setRadiusSearch(0.15f); //2cm 测距精度 015 haixing
	filter.compute(keypointIndices);
	pcl::copyPointCloud(*map_ptr, keypointIndices.points, map_temp);*/
	//pointCloudRangeFilter(map_temp,75); //距离滤波可以去了
	tools.timeUsed();
	std::cout<<" 局部地图大小: "<<map_temp.size() <<std::endl;
	return  map_temp;
}

//6.2 建图前端 点面icp
int point2planeICP(){
	//g2o结果存储
	PoseGraphIO g2osaver;
	pcl::PointCloud<pcl::PointXYZI> tfed;
	pcl::PointCloud<pcl::PointXYZI> cloud_continus_time_T_world;
	pcl::PointCloud<pcl::PointXYZI> cloud_continus_time_T_LiDAR;
	pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI> nonan;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(new pcl::PointCloud<pcl::PointXYZI>);
	mypcdCloud xyzItimeRing; //现在改了之后的点
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai(new pcl::PointCloud<pcl::PointXYZI>);//io 进来的点
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_local_map(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZI>);				//线性去畸变的图
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
	ros::Publisher test_frame_linear;
	test_frame = node.advertise<sensor_msgs::PointCloud2>("/local_map", 5);
	test_frame_linear = node.advertise<sensor_msgs::PointCloud2>("/current_frame_linear", 5);
	//todo end 这里可以去掉ros
	//存tf的
	std::vector<Eigen::Matrix4f> poses;
	std::vector<pcl::PointCloud<pcl::PointXYZI>>  clouds;
	//每两帧之间的变换
	std::vector<Eigen::Matrix4f> icp_result;
	Eigen::Matrix4f current_scan_pose = Eigen::Matrix4f::Identity();
	//滤波相关
	pcl::UniformSampling<pcl::PointXYZI> filter;
	pcl::PointCloud<int> keypointIndices;
	//车速
	float curr_speed = 0;
	//continus-time distortion
	std::vector<Eigen::Matrix4f>  poses_distortion;
	std::vector<mypcdCloud> clouds_distortion_origin;
//class
	util tools,tools2;
	registration icp;
	icp.setParam("/media/echo/DataDisc/3_program/mapping/cfg/icp.yaml");
	icp.SetNormalICP();
	pcl::PCDWriter writer;
	bool first_cloud = true;
	bool distortion = true;
	std::cout<<file_names_ .size()<<std::endl;
	std::cout<<start_id<<" "<<end_id<<std::endl;
	
	for(int i = 0;  i <file_names_ .size();i++){
		if (i>start_id && i<end_id) {
			tools2.timeCalcSet("total");
			pcl::io::loadPCDFile<mypcd>(file_names_[i], xyzItimeRing);
			pcl::copyPointCloud(xyzItimeRing,*cloud_hesai);
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
				//2.1 加个去畸变
				simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
				//2.1.1 设为普通icp********
				icp.transformation = Eigen::Matrix4f::Identity();
				//2.2 输入的也应该降采样
				filter.setInputCloud(cloud_bef);
				filter.setRadiusSearch(0.02f); //2cm 根据测距精度设置的
				filter.compute(keypointIndices);
				pcl::copyPointCloud(*cloud_bef, keypointIndices.points, *cloud_filtered);
				*cloud_bef = *cloud_filtered;
				//(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
				keypointIndices.clear();
				//2.3 进行point 2 plane ICP ********** 去3次畸变2次icp ****&&&&这个当做 lidar odom
				tools.timeCalcSet("第一次ICP用时     ");
				tfed = icp.normalIcpRegistration(cloud_bef,*cloud_local_map);
				icp_result.push_back(icp.increase);
				//2.3.1 再次去畸变 再次减小范围
				simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
				//pointCloudRangeFilter(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
				tools.timeUsed();
				//2.3.2 再次icp           *********** &&&&这个当做 lidar mapping
				tools.timeCalcSet("局部地图用时    ");
				*cloud_local_map = lidarLocalMap(poses,clouds);  //生成局部地图****
				tools.timeUsed();
				tools.timeCalcSet("第二次ICP用时    ");
				tfed = icp.normalIcpRegistrationlocal(cloud_bef,*cloud_local_map);
				icp_result.back() = icp_result.back()*icp.pcl_plane_plane_icp->getFinalTransformation(); //第一次结果*下一次去畸变icp结果
				//2.3.3 再次去畸变 再次减小范围
				simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
				//pointCloudRangeFilter(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
				tools.timeUsed();
				//2.4 speed
				curr_speed = sqrt(icp_result.back()(0,3)*icp_result.back()(0,3)+
								  icp_result.back()(1,3)*icp_result.back()(1,3))/0.1;
				std::cout<<"*****点云ID: "<<i<<" ***** speed: "<<3.6*curr_speed<<" km/h ********"<<std::endl;
				*cloud_local_map = *cloud_bef;
				//可以用恢复出来的位姿 tf 以前的点云
				clouds.push_back(*cloud_bef);
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
				//运行最终的去畸变
				if(0){ //存大点云
					std::cout<<"全局坐标 \n"<<current_pose.matrix()<<std::endl;
					pcl::transformPointCloud(*cloud_bef,tfed,current_pose);
					*cloud_map += tfed;
			/*		tools.timeCalcSet("连续时间去畸变用时:    ");
					cloud_continus_time_T_world = continusTimeDistrotion(poses_distortion,clouds_distortion_origin);//这里放的是最新的一帧和位姿
					*cloud_map_continus_time += cloud_continus_time_T_world;
					*//*			if (poses_distortion.size() == 4){ //进行了连续时间的去畸变就替换这个 转换到最新的T的坐标系下面
									pcl::transformPointCloud(cloud_continus_time_T_world,cloud_continus_time_T_LiDAR,current_pose.matrix().inverse());
									clouds[clouds.size()-3] = cloud_continus_time_T_LiDAR;
								}*//*
					tools.timeUsed();*/
					//todo 这里可以去掉ros
	/*				pcl::toPCLPointCloud2(*cloud_map_continus_time, pcl_frame);
					pcl_conversions::fromPCL(pcl_frame, to_pub_frame);
					to_pub_frame.header.frame_id = "/map";
					test_frame.publish(to_pub_frame);*/
	
					pcl::toPCLPointCloud2(tfed, pcl_frame);
					pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
					to_pub_frame_linear.header.frame_id = "/map";
					test_frame_linear.publish(to_pub_frame_linear);
					
					pcl::toPCLPointCloud2(*cloud_local_map, pcl_frame);
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
	g2osaver.saveGraph("/media/echo/DataDisc/3_program/mapping/cmake-build-debug/g2o/icp.g2o");
	writer.write("cloud_map.pcd",*cloud_map, true);
	//writer.write("distro_final.pcd",*cloud_map_continus_time, true);
	//可视化一下
/*	pcl::visualization::PCLVisualizer vis("PlICP");
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> aligned_handler(cloud_map, "intensity");
	vis.addPointCloud(cloud_map, aligned_handler, "map");
	vis.spin();*/
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

//功能5.1 LiDAR + GNSS mapping 标定
//程序准备设计的方案: 1. 读取一个bag 取其中的一段时间 eg 60s 600 帧进行 odom的计算
//2. 读取gps 转成lla
//3. 时间戳对齐
//4. 设置两个优化变量 一个是两个传感器的外参 一个是旋转的角度,就是雷达初始时刻相对于正北的朝向;
//5.
// /media/echo/DataDisc/9_rosbag/rsparel_64_ins 这个好像能work gps时间戳不太对 就用时间戳减1s
void LiDARGNSScalibration (){
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
	pcl::io::loadPCDFile<pcl::PointXYZ>("/home/echo/Desktop/gnss.pcd", *gps_position); //gnssPCDExtrinsicParameters 函数得到的gnss轨迹
	trans_vector = getEigenPoseFromg2oFile("/home/echo/Desktop/icp.g2o");//通过功能2 得到的位置
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
void readAndSaveHesai(std::string path){
	ReadBag a;
	a.readHesai(path);
}

//功能8 用来测试模块好使不
void testFunction(){

}
//功能9 普通的16线建图
void rslidarmapping(){
	registration icp;
	pcl::PointCloud<pcl::PointXYZI> xyzItimeRing;
	for(int i = 0;  i <file_names_ .size();i++){
		pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], xyzItimeRing);
	}
}
//intensity 的 edge 可不可以检测的到
int main(int argc,char** argv){
	//todo 这里可以去掉ros
	ros::init(argc, argv, "map");
	//获得参数
	getParam(argc,argv);
	//得到所有的pcd名字
	GetFileNames(filepath,"pcd");
  
	switch(status)
	{
		//1. g2o+pcd拼图
		case 1 :
			g2omapping();
			cout << "g2o mapping start！" << endl;
			break;
		case 2 :
			//2. 对于何塞雷达的建图 需要有时间戳 和线数信息
			setStartEnd();
			cout << "point to plane ICP start:" << endl;
			//读bag的功能测试ok
			//readAndSaveHesai("/media/echo/DataDisc/9_rosbag/rsparel_64_ins/2019-11-06-20-43-12_0.bag");
			point2planeICP();//普通点面icp
			cout << "point to plane ICP finish!" << endl;
			break;
		case 3 ://3. 可通行区域建图
			traversableMapping();
			cout << "traversable Mapping finish!" << endl;
			break;
		case 4 ://4. 利用编码器建图
			encoderMapping();
			break;
		case 5://5. Calibrating the extrinsic parameters
			LiDARGNSScalibration();
			LiDARGNSSMapping();
			break;
		case 6: //6. ndt建图
			setStartEnd();
			NDTmapping();
			break;
		case 7://7. 从bag中读何塞rawdata
			readAndSaveHesai("/media/echo/DataDisc/9_rosbag/zed_pandar64_ins/Hesai_back_afternoon_2.bag");
			break;
		case 8://8. 测试新写的函数
			testFunction();
			break;
		case 9:
			rslidarmapping();
			break;
		default :
			cout << "无效输入" << endl;
	}
	return(0);
}
