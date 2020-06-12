//
// Created by echo on 2020/4/11.
//

#include "otherFunctions.h"

int main_function::getParam(int argc, char **argv) {
	std::cout
			<< "设置建图模式: 1.g2o+pcd的传统模式(pcd+g2o路径) 2.point to plane ICP (需要提供pcd路径) 3.bpreal ground mapping (pcd+g2o路径)"
			<< "4. 使用编码器和GPS LiDAR 建图" << "5. LiDAR gps 外参标定" << "6. NDT maping" << std::endl;
	cin >> status;
	std::cout << "status: " << status << std::endl;
	if (status == 1 || status == 3) {
		if (argc != 3 && argc != 5) {
			std::cout
					<< "argument error! argument number has to be 3/5! The first one should be pcd path the second should be g2o path"
					<< std::endl;
			std::cout
					<< "./pcd_reader /media/echo/35E4DE63622AD713/fushikang/loop_pcd_single /media/echo/35E4DE63622AD713/fushikang/lihaile.g2o "
					<< std::endl;
			std::cout
					<< "/media/echo/DataDisc/1_pcd_file/pku_bin /media/echo/DataDisc/2_g2o/pku/4edges_out.g2o 500 12210"
					<< std::endl;
			return (-1);
		}
		filepath = argv[1];
		g2o_path = argv[2];
		if (argc == 5) {
			start_id = atoi(argv[3]);
			end_id = atoi(argv[4]);
		}
		std::cout << "start_id " << start_id << " end_id " << end_id;
	}
	if (status == 2 || status == 6) {
		if (argc != 2) {
			std::cout << "argument error! argument number has to be 2! The first one should be pcd path"
					  << std::endl;
			std::cout << "e.g.: \n ./pcd_reader /media/echo/35E4DE63622AD713/fushikang/loop_pcd_single"
					  << std::endl;
			std::cout << "输入pcd路径: " << std::endl;
			cin >> filepath;
			cout << filepath << endl;
		} else {
			filepath = argv[1];
		}
	}
}

ros::Time main_function::fromPath2Time(std::string s) {
	std::vector<std::string> vStr;
	ros::Time cur_time;
	boost::split( vStr, s, boost::is_any_of( "./" ), boost::token_compress_on );
	std::vector<std::string> buffer;
	for( std::vector<std::string>::iterator it = vStr.begin(); it != vStr.end(); ++ it )
		buffer.push_back(*it);
	cur_time.sec = std::atoi(buffer[buffer.size()-3].c_str());
	int nsec_check = std::atoi(buffer[buffer.size()-2].c_str());
	cur_time.nsec = nsec_check;
	return cur_time;
}
bool main_function::GetIntFileNames(const std::string directory, const std::string suffix){
	file_names_.clear();
	DIR *dp;
	struct dirent *dirp;
	dp = opendir(directory.c_str());
	if (!dp) {
		std::cerr << "cannot open directory:" << directory << std::endl;
		return false;
	}
	std::string file;
	while (dirp = readdir(dp)) {
		file = dirp->d_name;
		if (file.find(".") != std::string::npos) {
			file = directory + "/" + file;
			if (suffix == file.substr(file.size() - suffix.size())) {
				file_names_.push_back(file);
			}
		}
	}
	closedir(dp);
	std::sort(file_names_.begin(), file_names_.end());
	
	if (file_names_.empty()) {
		std::cerr << "directory:" << directory << "is empty" << std::endl;
		return false;
	}
	for (int i = 0; i < file_names_.size(); ++i) {
		file_names_[i]  = directory + "/" + std::to_string(i) +".pcd";
	}
	std::cerr << "路径: " << directory << " 有" << file_names_.size() << "个pcd文件" << std::endl;
	return true;
}
bool main_function::GetFileNames(const std::string directory, const std::string suffix) {
	file_names_.clear();
	DIR *dp;
	struct dirent *dirp;
	dp = opendir(directory.c_str());
	if (!dp) {
		std::cerr << "cannot open directory:" << directory << std::endl;
		return false;
	}
	std::string file;
	while (dirp = readdir(dp)) {
		file = dirp->d_name;
		if (file.find(".") != std::string::npos) {
			file = directory + "/" + file;
			if (suffix == file.substr(file.size() - suffix.size())) {
				file_names_.push_back(file);
			}
		}
	}
	closedir(dp);
	std::sort(file_names_.begin(), file_names_.end());
	
	if (file_names_.empty()) {
		std::cerr << "directory:" << directory << "is empty" << std::endl;
		return false;
	}
	std::cerr << "路径: " << directory << " 有" << file_names_.size() << "个pcd文件" << std::endl;
	return true;
}

bool main_function::GetPNGFileNames(const std::string directory, const std::string suffix) {
	PNG_file_names_.clear();
	DIR *dp;
	struct dirent *dirp;
	dp = opendir(directory.c_str());
	if (!dp) {
		std::cerr << "cannot open directory:" << directory << std::endl;
		return false;
	}
	std::string file;
	while (dirp = readdir(dp)) {
		file = dirp->d_name;
		if (file.find(".") != std::string::npos) {
			file = directory + "/" + file;
			if (suffix == file.substr(file.size() - suffix.size())) {
				PNG_file_names_.push_back(file);
			}
		}
	}
	closedir(dp);
	std::sort(PNG_file_names_.begin(), PNG_file_names_.end());
	
	if (PNG_file_names_.empty()) {
		std::cerr << "directory:" << directory << "is empty" << std::endl;
		return false;
	}
	std::cerr << "路径: " << directory << " 有" << PNG_file_names_.size() << "个PNG文件" << std::endl;
	return true;
}

bool main_function::FindFileseq(int64_t seq)  {
	int64_t idx_file = seq;
	if (idx_file > file_names_.size() - 1) {
		return INT64_MAX;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	uint64_t idx_seperator = file_names_[idx_file].find_last_of("/");
	filename = file_names_[idx_file].substr(idx_seperator + 1);
	std::cout << "PCD file is " << filename << " seq is" << idx_file << std::endl;
	return true;
}

pcl::PointCloud<pcl::PointXYZI>
main_function::continusTimeDistrotion(std::vector<Eigen::Matrix4f> &poses, std::vector<mypcdCloud> &clouds) {
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

int main_function::g2omapping() {
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
//todo地图坐标系下把当前sscan建立成octo
pcl::PointCloud<pcl::PointXYZI> main_function::localMapOct(pcl::PointCloud<pcl::PointXYZI> last_fine,pcl::PointCloud<pcl::PointXYZI> this_coarse){
	//输入: 1.当前tf过的点云,用于分割roi 2. 上次精细配准的点云,用于累加地图 返回localmap
	util tools;
	tools.timeCalcSet("$$ cha tree    ");
	//上次的点云放进来
	pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new	pcl::PointCloud<pcl::PointXYZI>);
	for (int k = 0; k < last_fine.size(); ++k) {
		octree->addPointToCloud(last_fine[k],octpcd);
	}
	tools.timeUsed();
	//DS scan
	pcl::UniformSampling<pcl::PointXYZI> filter;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tfed_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	*tfed_ptr = this_coarse;
	filter.setInputCloud(tfed_ptr);
	filter.setRadiusSearch(1);
	pcl::PointCloud<int> keypointIndices;
	filter.compute(keypointIndices);
	pcl::copyPointCloud(this_coarse, keypointIndices.points, *filteredCloud);
	std::cout<<"$$ filter scan size: "<<filteredCloud->size()<<std::endl;
	std::vector<int> pointIdxall;
	//根据结果获得localmap
	tools.timeCalcSet("$$ local map time a   ");//0.00561342
	pcl::PointCloud<pcl::PointXYZI> local_map;
	for (int l = 0; l < filteredCloud->size(); ++l) {
		std::vector<int> pointIdxVec1;

		octree->voxelSearch (filteredCloud->points[l], pointIdxVec1);
/*		for (int j = 0; j < pointIdxVec1.size(); ++j) {
			pointIdxall.push_back(pointIdxVec1[j]);
		}*/
		for (int j = 0; j < pointIdxVec1.size(); ++j) {
			local_map.push_back(octpcd->points[pointIdxVec1[j]]);
		}
	}
	tools.timeUsed();
/*	tools.timeCalcSet("$$ local map time b   ");//0.0995338
	std::cout<<"vector size: "<<pointIdxall.size()<<std::endl;
	std::sort(pointIdxall.begin(), pointIdxall.end());
	std::vector<int>::iterator new_end = unique(pointIdxall.begin(), pointIdxall.end());//"删除"相邻的重复元素
	pointIdxall.erase(new_end, pointIdxall.end());//删除(真正的删除)重复的元素
	tools.timeUsed();
	tools.timeCalcSet("$$ local map time c  ");//0.0155058
	for (int j = 0; j < pointIdxall.size(); ++j) {
		local_map.push_back(octpcd->points[pointIdxall[j]]);
	}
	tools.timeUsed();*/
	return local_map;
}
pcl::PointCloud<pcl::PointXYZI> main_function::lidarLocalMapDistance(std::vector<Eigen::Matrix4f> &poses,
																	 std::vector<pcl::PointCloud<pcl::PointXYZI>> &clouds,
																	 double distiance, int buffer_size,
																	 bool &local_map_updated,
																	 pcl::PointCloud<pcl::PointXYZI> last_local_map) {
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
 
	//满足运动距离
	if(distance_calc>distiance){
		local_map_updated = true;
		//1. 最新帧降采样
		*map_temp_ptr = clouds.back();
		sor.setInputCloud(map_temp_ptr);
		//sor.setLeafSize(0.05f, 0.05f, 0.05f);//室内
		sor.setLeafSize(0.1f, 0.1f, 0.1f); //外面
		sor.filter(clouds.back());
/*		*map_temp_ptr = clouds.back();
		sor1.setInputCloud(map_temp_ptr);
		sor1.setMeanK(50);
		sor1.setStddevMulThresh(1.0);
		sor1.filter(clouds.back());*/
		
		//2. note 15帧64线 0.02 大概225520个点 //10帧试试
		if (poses.size() >= buffer_size) { //后期点云比较多的情况
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
		//4.整体地图降采样
		sor.setInputCloud(map_ptr);
		sor.setLeafSize(0.25f, 0.25, 0.1f);
		sor.filter(map_temp);
		
		return map_temp;
	}else{
		local_map_updated = false;
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

int main_function::point2planeICP() {
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
	ros::Publisher local_map_pub;
	ros::Publisher path_publish;
	ros::Publisher scan_tfed;
	test_frame = node.advertise<sensor_msgs::PointCloud2>("/local_map", 5);
	continue_frame = node.advertise<sensor_msgs::PointCloud2>("/continue_frame", 5);
	test_frame_linear = node.advertise<sensor_msgs::PointCloud2>("/current_frame_linear", 5);
	local_map_pub = node.advertise<sensor_msgs::PointCloud2>("/local_map_oct", 5);
	path_publish = node.advertise<nav_msgs::Path>("/lidar_path", 5);
	scan_tfed = node.advertise<sensor_msgs::PointCloud2>("/scan_tf", 5);
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
	float last_speed = 0;
	float acc = 0;//加速度
	//continus-time distortion
	std::vector<Eigen::Matrix4f>  poses_distortion;
	std::vector<mypcdCloud> clouds_distortion_origin;
	//kd tree 测试
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (1);
	pcl::PointCloud<pcl::PointXYZI>::Ptr octpcd(new pcl::PointCloud<pcl::PointXYZI>);
	octree.setInputCloud (octpcd);
	pcl::PointCloud<pcl::PointXYZI> oct_last;
	pcl::PointCloud<pcl::PointXYZI> oct_cur;
//class
	util tools,tools2;
	registration icp;
	//位姿存成csv
	CSVio csvio;//位姿csv
	imgAddColor2Lidar a;//投影点云
	a.readExInt("/home/echo/fusion_ws/src/coloured_cloud/ex_params.txt");
	//投影相关
	bool color = false;
	pcl::PointCloud<pcl::PointXYZRGB> tosave;
	cv::Mat mat;VLPPointCloud cloudin;
/*		icp.setParam("/media/echo/DataDisc/3_program/mapping/cfg/icp.yaml");*/
	pcl::PCDWriter writer;
	bool first_cloud = true;
	bool distortion = true;
	std::cout<<file_names_ .size()<<std::endl;
	std::cout<<start_id<<" "<<end_id<<std::endl;
	
	bool VLP = true;
	std::string LiDAR_type = "robo";
	bool local_map_updated = true; //todo 加入地图更新判断 1100-3000
	std::vector<nav_msgs::Odometry> odoms;//当前结果转成odom 存储
	nav_msgs::Odometry current_odom;
	//oct
	localMapOctInit(1);
	//开始迭代
	if(color){
		if(file_names_ .size() != PNG_file_names_ .size()){
			std::cout<<"wrong size, pcd: "<<file_names_.size()<<" PNG "<<PNG_file_names_ .size()<<std::endl;
			return(0);
		}
	}

	for(int i = 0;  i <file_names_ .size();i++){
		tools2.timeCalcSet("total");
		if (i>=start_id && i<=end_id) {
			//存储时间戳
			ros::Time cur_time;
			cur_time = fromPath2Time(file_names_[i]);
			//0. 读取不同pcd类型
			//存储完成
			if(LiDAR_type == "VLP"){//判断用的是不是vlp的,用的话进行转换
				pcl::io::loadPCDFile<VLPPoint>(file_names_[i], xyzirVLP);
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
			if(first_cloud){                  //1.1 第一帧 不进行计算
				pcl::copyPointCloud(*cloud_hesai,*cloud_local_map);
				poses.push_back(Eigen::Matrix4f::Identity());
				clouds.push_back(*cloud_local_map);
				first_cloud = false;
				g2osaver.insertPose(Eigen::Isometry3d::Identity());
				oct_last = *cloud_local_map;
			} else{
				//todo 有个bug,就是第二次去畸变没有用完全的tf去畸变 没问题
				//2.1 加个去畸变
				simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef); //T_l-1_l
				//2.1.1 设为普通icp********
				icp.transformation = Eigen::Matrix4f::Identity();
				//2.2 输入的也应该降采样 todo 输入降采样
				//(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
				
				//2.3 ICP
				tools.timeCalcSet("第一次ICP用时     ");
				icp.SetNormalICP(); //设定odom icp参数0.5+acc*0.01
				tfed = icp.normalIcpRegistration(cloud_bef,*cloud_local_map);
				icp_result.push_back(icp.increase);//T_l_l+1
				//2.3.1 再次去畸变 再次减小范围
				simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
				//pointCloudRangeFilter(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
				tools.timeUsed();
				
				tools.timeCalcSet("局部地图用时    ");
				//2.3.2.1 局部地图生成
				//*cloud_local_map = lidarLocalMap(poses,clouds,50);  //生成局部地图****
				//todo 局部上色
				*cloud_local_map = lidarLocalMapDistance(poses,clouds,0.6,100 ,local_map_updated,*cloud_local_map);  //生成局部地图****
				/*//恢复位姿 todo voxel based local map
				Eigen::Matrix4f current_posea = Eigen::Matrix4f::Identity();
				for (int k = 0; k < icp_result.size(); ++k) {
					current_posea *= icp_result[k];
				}
				pcl::transformPointCloud(*cloud_bef,oct_cur,current_posea);
				*//**cloud_local_map  = *//*localMapOct(oct_last,oct_cur);
				localMapOct(oct_last,oct_cur);
				std::cout<<oct_last.size()<<" "<<oct_cur.size()<<std::endl;*/
				tools.timeUsed();
				
				//2.3.2 ******再次icp           *********** &&&&这个当做 lidar mapping
				tools.timeCalcSet("第二次ICP用时    ");
				icp.SetPlaneICP();	//设定点面 mapping icp参数0.3+acc*0.01
				tfed = icp.normalIcpRegistrationlocal(cloud_bef,*cloud_local_map);
				icp_result.back() = icp_result.back()*icp.pcl_plane_plane_icp->getFinalTransformation(); //第一次结果*下一次去畸变icp结果
				//2.3.3 再次去畸变 再次减小范围
				simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
				//pointCloudRangeFilter(*cloud_bef,75 - 1.5*(curr_speed/10)); //根据车速减小范围
				tools.timeUsed();
				
				//2.4 speed
				curr_speed = sqrt(icp_result.back()(0,3)*icp_result.back()(0,3)+icp_result.back()(1,3)*icp_result.back()(1,3))/0.1;
				// 2.5 点云投影
				if(i-1>=0 && color){
					mat = cv::imread(PNG_file_names_[i-1]);
					tosave  = a.pclalignImg2LiDAR(mat,*cloud_bef);
				}
				//2.6 存储结果
				*local_map_to_pub = *cloud_local_map;
				*cloud_local_map = *cloud_bef; 	//下一帧匹配的target是上帧去畸变之后的结果
				//可以用恢复出来的位姿 tf 以前的点云
				clouds.push_back(*cloud_bef);
				std::stringstream pcd_save;
		/*		pcd_save<<"dist_pcd/"<<i<<".pcd";
				writer.write(pcd_save.str(),*cloud_bef, true);*/
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
				std::cout<<"*****上次点云ID: "<<i<<" ***** speed: "<<3.6*curr_speed<<" km/h"<<" acc: "<<acc<<" m/s^2\n"<<std::endl;
				//存一下'csvio
				Eigen::Isometry3d se3_save;
				csvio.LiDARsaveOnePose(Eigen::Isometry3d(current_pose.matrix().cast<double>()),cur_time);//转csv用的
				
				//保存速度加速度
				acc = (curr_speed- last_speed)/0.1;// a = dv/dt
				last_speed = curr_speed;
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
					oct_last = tfed;
					pcd_save<<"tf_ed/"<<i<<".pcd";
					writer.write(pcd_save.str(),tfed, true);
					//存的点云缩小点,每50帧存一下结果;
		/*			if(i%100==50){
						pcl::PointCloud<int> keypointIndices;
						filter_us.setInputCloud(cloud_map);
						filter_us.setRadiusSearch(0.02f);
						filter_us.compute(keypointIndices);
						pcl::copyPointCloud(*cloud_map, keypointIndices.points, cloud_map_ds);
						*cloud_map = cloud_map_ds;
						writer.write("cloud_map.pcd",*cloud_map, true);
						if(color){
							writer.write("cloud_map_color.pcd",*cloud_map_color, true);
						}
					}*/
					
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
					
					pcl::toPCLPointCloud2(icp.local_map_with_normal, pcl_frame);
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
				//2.7 kd tree 测试
				if(i>=3460){
					//输入: 1.当前tf过的点云,用于分割roi 2. 上次精细配准的点云,用于累加地图 返回localmap
	/*				pcl::PointCloud<pcl::PointXYZI> local_map;
					pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_ptr(new pcl::PointCloud<pcl::PointXYZI> );
					local_map  = localMapOct(tfed,tfed);
					pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals  (new pcl::PointCloud<pcl::PointXYZINormal>);
					*local_map_ptr = local_map;
					tools.timeCalcSet("局部地图 normal raidius 0.15:     ");
					icp.addNormalRadius(local_map_ptr,cloud_with_normals);
					tools.timeUsed();
					std::cout<<"local_map size: "<<local_map.size()<<std::endl;
					pcl::toPCLPointCloud2(*cloud_with_normals, pcl_frame);
					pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
					to_pub_frame_linear.header.frame_id = "/map";
					local_map_pub.publish(to_pub_frame_linear);
					
					pcl::toPCLPointCloud2(tfed, pcl_frame);
					pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
					to_pub_frame_linear.header.frame_id = "/map";
					scan_tfed.publish(to_pub_frame_linear);*/
				}
			}
		}
	}
	std::cout<<save_g2o_path<<"\n"<<save_pcd_path<<std::endl;
	g2osaver.saveGraph(save_g2o_path);
	writer.write(save_pcd_path,*cloud_map, true);
	if(color){
		writer.write(save_color_pcd_path,*cloud_map_color, true);
	}
/*	writer.write("distro_final.pcd",*cloud_map_continus_time, true);*/
	csvio.LiDARsaveAll("useless");
	return(0);
}

void main_function::traversableMapping(){
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

void main_function::encoderMapping() {
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

void main_function::NDTmapping() {
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
//todo GPS_loop_mapping中的find函数找到最近的
Eigen::Isometry3d  main_function::LiDARGNSScalibration(std::string lidar_g2o, std::string gps_pcd) {
	pcl::PCDWriter writer;
//	ReadBag rb;
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
	
	for (int i = 0; i < gps_position->size(); ++i) {
		pcl::PointXYZ temp;
		temp.x = trans_vector[i](0,3);
		temp.y = trans_vector[i](1,3);
		temp.z = trans_vector[i](2,3);
 
		lidar_position.push_back(temp);
		gps_position_save.push_back(gps_position->points[i]);
	}
 
	//**这一部分产生一一对应的关系
	//todo 这里gps的位置比较多, 所以以 惯导的数量为基础 惯导 50hz lidar 10 hz
	//改成lidar10hz gps 1hz
	for (int j = 0; j < 8500 ; ++j) {
		Eigen::Matrix4d temp;
		temp.setIdentity();
		temp(0,3) = gps_position->points[j].x;
		temp(1,3) = gps_position->points[j].y;
		temp(2,3) = gps_position->points[j].z;
		gps_poses.push_back(temp);
	}
	for (int k = 0; k < 8500; ++k) {
		LiDAR_poses.push_back(trans_vector[k+1].matrix());
	}
	Eigen::Vector3d arm;
	arm(2) = 0.3;//杆臂值
	calibrate.CalibrateGNSSLiDARICP(gps_poses,LiDAR_poses,T_lidar2INS, arm);//用icp算法确定外参
	pcl::transformPointCloud(lidar_position,lidar_position_tfed,T_lidar2INS.inverse().matrix());
	writer.write("route/lidar_position.pcd",lidar_position_tfed);
	writer.write("route/raw_lidar_position.pcd",lidar_position);
	writer.write("route/gps_position_save.pcd",gps_position_save);
	std::cout<<"标定结果:\n"<<T_lidar2INS.inverse().matrix()<<std::endl;
	return  T_lidar2INS;
}

void main_function::transformOnePoint(Eigen::Matrix4f t, pcl::PointXYZI &input) {
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

void
main_function::simpleDistortion(VLPPointCloud input, Eigen::Matrix4d increase, pcl::PointCloud<pcl::PointXYZI> &output) {
	output.clear();
 
	pcl::PointCloud<PointTypeBeam>::Ptr test(new pcl::PointCloud<PointTypeBeam>);
	pcl::PointCloud<PointTypeBeam>::Ptr pcout(new pcl::PointCloud<PointTypeBeam>);
	
	featureExtraction Feature;
	Eigen::Isometry3d se3;
	se3 = increase ;
	PointTypeBeam temp;
	for (int i = 0; i < input.size(); ++i) {
		temp.x = input[i].x;
		temp.y = input[i].y;
		temp.z = input[i].z;
		temp.intensity = input[i].intensity;
		temp.pctime = (input[i].time - input[input.size()-1].time) / (input[input.size()-1].time-input[0].time); // t/|t|
		temp.beam = input[i].ring;
		test->push_back(temp);
	}
	Feature.adjustDistortion(test, pcout, se3);
	pcl::copyPointCloud(*pcout, output);
}
void
main_function::simpleDistortion(mypcdCloud input, Eigen::Matrix4f increase, pcl::PointCloud<pcl::PointXYZI> &output) {
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
		temp.pctime = (input[i].timestamp - input[input.size()-1].timestamp) * 10;//					算出来的t是最后一个点的位姿
		//temp.pctime = (input[i].timestamp - input[0].timestamp) * 10;  //这个正确的但是,第以一圈开始为起点 算出来的t是第一个点的位姿
		temp.beam = input[i].ring;
		test->push_back(temp);
	}
	Feature.adjustDistortion(test, pcout, se3);
	pcl::copyPointCloud(*pcout, output);
}

std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
main_function::getEigenPoseFromg2oFile(std::string g2ofilename) {
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

pcl::PointCloud<pcl::PointXYZI>::Ptr
main_function::align(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration,
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

void main_function::pointCloudRangeFilter(pcl::PointCloud<pcl::PointXYZI> &input, float range) {
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
void main_function::testspline(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector) {
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


void
main_function::genfeaturemap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
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


pcl::PointCloud<pcl::PointXYZI>
main_function::lidarLocalMap(std::vector<Eigen::Matrix4f> &poses, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clouds,
							 int buffer_size){
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



void main_function::saveFile(std::string outFilename, std::vector<VertexSE3 *> vertices, std::vector<EdgeSE3 *> edges) {
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

void main_function::setStartEnd() {
	std::cout << "设置起始pcd" << std::endl;
	cin >> start_id;
	std::cout << "start: " << start_id << std::endl;
	std::cout << "设置结束pcd" << std::endl;
	cin >> end_id;
	std::cout << "end: " << end_id << std::endl;
}

int main_function::g2oColorMapping() {
	//得到所有的位姿向量
	trans_vector = getEigenPoseFromg2oFile(g2o_path);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
	cout << "trans_vector.size() : " << trans_vector.size() << " file_names_.size() : " << file_names_.size()<< endl;
	if (trans_vector.size() == file_names_.size()) {
		genlocalmap(trans_vector, filepath, *cloud1);
	} else {
		cout << "!!!!! PCD & g2o does not have same number " << endl;
		return 0;
	}
}
//构建gps的因子图
void main_function::gpsBasedOptimziation(std::string lidar_path,std::string gps_path,Eigen::Isometry3d lidar_to_gps,std::string save_path) {

}
//todo 完成这个
void main_function::IMUMapping(){
	//1.获得 IMUdata
	start_id = 0;
	IMUPreintergration();
	//2. LiDAR mapping
	GetIntFileNames("/media/echo/DataDisc2/shandong/pcd_inout_abstime","pcd");
	IMUBasedpoint2planeICP();
}
void main_function::IMUPreintergration() {
	
	Eigen::Quaterniond q;
	Eigen::Quaterniond imu_angle_speed;
	ros::Publisher imu_pub;
	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");
	imu_pub = node.advertise<sensor_msgs::Imu>("/imu_intergrate", 5);
	sensor_msgs::Imu topub;
	topub.header.frame_id = "map";
	q.setIdentity();
	CSVio csvio;
	csvio.ReadImuCSV("/home/echo/shandong_in__out/imu/imu.csv",	IMUdata );
//	ros::Rate r(125);
/*	for (int i = 0; i < IMUdata.size(); ++i) {
	//1.q
		topub.linear_acceleration.x = IMUdata[i][0];
		topub.linear_acceleration.y = IMUdata[i][1];
		topub.linear_acceleration.z = IMUdata[i][2];
		topub.angular_velocity.x = IMUdata[i][3];
		topub.angular_velocity.y = IMUdata[i][4];
		topub.angular_velocity.z = IMUdata[i][5];
		imu_angle_speed.w() = 1;
		imu_angle_speed.x() = IMUdata[i][3]*0.5*0.008;
		imu_angle_speed.y() = IMUdata[i][4]*0.5*0.008;
		imu_angle_speed.z() = IMUdata[i][5]*0.5*0.008;
		q = q*imu_angle_speed;
		topub.orientation.x = q.x();
		topub.orientation.y = q.y();
		topub.orientation.z = q.z();
		topub.orientation.w = q.w();
		imu_pub.publish(topub);
		//2.v
		Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);
 		std::cout<<"roll: "<<eulerAngle(0)<<" pitch: "<<eulerAngle(1)<<" yaw: "<<eulerAngle(2)<<" time: "<<IMUdata[i][6]<<std::endl;
		r.sleep();
	}*/
}



void main_function::genlocalmap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
						   std::string filepath, pcl::PointCloud<pcl::PointXYZI> &bigmap) {
	Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
	//1.1提取地面,准备产生地面的地图
	PlaneGroundFilter filter;
	featureExtraction Feature;
 
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
	pcl::PointCloud<PointTypeSm>::Ptr segmentedCloud_tf(new pcl::PointCloud<PointTypeSm>);
	pcl::PointCloud<PointTypeSm>::Ptr segmentedCloud_map(new pcl::PointCloud<PointTypeSm>);
	pcl::PointCloud<pcl::PointXYZI> cloud_save;
	
	cout << "start interation" << endl;
 
	pcl::PCDWriter writer;
	float min_intensity;
	float max_intensity;
	util tools;
	//1.遍历所有点
	end_id = file_names_.size();
	std::cout<<"end ID: "<<end_id<<std::endl;
	for (int i = 1; i < file_names_.size(); i++) {
 
		if (i > start_id && i < end_id) {
			//1.1设置起始和结束的点
			std::vector<int> indices1;
			//读点云
			pcl::io::loadPCDFile<pcl::PointXYZI>(file_names_[i], *cloud_bef);
			pcl::removeNaNFromPointCloud(*cloud_bef, *cloud_rot, indices1);
			
			*cloud_bef = *cloud_rot;
			//调整pandar的反射率
/*			min_intensity = cloud_bef->points[0].intensity;
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
			}*/
			//结束调整反射率
			//开始采集感兴趣的区域
/*			if (tensorvoting) {
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
			*cloud_bef = *cloud_rot;*/
			//计算两帧的增量
			out3d = trans_vector[i].matrix();
			out3d = trans_vector[i - 1].inverse().matrix() * out3d.matrix();
			
	 
			//去畸变
			Feature.checkorder(cloud_bef, test);
			Feature.adjustDistortion(test, pcout, out3d);
 			//loam 特征提取 可以注释掉
	/*		Feature.calculateSmoothness(pcout, segmentedCloud);
			Feature.calcFeature(segmentedCloud);*/
 
			//tmp用来转换格式 把去了畸变的放进去
			tmp->clear();
			tmp->resize(pcout->size());
			for (int j = 0; j < pcout->size(); ++j) {
				tmp->points[j].x = pcout->points[j].x;
				tmp->points[j].y = pcout->points[j].y;
				tmp->points[j].z = pcout->points[j].z;
				tmp->points[j].intensity = pcout->points[j].intensity;
			}
			//todo 改成 第一步提取index 先,之后记录index,最后滤除
			//***这里是地面滤波器
/*			filter.point_cb(*tmp);
			*tmp = *filter.g_ground_pc;*/
			pcl::VoxelGrid<pcl::PointXYZI> sor;
			sor.setInputCloud(tmp);                   //设置需要过滤的点云给滤波对象
			sor.setLeafSize(0.2, 0.2, 0.2);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
			sor.filter(*cloud_aft);
			//new
			pcl::transformPointCloud(*cloud_aft, *tmp, trans_vector[i].matrix());
			//loam feature的
			/*pcl::transformPointCloud(*segmentedCloud, *segmentedCloud_tf, trans_vector[i].matrix());
			for (int k = 0; k < segmentedCloud_tf->size(); ++k) {
				pcl::PointXYZI p;
				p.x = segmentedCloud_tf->points[k].x;
				p.y = segmentedCloud_tf->points[k].y;
				p.z = segmentedCloud_tf->points[k].z;
				p.intensity = segmentedCloud_tf->points[k].smooth;
				cloud_save.push_back(p);
			}*/
			*cloud_add += *tmp;
	 
/*			if (!tensorvoting) {
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
			}*/
			int percent = 0;
			int size_all = 0;
			size_all = end_id - start_id;
			percent = i * 100 / size_all;
			std::cout <<i<<" "<< percent << "%" << std::endl;
			
		}
	}
	cout << "end interation" << endl;
 
	//转换回(0,0,0,0,0,0)
	//pcl::transformPointCloud(*cloud_add, *cloud_aft, trans_vector[0].inverse().matrix());
	// 全点云时候应当加
/*	if (!tensorvoting) {
		pcl::VoxelGrid<pcl::PointXYZI> sor;
		sor.setInputCloud(cloud_aft);
		sor.setLeafSize(0.2, 0.2, 0.2);
		sor.filter(*cloud_add);
	}*/
	// pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// outrem.setInputCloud(cloud_add);
	// outrem.setRadiusSearch(0.5);
	// outrem.setMinNeighborsInRadius (5);
	// // 应用滤波器
	// outrem.filter (*cloud_aft);
	//这里就是整
/*	bigmap.clear();
	for (int l = 0; l < cloud_add->size(); ++l) {
		if (cloud_add->points[l].intensity > (float) 3.5 && cloud_add->points[l].intensity < (float) 30) {
			if(cloud_add->points[l].intensity ==4){
				cloud_add->points[l].intensity =0;
			}
			bigmap.push_back(cloud_add->points[l]);
		}
	}
	*cloud_add = bigmap;*/
	cout << "map saving" << endl;
	writer.write<pcl::PointXYZI>("final_map.pcd", *cloud_add, true);
	//writer.write<pcl::PointXYZI>("loam_feature.pcd", cloud_save, true);
	
}

void main_function::genColormap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,std::string picParam) {
	Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_add(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ> localmap;
	VLPPointCloud xyzirVLP;
	mypcdCloud xyzItimeRing;
	pcl::PointCloud<pcl::PointXYZRGB> tfed_color;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_map_color(new 	pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<PointXYZRGBI> _tfed_color_i;
	pcl::PointCloud<PointXYZRGBI>::Ptr cloud_i_map_color(new 	pcl::PointCloud<PointXYZRGBI>);
	
	Eigen::Isometry3d out3d = Eigen::Isometry3d::Identity();
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZRGB> tosave;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tosave_ptr(new 	pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<PointXYZRGBI> tosave_i;
	cv::Mat mat;
	PlaneGroundFilter filter;
	imgAddColor2Lidar a;//投影点云
	a.readExInt("/home/echo/shandong_in__out/exparam.txt");
	//1.遍历所有点
	end_id = file_names_.size();
	start_id = 1;
	for (int i = 1; i < file_names_.size()-3; i++) {
		int percent = 0;
		int size_all = 0;
		size_all = static_cast<int>(file_names_.size());
		percent = i * 100 / size_all;
	
		
		if (i >= start_id && i < end_id) {
			//1.1设置起始和结束的点
	/*		std::vector<int> indices1;
			//读点云
			pcl::io::loadPCDFile<VLPPoint>(file_names_[i], xyzirVLP);
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
			//计算两帧的增量
			out3d = trans_vector[i].matrix();
			out3d = trans_vector[i - 1].inverse().matrix() * out3d.matrix();
			//去畸变
			Eigen::Matrix4f out3f;
			out3f = out3d.matrix().cast<float>();
			simpleDistortion(xyzItimeRing,out3f.inverse(),*cloud_bef);*/
			loopClosure lc;
			lc.genVLPlocalmap(0,trans_vector,i,filepath,*cloud_bef);
			//地面点
/*			filter.point_cb(*cloud_bef);
			*cloud_bef = *filter.g_ground_pc;*/
			if(i-1>=0){
				mat = cv::imread(PNG_file_names_[i-1]);
				tosave  = a.pclalignImg2LiDAR(mat,*cloud_bef);
				//tosave_i = a.alignImg2LiDAR(mat,*cloud_bef);//_PointXYZRGBL 带intensity
/*				cv::Mat mat_save;
				mat_save = a.pcd2img(mat,*cloud_bef);
				std::stringstream ss;
				ss<<"/home/echo/img_intensity/"<<i<<".png";
				cv::imwrite(ss.str(),mat_save);*/
			}
/*			*tosave_ptr = tosave;
			pcl::VoxelGrid<pcl::PointXYZRGB> sor;
			sor.setInputCloud(tosave_ptr);                   //设置需要过滤的点云给滤波对象
			sor.setLeafSize(0.2, 0.2, 0.2);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
			sor.filter(tosave);*/
			pcl::transformPointCloud(tosave,tfed_color,trans_vector[i].matrix());
			*cloud_map_color += tfed_color;
			//pcl::transformPointCloud(tosave_i,_tfed_color_i,trans_vector[i].matrix());
			//*cloud_i_map_color += _tfed_color_i;
			std::cout << percent << "%" <<std::endl;
		}
	}
	pcl::io::savePCDFileASCII("cloud_map_color.pcd",*cloud_map_color);
	//writer.write("cloud_map_color.pcd",*cloud_map_color, true);
	//pcl::io::savePCDFileASCII("s.pcd",*cloud_i_map_color);
	//writer.write("cloud_map_color_i.pcd",*cloud_i_map_color, true);
}
//带imu的 icp
int main_function::IMUBasedpoint2planeICP() {
	//0. 测试 不用icp 直接读结果
	trans_vector = getEigenPoseFromg2oFile("/home/echo/shandong_in__out/LiDAR_Odom.g2o");
	//1. local map index位为时间 1e-7 s
 
	bool first_cloud = true;
 	//2. 读取到的点云
	VLPPointCloud xyzirVLP;
	//3. 重力matrix 取第一秒
	Eigen::Isometry3d T_w_L0;
	T_w_L0 = GetRollPitch(1200);//6.9 19
 	std::cout<<T_w_L0.matrix()<<std::endl;
	//4.第一个点的时间
	float first_point_time = 0;
	//测试imu积分
	Eigen::Vector3d Vw;
	Vw.setZero();
 
	//5. 点云匹配部分
	std::vector<Eigen::Matrix4d> poses;
	std::vector<pcl::PointCloud<pcl::PointXYZI>>  clouds;
	//6. 位姿结果
	Eigen::Isometry3d last_scan_begin,last_scan_end;
	
	//7.去畸变的点云
	pcl::PointCloud<pcl::PointXYZI> imu_distorted_pointcloud;
	start_id = 0;
	end_id = 600;
	std::vector<double> x,xl , v_lx,v_ly,v_lz,p_lx,p_ly,p_lz,v_x,v_y,v_z,r_x,r_y,r_z,p_x,p_y,p_z,r_lz,r_ly,r_lx;
	pcl::io::loadPCDFile<VLPPoint>(file_names_[0], xyzirVLP);
	int start_index = IMUCorreIndex(xyzirVLP[xyzirVLP.size()-1].time);
	
	for(int i = 1;  i <file_names_ .size();i++){
		if (i>=start_id && i<=end_id) {//0.0控制开始结束
			//0.1 读取vlp pcd
			pcl::io::loadPCDFile<VLPPoint>(file_names_[i], xyzirVLP);
			//1. 这里用pcl的 plane to plane icp
			if(first_cloud){
				//1.1.1 第一帧 不进行计算 scan_begin
				last_scan_begin = trans_vector[i-1]*T_w_L0;
				first_cloud = false;
//				poses.push_back(Eigen::Matrix4d::Identity());
			} else{
		
				//1.2.1 当前加载帧  起始index
				last_scan_end = trans_vector[i]*T_w_L0;
				//1.2.2 获得平均速度 上帧的
				std::cout<<i<<std::endl;
				Eigen::Vector3d this_begin_speed = GetVelocityOfbody(last_scan_begin,last_scan_end,IMUCorreIndex(xyzirVLP[0].time));
				imu_distorted_pointcloud = IMUDistortion(xyzirVLP,Eigen::Isometry3d::Identity(),this_begin_speed,IMUCorreIndex(xyzirVLP[0].time));
				//1.2.2 存下结果
				pcl::PCDWriter writer;
				std::stringstream pcd_save;
				pcd_save<<"/home/echo/6_test_pcd/imu_distort/"<<i<<".pcd";
				writer.write(pcd_save.str(),imu_distorted_pointcloud, false);
				//pcl::transformPointCloud(*cloud_aft, *tmp, trans_vector[i].matrix());
				//结束时候index
				last_scan_begin = last_scan_end;
				//matplotlib
				xl.push_back((i-1.0)/10.0);
				v_lx.push_back(this_begin_speed(0));
				v_ly.push_back(this_begin_speed(1));
				v_lz.push_back(this_begin_speed(2));
				p_lx.push_back(last_scan_end.translation()(0));
				p_ly.push_back(last_scan_end.translation()(1));
				p_lz.push_back(last_scan_end.translation()(2));
				Eigen::Quaterniond eulerAngle(last_scan_end.rotation());
				r_lx.push_back(eulerAngle.x());
				r_ly.push_back(eulerAngle.y());
				r_lz.push_back(eulerAngle.z());
			}
		}
	}
	std::cout<<bias_gyro<<std::endl;
	std::cout<<"start: "<<start_index<<std::endl;
	for (int k = start_index; k < IMUdata.size(); ++k) {
		if(k<7500){
			IMUIntergrate(T_w_L0,Vw,k);
			x.push_back(k/125.0);
			v_x.push_back(Vw(0));
			v_y.push_back(Vw(1));
			v_z.push_back(Vw(2));
			p_x.push_back(T_w_L0.translation()(0));
			p_y.push_back(T_w_L0.translation()(1));
			p_z.push_back(T_w_L0.translation()(2));
			Eigen::Quaterniond eulerAngle(T_w_L0.rotation());
			r_x.push_back(eulerAngle.x());
			r_y.push_back(eulerAngle.y());
			r_z.push_back(eulerAngle.z());
		}
	}
	
	plt::figure_size(10000, 4000);
	plt::title("IMU lidar q Estimate");
	plt::plot(x, r_x);
	plt::plot(x, r_y);
	plt::plot(x, r_z);
	plt::plot(xl, r_lx,".");
	plt::plot(xl, r_ly,".");
	plt::plot(xl, r_lz,".");
	plt::named_plot("r_x", x, r_x);
	plt::named_plot("r_y", x, r_y);
	plt::named_plot("r_z", x, r_z);
	plt::named_plot("r_lx", xl, r_lx);
	plt::named_plot("r_ly", xl, r_ly);
	plt::named_plot("r_lz", xl, r_lz);
	plt::legend();
	plt::save("./q.png");
	plt::close();
	
	plt::figure_size(10000, 4000);
	plt::title("IMU Lidar v Estimate");
	plt::plot(x, v_x);
	plt::plot(x, v_y);
	plt::plot(x, v_z);
	plt::plot(xl, v_lx,".");
	plt::plot(xl, v_ly,".");
	plt::plot(xl, v_lz,".");
	plt::named_plot("v_x", x, v_x);
	plt::named_plot("v_y", x, v_y);
	plt::named_plot("v_z", x, v_z);
	plt::named_plot("v_lx", xl, v_lx);
	plt::named_plot("v_ly", xl, v_ly);
	plt::named_plot("v_lz", xl, v_lz);
	plt::legend();
	plt::save("./v_imu_lidar.png");
	plt::close();
	
	plt::figure_size(10000, 4000);
	plt::title("IMU Lidar p Estimate");
	plt::plot(x, p_x);
	plt::plot(x, p_y);
	plt::plot(x, p_z);
	plt::plot(xl, p_lx,".");
	plt::plot(xl, p_ly,".");
	plt::plot(xl, p_lz,".");
	plt::named_plot("p_x", x, p_x);
	plt::named_plot("p_y", x, p_y);
	plt::named_plot("p_z", x, p_z);
	plt::named_plot("p_lx", xl, p_lx);
	plt::named_plot("p_ly", xl, p_ly);
	plt::named_plot("p_lz", xl, p_lz);
	plt::legend();
	plt::save("./p_imu_lidar.png");
	plt::close();
/*	g2osaver.saveGraph(save_g2o_path);
	csvio.LiDARsaveAll("useless");*/
	return(0);
}
int main_function::IMUCorreIndex(double time){
	double time_diff = 0;
	int index_tmp = -1;
	bool find = false;
	int last = last_imu_index;
	double curr_time = 0;
	//roi先找
	for (int i = last; i < last + 100 && i < IMUdata.size(); ++i) {
		curr_time = IMUdata[i][6];
		time_diff = fabs(curr_time - time);
		if(time_diff <= 0.004){
			last_imu_index = i;
			find = true;
			return  last_imu_index;
		}
	}
	//找所有的
	if(!find){
		for (int i = 0;  i < IMUdata.size(); ++i) {
			curr_time = IMUdata[i][6];
			time_diff = fabs(curr_time - time);
			if(time_diff <= 0.004){
				last_imu_index = i;
				find = true;
				std::cerr<<"refind!!!!"<<std::endl;
				return  last_imu_index;
			}
		}
	}
	//找所有的
	if(!find){
		for (int i = 0;  i < IMUdata.size(); ++i) {
			curr_time = IMUdata[i][6];
			time_diff = fabs(curr_time - time);
			if(time_diff <= 0.008){
				last_imu_index = i;
				find = true;
				std::cerr<<"refind 0.008!!!! "<<time_diff<<std::endl;
				return  last_imu_index;
			}
		}
	}
	//实在没有
	if(!find) {
		std::cerr << "can not find!!!!" << std::endl;
		last_imu_index = index_tmp;
	}
	return  last_imu_index;
}
//imu 估计未来一帧的PQV 没有考虑外参
//1. 重力怎么处理?
void main_function::IMUPQVEstimation(int index, Eigen::Isometry3d current_pose, Eigen::Isometry3d LastPose,
									 std::vector<Eigen::Isometry3d> &pq, std::vector<Eigen::Vector3d> &v,double first_scan_time) {
	double imu_time = IMUdata[index][6];
	std::cout<<" >>>>>>>> "<<index<<" imu time diff "<<imu_time-first_scan_time<<std::endl;
	Eigen::Quaterniond Qwb,imu_angle_speed;//odom0坐标系下的Q
	Eigen::Vector3d Pwb,Vwb,Vwb_middle;//odom坐标系下的p
	Eigen::Isometry3d deltaT;
	deltaT = current_pose*LastPose.inverse() ;
	Qwb = current_pose.rotation();		//第一帧的四元数 = 上次mapping的结果 实际
	Pwb = current_pose.translation();	//第一帧的位置   = 上次mapping的结果
	Vwb_middle = deltaT.translation()*10;//上帧中间的速度
	std::cout<<"\n V: "<<Vwb_middle<<std::endl;
//	Vwb = ; //速度先整成上次的中间速度再积分两次
	Eigen::Vector3d dtheta_half ;
	imu_angle_speed.w() = 1;
	imu_angle_speed.x() = IMUdata[index][3]*0.5*0.008;
	imu_angle_speed.y() = IMUdata[index][4]*0.5*0.008;
	imu_angle_speed.z() = IMUdata[index][5]*0.5*0.008;
	Qwb = Qwb*imu_angle_speed;
	//
/*	Eigen::Quaterniond dq;
	Eigen::Vector3d dtheta_half = imupose.gyro * dt / 2.0;
	dq.w() = 1;
	dq.x() = dtheta_half.x();
	dq.y() = dtheta_half.y();
	dq.z() = dtheta_half.z();
	Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;
	
	Qwb = Qwb * dq;
	Qwb.normalize(); //归一化
	Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
	Vw = Vw + acc_w * dt;*/
}
Eigen::Isometry3d main_function::GetRollPitch(int length){
	Eigen::Vector3d g_c,gyro,bg;
	gravity.setZero();
	for (int l = 0; l < length; ++l) {
		g_c(0) = IMUdata[l][0];
		g_c(1) = IMUdata[l][1];
		g_c(2) = IMUdata[l][2];
		gyro(0) = IMUdata[l][3];
		gyro(1) = IMUdata[l][4];
		gyro(2) = IMUdata[l][5];
		gravity += g_c;
		bias_gyro += gyro;
	}
	std::cout <<" bias_gyro: "<<bias_gyro<<std::endl;
	g_c = gravity;
	gravity = g_c/length;
	bg = bias_gyro;
	bias_gyro = bg/length;
	//设置W系下面的重力
	gravity_w[0] = 0;
	gravity_w[1] = 0;
	gravity_w[2] = -gravity.norm();
	std::cout<<" g: "<<gravity.norm()<<" x,y,z: "<<gravity.transpose()<<" bias_gyro: "<<bias_gyro<<std::endl;
	double roll = atan2(gravity[1],gravity[2]);
	double pitch = atan2(-gravity[0],sqrt(gravity[2]*gravity[2]+gravity[1]*gravity[1]));
	std::cout<<" roll: "<<roll*180/M_PI<<" pitch: "<<pitch*180/M_PI<<std::endl;
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
	Eigen::Quaterniond q =   pitchAngle * rollAngle;
//	Eigen::Quaterniond q =   pitchAngle * rollAngle;
/*	0.999994 0.000161729  0.00353713 nxp公式
	1.19871e-05    0.998796   -0.049057
   -0.00354081   0.0490567     0.99879*/
/*	0.999994 0.000161729  0.00353713 这个
	1.19871e-05    0.998796   -0.049057
 	-0.00354081   0.0490567     0.99879*/
	Eigen::Isometry3d initRotate =Eigen::Isometry3d::Identity();
	initRotate.rotate(q);//重力初值
	return initRotate;
}
void main_function::IMUIntergrate(Eigen::Isometry3d& PQ,Eigen::Vector3d& V,int ImuIndex){
	Eigen::Quaterniond Qwb;
	Qwb = PQ.rotation();
	Eigen::Vector3d gyro,acc,Pwb,Vw;
	Vw = V;
	Pwb = PQ.translation();
	gyro[0] = IMUdata[ImuIndex][3];
	gyro[1] = IMUdata[ImuIndex][4];
	gyro[2] = IMUdata[ImuIndex][5];
	acc[0]  = IMUdata[ImuIndex][0];
	acc[1]  = IMUdata[ImuIndex][1];
	acc[2]  = IMUdata[ImuIndex][2];
	gyro = gyro-bias_gyro;
	float dt = 0.008;
	
	Eigen::Quaterniond dq;
	Eigen::Vector3d dtheta_half = gyro * dt / 2.0;
	dq.w() = 1;
	dq.x() = dtheta_half.x();
	dq.y() = dtheta_half.y();
	dq.z() = dtheta_half.z();
	
	Eigen::Vector3d acc_w = Qwb * (acc) + gravity_w;
	
	Qwb = Qwb * dq;
	Qwb.normalize(); //归一化
	Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
	Vw = Vw + acc_w * dt;
	PQ.setIdentity();
	PQ.rotate(Qwb);
	PQ.pretranslate(Pwb);
	V = Vw;
	//std::cout<<"current P \n"<<Pwb<<" \n current Q: \n "<<Qwb.matrix()<<" \n current V: \n"<<Vw<<std::endl;
}
//22 imu去畸变 需要当前开始点的 P Q V, 开始点的imu index 返回去畸变的点云 其中PQ 是上次 icp 的结果, 去畸变后应当及时 T到最后一个点的位置
pcl::PointCloud<pcl::PointXYZI>
main_function::IMUDistortion(VLPPointCloud point_in, Eigen::Isometry3d PQ, Eigen::Vector3d V, int ImuIndex) {
	PQ = Eigen::Isometry3d::Identity();// IMU系 除了velocity 都没啥用
	Eigen::Isometry3d PQ_Dist;
	Eigen::Vector3d V_Dist;
	PQ_Dist = PQ;
	V_Dist = V;
	std::vector<VLPPointCloud> imu_pcd;
	std::vector<pcl::PointCloud<pcl::PointXYZI>> imu_pcd_xyz;
	std::vector<Eigen::Isometry3d> imu_DT,imu_absT; //放 每个imu measurement的变化量
	imu_pcd.resize(13);//12.5hz +_ 1
	imu_DT.resize(13);
	imu_absT.resize(13);
	imu_pcd_xyz.resize(13);
	for (int m = 0; m < 13; ++m) {
		imu_absT[m].setIdentity();
	}
	
	pcl::PointCloud<pcl::PointXYZI> result;
	pcl::PointCloud<pcl::PointXYZI> onepoint;
	//当前点的位姿 = T last * T distortion
	Eigen::Isometry3d current_t = Eigen::Isometry3d::Identity();
	//1.0 点云按imu时间戳排列
	for (int i = 0; i < point_in.size(); ++i) {
		for (int j = 0; j < 13; ++j) {
			if (point_in[i].time>=IMUdata[ImuIndex+j][6] && point_in[i].time<IMUdata[ImuIndex+j+1][6]){
				imu_pcd[j].push_back(point_in[i]);
				pcl::PointXYZI temp;
				temp.x = point_in[i].x;
				temp.y = point_in[i].y;
				temp.z = point_in[i].z;
				temp.intensity = point_in[i].intensity;
				imu_pcd_xyz[j].push_back(temp);
				break;
			}
		}
	}
	//1.1 得到各个点的delta T 每次imu之间的姿态变化量
	for (int k = 0; k < 13; ++k) {
		IMUIntergrate(PQ_Dist,V_Dist,ImuIndex +  k);
		imu_DT.push_back(PQ*PQ_Dist.inverse());
//		std::cout<<"delta T: \n"<<PQ*PQ_Dist.inverse().matrix()<<std::endl;//感觉数值都挺正常
		PQ = PQ_Dist;
		imu_absT[k] = PQ_Dist;
//		std::cout<<k<<" 相对雷达起点的位姿:\n"<<PQ_Dist.matrix()<<std::endl;
	}
	//1.2 每组去个畸变
	
	for (int l = 0; l < imu_pcd_xyz.size(); ++l) {
//		current_t = PQ*imu_absT[l].inverse();
		current_t = imu_absT[l] ;
//		onepoint = imu_pcd_xyz[l];
//		std::cout<<l<<" 相对雷达起点的位姿:\n"<<current_t.matrix()<<std::endl;
		pcl::transformPointCloud(imu_pcd_xyz[l],onepoint, current_t.matrix());//一块一块tf
		//simpleDistortion(imu_pcd[l],imu_DT[l].matrix(),onepoint);
		result += onepoint;
	}
	//2.去畸变
	return result;
}
//1.2.2 获得速度
Eigen::Vector3d main_function::GetVelocityOfbody(Eigen::Isometry3d last, Eigen::Isometry3d cur, int ImuIndex) {
	Eigen::Vector3d velocity;
	Eigen::Isometry3d DeltaT ,pose;

	DeltaT = last.inverse().matrix() * cur.matrix();
	std::cout<<"雷达测出来的变化量 x: "<<DeltaT.matrix()(0,3)<<" y: "<<DeltaT.matrix()(1,3)<<" z: "<<DeltaT.matrix()(2,3)<<std::endl;
	pose = last*(0.5*DeltaT.matrix());
	velocity[0]=  DeltaT(0,3)/0.1;
	velocity[1]=  DeltaT(1,3)/0.1;
	velocity[2]=  DeltaT(2,3)/0.1;
	std::cout<<"雷达测出来的中间点速度  : "<<velocity.transpose()<<std::endl;
	IMUIntergrate(pose,velocity,ImuIndex-5);
	IMUIntergrate(pose,velocity,ImuIndex-4);
	IMUIntergrate(pose,velocity,ImuIndex-3);
	IMUIntergrate(pose,velocity,ImuIndex-2);
	IMUIntergrate(pose,velocity,ImuIndex-1);
	IMUIntergrate(pose,velocity,ImuIndex);
	std::cout<<"imu测出来的上帧结束点的速度: "<<velocity.transpose()<<std::endl;
	return velocity;
}
