//
// Created by echo on 2020/4/26.
//

#include "GPS_loop_mapping.h"

void
GPS_loop_mapping::	GPSandPose(std::string lidar_pose, std::string gps_constraint, Eigen::Isometry3d extrinsic_matrix) {
	ceres::examples::MapOfPoses poses;//这个项目中定义的map pose
	ceres::examples::VectorOfConstraints constraints;//这个项目中定义的 约束向量
	ceres::examples::ReadG2oFile(lidar_pose, &poses, &constraints);//读取g2o文件
	std::vector<std::pair<int,Eigen::Vector3d>> relation;//gps和位姿的映射
	PoseGraphIO wtf;//保存结果
	RelationG2OGPS("/home/echo/shandong_in__out/LiDAR_pose.csv","/home/echo/shandong_in__out/gps.csv",relation);//获得gps和位姿的映射
	poseTF(poses,extrinsic_matrix);//旋转当前LiDAR 位姿
	ceres::Problem problem;
	BuildOptimizationProblem(constraints, &poses, &problem);			        //闭环的约束项
	BuildGPSOptimizationProblem(constraints, &poses, &problem,relation);            //gps的约束项
	SolveOptimizationProblem(&problem);						//解决
	OutputPoses("poses_optimized1.txt", poses);
	pcl::PointCloud<pcl::PointXYZ> tfed;
	for (int i = 0; i < poses.size(); ++i) {
		Eigen::Isometry3d tf_ed =Eigen::Isometry3d::Identity();
		tf_ed.translate(poses[i].p);
		tf_ed.rotate(poses[i].q);
		wtf.insertPose(tf_ed);
		tfed.push_back(pcl::PointXYZ(poses[i].p(0),poses[i].p(1),poses[i].p(2)));
	}
	pcl::PCDWriter writer;
	writer.write("gps_constrained.pcd",tfed);
	wtf.saveGraph("gps_constrained.g2o");
}

void GPS_loop_mapping::poseTF(ceres::examples::MapOfPoses &input, Eigen::Isometry3d trans) {
	pcl::PointCloud<pcl::PointXYZ> tfed;
	pcl::PointCloud<pcl::PointXYZ> bef;
	PoseGraphIO wtf;
	for (int i = 0; i < input.size(); ++i) {
		Eigen::Isometry3d tf_ed =Eigen::Isometry3d::Identity();
		
		tf_ed.translate(input[i].p);
		tf_ed.rotate(input[i].q);
		bef.push_back(pcl::PointXYZ(input[i].p(0),input[i].p(1),input[i].p(2)));
		tf_ed = trans.matrix().inverse()*tf_ed.matrix();
 
		input[i].q = Eigen::Quaterniond(tf_ed.rotation());
		input[i].p(0) =tf_ed.matrix()(0,3);
		input[i].p(1) =tf_ed.matrix()(1,3);
		input[i].p(2) =tf_ed.matrix()(2,3);
		wtf.insertPose(tf_ed);
		tfed.push_back(pcl::PointXYZ(input[i].p(0),input[i].p(1),input[i].p(2)));
	}
	wtf.saveGraph("aligned.g2o");
	pcl::PCDWriter writer;
	writer.write("aligned.pcd",tfed);
	writer.write("raw_lidar.pcd",bef);
	std::cout<<"LiDAR tf to GPS cordinate"<<std::endl;
}


//没写完,回头继续写,直接读csv,然后找到时间最近的构建因子图
template<typename Pose, typename Constraint, typename MapAllocator, typename VectorAllocator>
void GPS_loop_mapping::G2OandPCD(std::string lidar_pose, std::string gps_constraint,
								 std::map<int, Pose, std::less<int>, MapAllocator> *poses,
								 std::vector<Constraint, VectorAllocator> *constraints) {
	
}
//找到gps和lidar对应关系 可以放到csvio
void GPS_loop_mapping::RelationG2OGPS(std::string lidar_pose, std::string gps_constraint,
									  std::vector<std::pair<int, Eigen::Vector3d>> &relation) {
	std::ifstream fin(lidar_pose); //打开文件流操作
	std::string line;
	//读取lidar
	std::vector<Eigen::Vector2d> index_time;
	std::vector<Eigen::Vector4d> pose_time;
	std::pair<int, Eigen::Vector3d> one_relation;
	double index_lidar = 0;
	//读取LiDAR csv
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		index_lidar++;
		std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		std::vector<std::string> fields; //声明一个字符串向量
		std::string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field);
		}
		std::string time = fields[13]; 			//读取时间戳
		index_time.push_back(Eigen::Vector2d(index_lidar, std::atof(time.c_str())));
	}
	//读取GPS CSV
	std::ifstream gin(gps_constraint);
	while (getline(gin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		std::vector<std::string> fields; //声明一个字符串向量
		std::string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field); 			//将刚刚读取的字符串添加到向量fields中
		}
		std::string x = fields[0];
		std::string y = fields[1];
		std::string z = fields[2];
		std::string conv_xy = fields[3];
		std::string conv_z = fields[11];
		std::string time = fields[15];
		std::string status = fields[12];
		float fconv_xy = std::atof(conv_xy.c_str());
		if((status=="2")){
		/*	if((status=="2"||status=="1")&&fconv_xy<3.0){	*/  //如果是rtk模式就加入candidate,之后可以加入其他的约束,例如cov 协方差等
			pose_time.push_back(Eigen::Vector4d((double)std::atof(x.c_str()),(double)std::atof(y.c_str()),(double)std::atof(z.c_str()),(double)std::atof(time.c_str())));
		}
	}

	//找到时间戳关联
	int related_index = 0;
/*	for (int i = 0; i < index_time.size()-5; ++i) {
	 	float time_diff = (index_time[i](1)-pose_time[related_index](3));
	 	std::cout<<time_diff<<std::endl;
		if(fabs(time_diff)<=0.07){
			std::cout<<"lidar: "<<index_time[i](1)<<" gps: "<<pose_time[related_index](3)<<" diff: "
			<<fabs(index_time[i](1)-pose_time[related_index](3))<<std::endl;
			one_relation.first = index_time[i](0);//放入点云id
			one_relation.second = Eigen::Vector3d(pose_time[related_index](0),
					pose_time[related_index](1),pose_time[related_index](2));//放入gps点坐标
			relation.push_back(one_relation);
			related_index++;
		}
	}*/
	std::cout<<"gps: "<<pose_time.size()<<std::endl;
	for (int i = 0; i < index_time.size(); ++i) {
		for (int j = related_index; j < pose_time.size(); ++j) {
			float time_diff = (index_time[i](1)-0.1-pose_time[j](3));//-0.1 是对齐时间
			if(fabs(time_diff)<=0.05){
/*				std::cout<<"lidar: "<<index_time[i](1)<<" gps: "<<pose_time[j](3)<<" diff: "
						 <<fabs(index_time[i](1)-pose_time[j](3))<<std::endl;*/
				one_relation.first = index_time[i](0);//放入点云id
				one_relation.second = Eigen::Vector3d(pose_time[j](0),
													  pose_time[j](1),pose_time[j](2));//放入gps点坐标
			/*	if(	one_relation.first <7700){//地库附近不加gps约束*/
					relation.push_back(one_relation);
		/*		}*/
				related_index = j;
				continue;
			}
		}
	}
	//状态为2的gps
	pcl::PointCloud<pcl::PointXYZ> tfed;
	for (int i = 0; i < relation.size(); ++i) {
		tfed.push_back(pcl::PointXYZ(relation[i].second(0),relation[i].second(1),relation[i].second(2)));
	}
	pcl::PCDWriter writer;
	writer.write("select_gps.pcd",tfed);
}
//对应关系 带information
void GPS_loop_mapping::RelationG2OGPS(std::string lidar_pose, std::string gps_constraint,
									  std::vector<std::pair<int, Eigen::Vector3d>> &relation,
									  std::vector<Eigen::Vector3d> &conv_gps) {
	std::ifstream fin(lidar_pose); //打开文件流操作
	std::string line;
	//读取lidar
	std::vector<Eigen::Vector2d> index_time;
	std::vector<Eigen::Vector4d> pose_time;
	std::pair<int, Eigen::Vector3d> one_relation;
	conv_gps.clear();
	double index_lidar = 0;
	//读取LiDAR csv
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		index_lidar++;
		std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		std::vector<std::string> fields; //声明一个字符串向量
		std::string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field);
		}
		std::string time = fields[13]; 			//读取时间戳
		index_time.push_back(Eigen::Vector2d(index_lidar, std::atof(time.c_str())));
	}
	//读取GPS CSV
	std::ifstream gin(gps_constraint);
	while (getline(gin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		std::vector<std::string> fields; //声明一个字符串向量
		std::string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field); 			//将刚刚读取的字符串添加到向量fields中
		}
		std::string x = fields[0];
		std::string y = fields[1];
		std::string z = fields[2];
		std::string conv_xy = fields[3];
		std::string conv_z = fields[11];
		std::string time = fields[15];
		std::string status = fields[12];
		float fconv_xy = std::atof(conv_xy.c_str());
		if(status=="2"&&fconv_xy<1){	  //如果是rtk模式就加入candidate,之后可以加入其他的约束,例如cov 协方差等
			pose_time.push_back(Eigen::Vector4d((double)std::atof(x.c_str()),(double)std::atof(y.c_str()),(double)std::atof(z.c_str()),(double)std::atof(time.c_str())));
			conv_gps.push_back(Eigen::Vector3d((double)std::atof(conv_xy.c_str()),(double)std::atof(conv_xy.c_str()),(double)std::atof(conv_z.c_str())));
			std::cout<<pose_time.back()<<std::endl;
		}
	}
	//状态为2的gps
	pcl::PointCloud<pcl::PointXYZ> tfed;
	for (int i = 0; i < pose_time.size(); ++i) {
		tfed.push_back(pcl::PointXYZ(pose_time[i](0),pose_time[i](1),pose_time[i](2)));
	}
	pcl::PCDWriter writer;
	writer.write("select_gps.pcd",tfed);
	//找到时间戳关联
	int related_index = 0;
	for (int i = 0; i < index_time.size(); ++i) {
		if(fabs(index_time[i](1)-pose_time[related_index](3))<=0.05){
			std::cout<<"lidar: "<<index_time[i](1)<<" gps: "<<pose_time[related_index](3)<<" diff: "
					 <<fabs(index_time[i](1)-pose_time[related_index](3))<<std::endl;
			one_relation.first = index_time[i](0);//放入点云id
			one_relation.second = Eigen::Vector3d(pose_time[related_index](0),
												  pose_time[related_index](1),
												  pose_time[related_index](2));//放入gps点坐标
			relation.push_back(one_relation);
			related_index++;
		}
	}
}

void GPS_loop_mapping::BuildOptimizationProblem(const ceres::examples::VectorOfConstraints &constraints,
												ceres::examples::MapOfPoses *poses, ceres::Problem *problem) {
	CHECK(poses != NULL);
	CHECK(problem != NULL);
	if (constraints.empty()) {
		LOG(INFO) << "No constraints, no problem to optimize.";
		return;
	}
	
	ceres::LossFunction* loss_function = NULL;
	ceres::LocalParameterization* quaternion_local_parameterization =new ceres::EigenQuaternionParameterization;//函数就是定义四元数的加
	//这里吧所有的边都扔进去了
	for (ceres::examples::VectorOfConstraints::const_iterator constraints_iter = constraints.begin();
		 constraints_iter != constraints.end(); ++constraints_iter) {//迭代约束
		const ceres::examples::Constraint3d& constraint = *constraints_iter;//iterator是指针
		//确定开始结束 pose_begin_iter->poses pose_end_iter->poses  每种容器类型都定义了自己的迭代器类型，
		ceres::examples::MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);//当前的约束的from
		ceres::examples::MapOfPoses::iterator pose_end_iter   = poses->find(constraint.id_end);//当前约束的to
		//信息矩阵
		Eigen::Matrix<double, 6, 6> sqrt_information = constraint.information.llt().matrixL();
//		sqrt_information = Eigen::Matrix<double, 6,6>::Identity();
//		sqrt_information(4,4) = 0.98;
		// 自定义的pose graph error项 把约束的测量放进去(闭环) 这里放入的是测量边
		ceres::CostFunction* cost_function = ceres::examples::PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);
		//后4个是要优化的量 需要调整的位姿,邮储结果通过这个传出来,这些值要被调整的 最后pose是被调整了
		problem->AddResidualBlock(cost_function, loss_function,
								  pose_begin_iter->second.p.data(),
								  pose_begin_iter->second.q.coeffs().data(),
								  pose_end_iter->second.p.data(),
								  pose_end_iter->second.q.coeffs().data());
		
		problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
									 quaternion_local_parameterization);
		problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
									 quaternion_local_parameterization);
	}
	//需要设定一个node为constant
	ceres::examples::MapOfPoses::iterator pose_start_iter = poses->begin();
	CHECK(pose_start_iter != poses->end()) << "There are no poses.";
	//node 0 为常量
/*	problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
	problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());*/
	std::cout<<" loop added to Pose Graph"<<std::endl;
}

void GPS_loop_mapping::BuildGPSOptimizationProblem(const ceres::examples::VectorOfConstraints &constraints,
												   ceres::examples::MapOfPoses *poses, ceres::Problem *problem,
												   std::vector<std::pair<int, Eigen::Vector3d>> relation) {
	std::cout<<"there are: "<<relation.size()<<" gps constraints"<<std::endl;
	int index = 0;
	
	ceres::LossFunction* loss_function = NULL;
//	ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.5);
	ceres::LocalParameterization* quaternion_local_parameterization =new ceres::EigenQuaternionParameterization;//函数就是定义四元数的加
	
	
	for (ceres::examples::VectorOfConstraints::const_iterator constraints_iter = constraints.begin();
		 constraints_iter != constraints.end(); ++constraints_iter) {//迭代约束
		const ceres::examples::Constraint3d& constraint = *constraints_iter;//iterator是指针
		//确定开始结束 pose_begin_iter->poses pose_end_iter->poses  每种容器类型都定义了自己的迭代器类型，
		ceres::examples::MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);//当前的约束的from
		ceres::examples::MapOfPoses::iterator pose_end_iter   = poses->find(constraint.id_end);//当前约束的to
		for (int i = 0; i < relation.size(); ++i) {
			if(relation[i].first==index){//当前的gps构成了约束
				const Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::Matrix<double, 6, 6>::Identity();
				ceres::CostFunction* cost_function = ceres::examples::PoseGraphGPSErrorTerm::Create(relation[i].second, sqrt_information);
				problem->AddResidualBlock(cost_function, loss_function,
										  pose_begin_iter->second.p.data(),
										  pose_begin_iter->second.q.coeffs().data());
				continue;
			}
		}
		index++;
	}
	std::cout<<" GPS added to Pose Graph"<<std::endl;
}



