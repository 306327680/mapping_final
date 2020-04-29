//
// Created by echo on 2020/4/26.
//

#include "GPS_loop_mapping.h"

void
GPS_loop_mapping::GPSandPose(std::string lidar_pose, std::string gps_constraint, Eigen::Isometry3d extrinsic_matrix) {
	ceres::examples::MapOfPoses poses;//这个项目中定义的map pose
	ceres::examples::VectorOfConstraints constraints;//这个项目中定义的 约束向量
	ceres::examples::ReadG2oFile(lidar_pose, &poses, &constraints);//读取g2o文件
	std::vector<std::pair<int,Eigen::Vector3d>> relation;//gps和位姿的映射
	RelationG2OGPS("/media/echo/DataDisc/3_program/mapping/cmake-build-release/LiDAR_pose.csv",
			"/media/echo/DataDisc/3_program/mapping/cmake-build-release/test.csv",relation);//获得gps和位姿的映射
	poseTF(poses,extrinsic_matrix);
	std::cout<<constraints.size()<<" "<<poses.size()<<std::endl;
	ceres::Problem problem;
	BuildOptimizationProblem(constraints, &poses, &problem);
	SolveOptimizationProblem(&problem);
	OutputPoses("poses_optimized.txt", poses);
}

void GPS_loop_mapping::poseTF(ceres::examples::MapOfPoses &input, Eigen::Isometry3d trans) {
	pcl::PointCloud<pcl::PointXYZ> tfed;
	pcl::PointCloud<pcl::PointXYZ> bef;
	PoseGraphIO wtf;
	for (int i = 0; i < input.size(); ++i) {
		Eigen::Isometry3d tf_ed =Eigen::Isometry3d::Identity();
		
		tf_ed.translate(input[i].p);
		tf_ed.rotate(input[i].q);
//		std::cout<< "tf_ed"<< tf_ed.matrix()<<std::endl;
//		std::cout<< "trans"<<trans.matrix()<<std::endl;
		bef.push_back(pcl::PointXYZ(input[i].p(0),input[i].p(1),input[i].p(2)));
		tf_ed = trans.matrix().inverse()*tf_ed.matrix();
//		std::cout<<"tf_ed1"<< tf_ed.matrix()<<std::endl;
		input[i].q = Eigen::Quaterniond(tf_ed.rotation());
		input[i].p(0) =tf_ed.matrix()(0,3);
		input[i].p(1) =tf_ed.matrix()(1,3);
		input[i].p(2) =tf_ed.matrix()(2,3);
		wtf.insertPose(tf_ed);
//		std::cout<<">"<<input[i].p<<std::endl;
		tfed.push_back(pcl::PointXYZ(input[i].p(0),input[i].p(1),input[i].p(2)));
	}
	wtf.saveGraph("a.g2o");
	pcl::PCDWriter writer;
	writer.write("tfed.pcd",tfed);
	writer.write("bef.pcd",bef);
}


//没写完,回头继续写,直接读csv,然后找到时间最近的构建因子图
template<typename Pose, typename Constraint, typename MapAllocator, typename VectorAllocator>
void GPS_loop_mapping::G2OandPCD(std::string lidar_pose, std::string gps_constraint,
								 std::map<int, Pose, std::less<int>, MapAllocator> *poses,
								 std::vector<Constraint, VectorAllocator> *constraints) {
	
}
//找到gps和lidar对应关系
void GPS_loop_mapping::RelationG2OGPS(std::string lidar_pose, std::string gps_constraint,
									  std::vector<std::pair<int, Eigen::Vector3d>> &relation) {
	std::ifstream fin(lidar_pose); //打开文件流操作
	std::string line;
	//读取lidar
	std::vector<Eigen::Vector2d> index_time;
	std::vector<Eigen::Vector4d> pose_time;
	double index_lidar = 0;
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		index_lidar++;
		std::cout <<"原始字符串："<< line << std::endl; //整行输出
		std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		std::vector<std::string> fields; //声明一个字符串向量
		std::string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field); 			//将刚刚读取的字符串添加到向量fields中
		}
		std::string time = fields[13]; 			//清除掉向量fields中第一个元素的无效字符，并赋值给变量name
		index_time.push_back(Eigen::Vector2d(index_lidar, std::atof(time.c_str())));
	}
	
	std::ifstream gin(gps_constraint);
	while (getline(gin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		std::cout <<"原始字符串："<< line << std::endl; //整行输出
		std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		std::vector<std::string> fields; //声明一个字符串向量
		std::string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field); 			//将刚刚读取的字符串添加到向量fields中
		}
		std::string x = fields[0]; 			//清除掉向量fields中第一个元素的无效字符，并赋值给变量name
		std::string y = fields[1]; 			//清除掉向量fields中第二个元素的无效字符，并赋值给变量age
		std::string z = fields[2]; 		//清除掉向量fields中第三个元素的无效字符，并赋值给变量birthday
		std::string time = fields[15]; 		//清除掉向量fields中第三个元素的无效字符，并赋值给变量birthday
		std::string status = fields[12]; 		//清除掉向量fields中第三个元素的无效字符，并赋值给变量birthday
		if(status=="2"){
			pose_time.push_back(Eigen::Vector4d((double)std::atof(x.c_str()),(double)std::atof(y.c_str()),(double)std::atof(z.c_str()),(double)std::atof(time.c_str())));
			std::cout<<pose_time.back()<<std::endl;
		}
	}
	int related_index = 0;
	for (int i = 0; i < index_time.size(); ++i) {
		if(fabs(index_time[i](1)-pose_time[related_index](3))<=0.05){

			std::cout<<"lidar: "<<index_time[i](1)<<" gps: "<<pose_time[related_index](3)<<std::endl;
			related_index++;
		}
	}
	
}

