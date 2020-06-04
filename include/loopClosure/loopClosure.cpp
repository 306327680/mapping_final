//
// Created by echo on 2020/4/15.
//

#include "loopClosure.h"

bool loopClosure::GetFileNames(const std::string directory, const std::string suffix) {
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

bool loopClosure::FindFileseq(int64_t seq){
	int64_t idx_file = seq;
	if (idx_file > file_names_.size()-1) {
		return INT64_MAX;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	uint64_t idx_seperator = file_names_[idx_file].find_last_of("/");
	filename = file_names_[idx_file].substr(idx_seperator + 1);
	//std::cout<<"PCD file is "<<filename<<" seq is: "<<idx_file<<std::endl;
	return true;
}

void loopClosure::saveFile(std::string outFilename, std::vector<VertexSE3 *> vertices, std::vector<EdgeSE3 *> edges) {
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
	std::cout<<"\033[34m"<<"g2o saved as "<<outFilename<<std::endl;
}
void loopClosure::SaveLiDAREdge(){
 
	Eigen::Matrix<double, 6, 6> _information = Eigen::Matrix<double, 6, 6>::Identity();//信息矩阵
	for (int i = 0; i < trans_vector.size(); ++i) {
		g2o::VertexSE3 *v = new g2o::VertexSE3();
		
		if (i == 0){
			v->setFixed(true);
		}
		//顶点
		v->setId(i);
		v->setEstimate(trans_vector[i]);
		vertices.push_back(v);
		optimizer.addVertex(v);
		// 生成边
		if (i > 0) {
			Eigen::Matrix4d t_e_m;
			t_e_m = trans_vector[i-1].matrix();
			Eigen::Isometry3d t_e;
			t_e = t_e_m.inverse()* trans_vector[i].matrix() ;
			g2o::EdgeSE3 *e = new g2o::EdgeSE3();
			e->setVertex(0, optimizer.vertices()[i-1]); //debug
			e->setVertex(1, optimizer.vertices()[i]); //debug
			e->setMeasurement(t_e); //debug
			e->setInformation(_information);
			edges.push_back(e);
			optimizer.addEdge(e);
		}
	}
}
void loopClosure::SaveTrans(Eigen::Isometry3d curr) {
	Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
	VertexSE3 *cur = new VertexSE3;
	VertexSE3 *prev = new VertexSE3;;
	cur->setId(past);
	Eigen::Isometry3d t_v = Eigen::Isometry3d::Identity();
	cur->setEstimate(t_v);
	prev->setId(cur_id);
	prev->setEstimate(curr);
	//vertices.push_back(prev);
	EdgeSE3 *e = new EdgeSE3;
	e->setVertex(0, prev);
	e->setVertex(1, cur);
	e->setMeasurement(curr); //debug
	e->setInformation(information);
	edges.push_back(e);
}

std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
loopClosure::getEigenPoseFromg2oFile(std::string &g2ofilename) {
	std::ifstream fin(g2ofilename);
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
pcl::PointCloud<pcl::PointXYZ> loopClosure::NDT(const pcl::PointCloud<pcl::PointXYZ> &cloud_source,
												 const pcl::PointCloud<pcl::PointXYZ> &cloud_target,
												 Eigen::Isometry3d &icp_matrix) {
	pcl::PointCloud<pcl::PointXYZ> result;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setMaximumIterations(5000);
	ndt.setMaxCorrespondenceDistance(10);
	ndt.setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_source)));
	ndt.setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_target)));
	ndt.align(result);
	std::cout<<"curr align score: "<<ndt.getFitnessScore()<<std::endl;
	Eigen::Matrix4f tf_s2t = ndt.getFinalTransformation();
	icp_matrix = tf_s2t.cast<double>();
	//std::cout << tf_s2t << std::endl;
	return result; // source上应用这个矩阵，就可以转过去了
}
pcl::PointCloud<pcl::PointXYZ> loopClosure::NDT_OMP(const pcl::PointCloud<pcl::PointXYZ> &cloud_source,
												const pcl::PointCloud<pcl::PointXYZ> &cloud_target,
												Eigen::Isometry3d &icp_matrix) {
	pcl::PointCloud<pcl::PointXYZ> result;
	pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.000801);
	ndt.setMaximumIterations(100);
	ndt.setMaxCorrespondenceDistance(50);
	ndt.setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_source)));
	ndt.setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_target)));
	ndt.align(result);

	Eigen::Matrix4f tf_s2t = ndt.getFinalTransformation();
	icp_matrix = tf_s2t.cast<double>();
	ndt_score = ndt.getFitnessScore();
	//std::cout <<" ndt omp result : "<< tf_s2t << std::endl;
	return result;
}

pcl::PointCloud<pcl::PointXYZ> loopClosure::GICP(const pcl::PointCloud<pcl::PointXYZ> &cloud_source,
												 const pcl::PointCloud<pcl::PointXYZ> &cloud_target,
												 Eigen::Isometry3d &icp_matrix) {
	pcl::PointCloud<pcl::PointXYZ> result;
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp; //创建ICP对象，用于ICP配准
	gicp.setTransformationEpsilon(0.00001);
	gicp.setMaximumIterations(5000);
	gicp.setMaxCorrespondenceDistance(20);
	gicp.setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_source)));
	gicp.setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_target)));
	gicp.align(result);
	std::cout<<"curr align score: "<<gicp.getFitnessScore()<<std::endl;
	Eigen::Matrix4f tf_s2t = gicp.getFinalTransformation();
	icp_matrix = tf_s2t.cast<double>();
	std::cout << tf_s2t << std::endl;
	return result; // source上应用这个矩阵，就可以转过去了

}
pcl::PointCloud<pcl::PointXYZ> loopClosure::NormalICP(const pcl::PointCloud<pcl::PointXYZ> & cloud_source,
										 const pcl::PointCloud<pcl::PointXYZ> & cloud_target,
													  Eigen::Isometry3d init_pose,Eigen::Isometry3d & icp_matrix){
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(
			new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	//set up
	icp->setMaximumIterations(100);
	icp->setMaxCorrespondenceDistance(5);
	icp->setTransformationEpsilon(0.1);
	icp->setEuclideanFitnessEpsilon(0.01);
	//calc nromal
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimator_pa;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals_source(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals_target(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
	*temp = cloud_source;
	//source
	pcl::copyPointCloud(cloud_source,*cloud_source_);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimator_pa.setInputCloud(temp);
	normalEstimator_pa.setSearchMethod(searchTree);
	normalEstimator_pa.setKSearch(20);
	normalEstimator_pa.compute(*normals);
	pcl::concatenateFields(*cloud_source_, *normals, *cloud_with_normals_source);
	//target
	cloud_source_normals->clear();
	normals->clear();
	*temp = cloud_target;
	pcl::copyPointCloud(cloud_target,*cloud_source_);
	normalEstimator_pa.setInputCloud(temp);
	normalEstimator_pa.setSearchMethod(searchTree);
	normalEstimator_pa.setKSearch(20);
	normalEstimator_pa.compute(*normals);
	pcl::concatenateFields(*cloud_source_, *normals, *cloud_with_normals_target);
	//align
	icp->setInputSource(cloud_with_normals_source);
	icp->setInputTarget(cloud_with_normals_target);
	icp->align(*cloud_source_normals);
	icp_matrix = icp->getFinalTransformation().matrix().cast<double>();
	pcl::PointCloud<pcl::PointXYZ> resultXYZ;
	pcl::copyPointCloud(*cloud_source_normals,resultXYZ);
	std::cout<<icp->getFitnessScore()<<" result: "<<icp_matrix.matrix()<<std::endl;
	ndt_score = icp->getFitnessScore();
	return resultXYZ;
	
}

void loopClosure::genlocalmap(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
							  int num1, std::string filepath, pcl::PointCloud<pcl::PointXYZ> &bigmap) {
	std::cout<<num1<<std::endl;
	std::string core_pcd = file_names_[num1];
	
	int upper_bound = trans_vector.size();
	int lower_bound = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aft(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bef(
			new pcl::PointCloud<pcl::PointXYZ>);
	
	for(int i = num1; i>=lower_bound && num1-i<30;i--){
		std::cout<<i<<" lower_bound "<<file_names_[i]<<std::endl;
		std::vector<int> indices1;
		//读点云
		pcl::io::loadPCDFile<pcl::PointXYZ>(file_names_[i],*cloud_bef);
		pcl::removeNaNFromPointCloud(*cloud_bef, *cloud_bef, indices1);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud_bef);                   //设置需要过滤的点云给滤波对象
		sor.setLeafSize(0.2,0.2,0.2);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
		sor.filter(*cloud_aft);
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(cloud_aft);
		outrem.setRadiusSearch(0.5);
		outrem.setMinNeighborsInRadius (3);
		outrem.filter (*cloud_bef);
		pcl::transformPointCloud(*cloud_bef, *cloud_aft, trans_vector[i].matrix());
		*cloud_add += *cloud_aft;
	}
	for(int i = num1 + 1; i<upper_bound && i-num1<30 ;i++){
		std::cout<<i<<" upper_bound "<<file_names_[i]<<std::endl;
		std::vector<int> indices1;
		//读点云
		pcl::io::loadPCDFile<pcl::PointXYZ>(file_names_[i],*cloud_bef);
		pcl::removeNaNFromPointCloud(*cloud_bef, *cloud_bef, indices1);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud_bef);                   //设置需要过滤的点云给滤波对象
		sor.setLeafSize(0.2,0.2,0.2);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
		sor.filter(*cloud_aft);
		//new
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(cloud_aft);
		outrem.setRadiusSearch(0.5);
		outrem.setMinNeighborsInRadius (3);
		outrem.filter (*cloud_bef);
		pcl::transformPointCloud(*cloud_bef, *cloud_aft, trans_vector[i].matrix());
		*cloud_add += *cloud_aft;
	}
	//转换回(0,0,0,0,0,0)
	pcl::transformPointCloud(*cloud_add, *cloud_aft, trans_vector[num1].inverse().matrix());
	//全点云时候应当加
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_aft);                   //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.2,0.2,0.2);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
	sor.filter(*cloud_add);                      //执行滤波处理，存储输出
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud_add);
	outrem.setRadiusSearch(0.5);
	outrem.setMinNeighborsInRadius (5);
	// 应用滤波器
	outrem.filter (*cloud_aft);
	bigmap = *cloud_aft;
	std::cout<<"small map done index is : "<<num1<<" size: "<<bigmap.size()<<std::endl;
}

void loopClosure::addLoopEdge(int num1, int num2, std::string g2o_read, std::string g2o_save, std::string pcd_path) {
	std::string pcd_name1,pcd_name2,file_path1,file_path2;
	filepath = pcd_path;
	GetFileNames(pcd_path,"pcd");
	trans_vector = getEigenPoseFromg2oFile(g2o_read);
	past = num1;
	cur_id = num2;
	//在这里找到 序列
	FindFileseq(num1);
	pcd_name1 = filename;
	FindFileseq(num2);
	pcd_name2 = filename;
	
	file_path1=filepath+pcd_name1;                            //自己改路径
	file_path2=filepath+pcd_name2;                            //自己改路径
	
	if(pcd_name1.empty()) {
		std::cout << "pcd_name1: can not find suitable pcd file\n";
	}if(pcd_name2.empty()) {
		std::cout << "pcd_name2: can not find suitable pcd file\n";
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);  //创建点云
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path1,*cloud1)==-1){                   //读入PCD文件
		PCL_ERROR("Couldn't read pcd_file1 \n");                                                   //文件不存在
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path2,*cloud2)==-1){
		PCL_ERROR("Couldn't read pcd_file2 \n");
	}
	std::cout<<"Loaded "<<cloud1->width * cloud1->height<<" data points from " << pcd_name1 <<std::endl;
	std::cout<<"Loaded "<<cloud2->width * cloud2->height<<" data points from " << pcd_name2 <<std::endl;
	//生成局部地图

//进行点云icp配准
	std::vector<int> indices1;
	pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices1);
	std::vector<int> indices2;
	pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices2);
	
	genlocalmap(trans_vector,past,filepath,*cloud1);
	genlocalmap(trans_vector,cur_id,filepath,*cloud2);
 
	//初始位姿
	Eigen::Isometry3d init_pose = trans_vector[past].inverse()*trans_vector[cur_id];
	init_pose = init_pose.inverse();
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud1, *temp, init_pose.matrix());
	*cloud1 = *temp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
 
//  icp.align(Final_cloud);
	Eigen::Isometry3d gicp_result;
	*Final_cloud = GICP(*cloud1,*cloud2,gicp_result);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	final_cloud = Final_cloud;
	Eigen::Isometry3d icp_matrix;
	icp_matrix = gicp_result;
	std::cout<<icp_matrix*init_pose.inverse().matrix().inverse()<<std::endl;
	//得到最终的transform
	curICP = icp_matrix*init_pose.inverse().matrix().inverse();
	genlocalmap(trans_vector,past,filepath,*cloud1);
	genlocalmap(trans_vector,cur_id,filepath,*cloud2);
	pcl::transformPointCloud(*cloud1, *final_cloud, curICP.matrix());
	SaveTrans(curICP);
	saveFile(save_g2o_path,vertices,edges);
//对转换后的点云着色可视化 显示匹配前后的点云
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);                              //设置背景颜色
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> output_color (Final_cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> output_color_2 (cloud2, 255, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(Final_cloud, output_color, "Final cloud");
	viewer->addPointCloud<pcl::PointXYZ>(cloud2, output_color_2, "Final cloud3");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Final cloud");
	viewer->addCoordinateSystem (1.0);

//等待直到可视化窗口关闭
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
 
}
//********************************************
void
loopClosure::autoMaticLoopClosure(std::string LiDAR_g2o_read, std::string Final_g2o_save, std::string LiDAR_pcd_path,
								  std::string GPScsvPath, std::string LiDARcsv) {

	//1.得到雷达里程计
	trans_vector = getEigenPoseFromg2oFile(LiDAR_g2o_read);
	//2.得到点云路径 之后可以用 FindFileseq(num1);
	filepath = LiDAR_pcd_path;
    //2.1 构建lidar odom的factor
	SaveLiDAREdge();
/*	//3.读取GPS 时间位置 gps 被赋值
	gps2pcd(GPScsvPath);
	//4.读取LiDAR 时间
	LiDAR2pcd(LiDARcsv);
	//5.找到correspondece
	LiDARmatchIndex = LiDAR_GPS_math();
	//6.进行gps的 闭环 选取
	findLoopFromGPS();
	//7. gps 的匹配初值给定 算当前的lidar-gps 和loop-closure初始值 + 进行匹配
	GPS_align_TF_calc();
	//8. add result to pose graph
	saveFile(save_g2o_path,vertices,edges);*/
	//9. optimize 先进行一次factor graph优化
	GPSLoopClosureCalc("/home/echo/autoLoop.g2o");
	//10. 通过当前的odom 找到 correspondence
	findLoopFromOdom();
	//11. Add loop by optimize result /find hidden loop
	Odom_align_TF_calc();
	//12. save final graph
	saveFile("/home/echo/shandong_in__out/autoOdomLoop.g2o",vertices,edges);
}

void loopClosure::gps2pcd(std::string GPScsvPath) {
	std::ifstream gin(GPScsvPath);
	std::string line;
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
		if(((status=="2"||status=="0")&&fconv_xy<20)){
			/*	if((status=="2"||status=="1")&&fconv_xy<3.0){	*/  //如果是rtk模式就加入candidate,之后可以加入其他的约束,例如cov 协方差等
			pcl::PointXYZI temp;
			temp.x = std::atof(x.c_str());
			temp.y = std::atof(y.c_str());
			temp.z = std::atof(z.c_str());
			temp.intensity = std::atof(time.c_str());
			gps.push_back(temp);
		}
	}
}

void loopClosure::LiDAR2pcd(std::string LiDARcsvPath) {
	std::ifstream fin(LiDARcsvPath); //打开文件流操作
	std::string line;
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
		LiDAR_index_time.push_back(Eigen::Vector2d(index_lidar, std::atof(time.c_str())));
	}
}

std::vector<Eigen::Vector2d> loopClosure::LiDAR_GPS_math() {
	std::vector<Eigen::Vector2d> result;
	//设置初值 没match为-1
	for (int j = 0; j < gps.size(); ++j) {
		result.push_back(Eigen::Vector2d(-1,j));
	}
	for (int i = 0; i < LiDAR_index_time.size(); ++i) {
		for (int j = 0; j < gps.size(); ++j) {
			float time_diff = LiDAR_index_time[i](1) - 0.1 - gps[j].intensity;
			if(fabs(time_diff)<=0.05){
				//match的为LiDAR index
				result[j] = Eigen::Vector2d(LiDAR_index_time[i](0),j);
			}
		}
	}
	
	return result;
}

void loopClosure::findLoopFromGPS() {
	//1.计算运动里程
	pcl::PointCloud<pcl::PointXYZI>::Ptr gps_distance (new pcl::PointCloud<pcl::PointXYZI>);
	*gps_distance = gps;
	float total_distance = 0;
	gps_distance->points[0].intensity = 0;
	//intensity 放置的是运动距离
	for (int i = 1; i < gps.size(); ++i) {
		float dx = gps[i].x - gps[i-1].x;
		float dy = gps[i].y - gps[i-1].y;
		float dz = gps[i].z - gps[i-1].z;
		total_distance += sqrt(dx*dx+dy*dy+dz*dz);
		gps_distance->points[i].intensity = total_distance;
	}
	//2. 找到最近点 先找500 个最近点
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	std::vector<int> pointIdxNKNSearch(50000);
	std::vector<float> pointNKNSquaredDistance(50000);
	kdtree.setInputCloud (gps_distance);
	//3. 遍历 所有的 gps位置 找到最近40m的
	double last_distance = 0; //上次的运动距离,用来保证密度够小
	float search_radius = 60;
	float distance_threshold = 5; //闭环点间隔距离 1米产生一个闭环
	float moving_distance = 35; //防止前后几个pcd闭环 运动距离大于这个认为闭环
	for (int j = 0; j < gps_distance->size(); ++j) {
		if(fabs(last_distance-gps_distance->points[j].intensity) > distance_threshold){//距离上次闭环点距离大于2
			if ( kdtree.radiusSearch (gps_distance->points[j], search_radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				//对于一个gps点来说
				int near_index = -1;
				float near_distance = search_radius;
				//遍历所有距离小于 search_radius 的 candidate
				for (int i = 0; i < pointIdxNKNSearch.size(); ++i) {
					//运动距离大于100m
					if(fabs(gps_distance->points[j].intensity - gps_distance->points[pointIdxNKNSearch[i]].intensity)>moving_distance){
						//找到最近的闭环点
						if(near_distance > pointNKNSquaredDistance[i]){
							near_distance = pointNKNSquaredDistance[i];
							near_index = pointIdxNKNSearch[i];
						}
					}
				}
				//产生了闭环点
				if(near_index != -1){
					GPS_Loop_from_to.push_back(Eigen::Vector2d(j,near_index));
				}
			}
			last_distance = gps_distance->points[j].intensity;
		}
	}
	//4.可视化

	for (int k = 0; k < GPS_Loop_from_to.size(); ++k) {
		pcl::PointXYZI temp;
		temp = gps_distance->points[GPS_Loop_from_to[k](0)];
		temp.intensity = k;
		select.push_back(temp);
		temp = gps_distance->points[GPS_Loop_from_to[k](1)];
		temp.intensity = k;
		select.push_back(temp);
	}
	pcl::PCDWriter writer;
	writer.write("select_loop_gps.pcd",select);
	std::cout<<"1. gps loop closure candidate: "<<select.size()<<std::endl;
}

void loopClosure::findLoopFromOdom(){
	//1.计算运动里程
	pcl::PointCloud<pcl::PointXYZI>::Ptr gps_distance (new pcl::PointCloud<pcl::PointXYZI>);
	*gps_distance = optimized_lidar_odom_pcd; // 1. 计算odom运动距离
	float total_distance = 0;
	gps_distance->points[0].intensity = 0;
	//intensity 放置的是运动距离
	for (int i = 1; i < optimized_lidar_odom_pcd.size(); ++i) {
		float dx = optimized_lidar_odom_pcd[i].x - optimized_lidar_odom_pcd[i-1].x;
		float dy = optimized_lidar_odom_pcd[i].y - optimized_lidar_odom_pcd[i-1].y;
		float dz = optimized_lidar_odom_pcd[i].z - optimized_lidar_odom_pcd[i-1].z;
		if(sqrt(dx*dx+dy*dy+dz*dz) > 0.005){//滤波防止 不动也积累位移
			total_distance += sqrt(dx*dx+dy*dy+dz*dz);
		}
		gps_distance->points[i].intensity = total_distance;
	}
	//2. 找到最近点 先找500 个最近点
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

	kdtree.setInputCloud (gps_distance);
	//3. 遍历 所有的 gps位置 找到最近40m的
	double last_distance = 0; //上次的运动距离,用来保证密度够小
	float search_radius = 40;//直径?
	float distance_threshold = 15; //闭环点间隔距离 1米产生一个闭环
	float moving_distance = search_radius*1.1; //1.1倍半径内认为不产生闭环
	for (int j = 0; j < gps_distance->size(); ++j) {
		std::vector<int> pointIdxNKNSearch(50000);
		std::vector<float> pointNKNSquaredDistance(50000);
		if(fabs(last_distance-gps_distance->points[j].intensity) > distance_threshold){//距离上次闭环点距离大于2
			if ( kdtree.radiusSearch (gps_distance->points[j], search_radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				//对于一个gps点来说
				std::vector<int> near_index;
				int nearest_index = -1;
				float near_distance = search_radius*search_radius;
				//1. 遍历所有距离小于 search_radius 的 candidate
				for (int i = 0; i < pointIdxNKNSearch.size(); ++i) {
					//运动距离大于moving_distancem
					if(fabs(gps_distance->points[j].intensity - gps_distance->points[pointIdxNKNSearch[i]].intensity) > moving_distance){
						//找到最近的闭环点 //todo 距离大于一定找一个范围
						if(near_distance > pointNKNSquaredDistance[i]){
							near_distance = pointNKNSquaredDistance[i];
							nearest_index = pointIdxNKNSearch[i];
						}
					}
				}
				if(nearest_index != -1){
					near_index.push_back(nearest_index);//所有点中最近的点
				}
		
				//2. candidate2 比上次最近的运动距离远 776candidate没有就成338了
				nearest_index = -1;
				near_distance = search_radius*search_radius;
				for (int i = 0; i < pointIdxNKNSearch.size(); ++i) {
					//
					if((fabs(gps_distance->points[nearest_index].intensity - gps_distance->points[pointIdxNKNSearch[i]].intensity)> moving_distance)&&
					(fabs(gps_distance->points[j].intensity - gps_distance->points[pointIdxNKNSearch[i]].intensity) > moving_distance)){
						if(near_distance > pointNKNSquaredDistance[i]){
							near_distance = pointNKNSquaredDistance[i];
							nearest_index = pointIdxNKNSearch[i];
						}
					}
				}
				if(nearest_index != -1){
					near_index.push_back(nearest_index);//所有点中第二近的
				}
				//产生了闭环点 更新上次距离
				if(near_index.size() != 0){
					//放入闭环点的index
					for (int i = 0; i < near_index.size(); ++i) {
						Odom_Loop_from_to.push_back(Eigen::Vector2d(j,near_index[i]));
					}
					last_distance = gps_distance->points[j].intensity;
				}
			}
		}
	}
	//4.可视化
	for (int k = 0; k < Odom_Loop_from_to.size(); ++k) {
		pcl::PointXYZI temp;
		temp = gps_distance->points[Odom_Loop_from_to[k](0)];
		temp.intensity = k;
		select.push_back(temp);
		temp = gps_distance->points[Odom_Loop_from_to[k](1)];
		temp.intensity = k;
		select.push_back(temp);
	}
	pcl::PCDWriter writer;
	writer.write("select_loop_gps.pcd",select);
	writer.write("gps_distance.pcd",*gps_distance);
	std::cout<<"0. gps loop closure candidate: "<<select.size()<<std::endl;
}
//todo gps 给定初值 然后 normal icp
//1. local map 精度不高
//2. 需要通过gps给定初值
void loopClosure::GPS_align_TF_calc() {
	std::string pcd_name1,pcd_name2,file_path1,file_path2;
	GetFileNames(filepath,"pcd");//file_names_
	pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap_past_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap_cur_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//
	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");
	ros::Publisher test_frame;
	ros::Publisher test_frame1;
	ros::Publisher test_frame2;
	ros::Publisher gps_pub;
	ros::Publisher aligened_source;
	ros::Publisher raw_target;
	ros::Publisher raw_source;
	ros::Publisher final_source;
	ros::Publisher select_gps;
	pcl::PCLPointCloud2 pcl_frame;
	test_frame = node.advertise<sensor_msgs::PointCloud2>("/local_map", 5);
	test_frame1 = node.advertise<sensor_msgs::PointCloud2>("/local_map_1", 5);
	test_frame2 = node.advertise<sensor_msgs::PointCloud2>("/local_map_origin", 5);
	gps_pub = node.advertise<sensor_msgs::PointCloud2>("/gps", 5);
	aligened_source = node.advertise<sensor_msgs::PointCloud2>("/aligened_source", 5);
	raw_target = node.advertise<sensor_msgs::PointCloud2>("/raw_target", 5);
	final_source = node.advertise<sensor_msgs::PointCloud2>("/final_source", 5);
	raw_source = node.advertise<sensor_msgs::PointCloud2>("/raw_source", 5);
	raw_source = node.advertise<sensor_msgs::PointCloud2>("/raw_source", 5);
	select_gps = node.advertise<sensor_msgs::PointCloud2>("/gps_candidate", 5);
	Eigen::Isometry3d Tg_source,Tg_tar,Tg_Gsource,Tg_Gtarget;
	sensor_msgs::PointCloud2 to_pub_frame_linear;
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_cur_g(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_cur_g_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_tar_g(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_O_tf(new pcl::PointCloud<pcl::PointXYZ>);
	//遍历闭环的GPS点
	for (int i = 0; i < GPS_Loop_from_to.size(); ++i) {
		past   = FromGPSgetLiDARindex(GPS_Loop_from_to[i](0));
		cur_id = FromGPSgetLiDARindex(GPS_Loop_from_to[i](1));
		//0.得到gps的位姿; 通过gps 算出三个 值 T_Gs_Ls T_Gs_Gt T_Gt_Lt
		Tg_Gsource = GPSPose(GPS_Loop_from_to[i](0));
		Tg_Gtarget = GPSPose(GPS_Loop_from_to[i](1));
		//0.1 得到local odom 到 lla变换
		//todo 3-DOF icp
		Tg_source = GPSLiDARExtrinsicParam(GPS_Loop_from_to[i](0));
		Tg_tar    = GPSLiDARExtrinsicParam(GPS_Loop_from_to[i](1));
		//1.在这里找到 序列
		FindFileseq(past);
		pcd_name1 = filename;
		FindFileseq(cur_id);
		pcd_name2 = filename;
		//2.获取LiDAR pcd路径
		file_path1=filepath+pcd_name1;
		file_path2=filepath+pcd_name2;
		//3. 读取local map past 现在是local系下面的
		genVLPlocalmap(50,trans_vector,past,filepath,*LocalMap_past_ptr);
		//4. 读取local map cur
		genVLPlocalmap(50,trans_vector,cur_id,filepath,*LocalMap_cur_ptr);
		//5. 转到gps坐标系
		Eigen::Isometry3d init_pose = trans_vector[cur_id].inverse()*Tg_tar*Tg_source.inverse()*trans_vector[past];
		pcl::transformPointCloud(*LocalMap_past_ptr, *P_cur_g, init_pose.matrix());
		pcl::transformPointCloud(*LocalMap_cur_ptr, *P_tar_g, Eigen::Isometry3d::Identity().matrix());
		//6.对local map 进行配准
		//6.2 初始位姿 目前用的是认为两针之间odom够准
		pcl::PointCloud<pcl::PointXYZ>::Ptr Final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::Isometry3d gicp_result;
		//6.3 进行icp配准
		util t;
	/*	t.timeCalcSet("icp: ");
		*Final_cloud = NormalICP(*P_cur_g,*P_tar_g,Eigen::Isometry3d::Identity(),gicp_result);
		t.timeUsed();*/
		t.timeCalcSet("omp: ");
		*Final_cloud = NDT_OMP(*P_cur_g,*P_tar_g,gicp_result);
		t.timeUsed();
		//7. 发布tf过的点云
		//0.gps的
		pcl::PointCloud<pcl::PointXYZI> gps_T;
		pcl::transformPointCloud(gps, gps_T, trans_vector[cur_id].inverse()*Tg_tar.matrix());
		pcl::toPCLPointCloud2(gps_T, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		gps_pub.publish(to_pub_frame_linear);
		//1. Tar系下的tar
		pcl::toPCLPointCloud2(*P_tar_g, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		test_frame1.publish(to_pub_frame_linear);//local_map_1
		//2. Tar系下的cur
		pcl::toPCLPointCloud2(*P_cur_g, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		test_frame.publish(to_pub_frame_linear);//local_map
		//G系下的result
		pcl::toPCLPointCloud2(*Final_cloud, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		aligened_source.publish(to_pub_frame_linear);
		//odom坐标系下的 target
		pcl::toPCLPointCloud2(*LocalMap_cur_ptr, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		raw_target.publish(to_pub_frame_linear);
		//LiDAR begin 下的 aligned
		pcl::transformPointCloud(*LocalMap_past_ptr, *P_O_tf, (Tg_tar*gicp_result*Tg_source.inverse()).matrix());
		pcl::toPCLPointCloud2(*P_O_tf, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		final_source.publish(to_pub_frame_linear);
		//LIDAR0 下的 raw_source
		pcl::toPCLPointCloud2(*LocalMap_past_ptr, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		raw_source.publish(to_pub_frame_linear);
		//gps candidate
		pcl::transformPointCloud(select, gps_T, trans_vector[cur_id].inverse()*Tg_tar.matrix());
		pcl::toPCLPointCloud2(gps_T, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		select_gps.publish(to_pub_frame_linear);
	 	//7. 配准完成,恢复闭环边
	 	if(ndt_score<0.3){
			curICP = gicp_result*init_pose;
			SaveTrans(curICP);
			std::cout<<"1.add a edge!!!!!!"<<"  curr align score: "<<ndt_score<<std::endl;
			geometry_msgs::PoseStamped temp;
	/*		temp.pose.position.x = curICP.translation()(0,0);
			temp.pose.position.y = curICP.translation()(0,1);
			temp.pose.position.z = curICP.translation()(0,2);
			loopClosurePath.poses.push_back(temp);*/
	 	} else{
			std::cout<<"2. not add a edge!!!!!!"<<"  curr align score: "<<ndt_score<<std::endl;
	 	}
	}
}
int loopClosure::FromGPSgetLiDARindex(int gpsIndex){
	for (int i = 0; i < LiDARmatchIndex.size(); ++i) {
		if(LiDARmatchIndex[i](1) == gpsIndex){
			return LiDARmatchIndex[i](0);
		}
	}
	std::cerr<<"can not find LiDAR index !!!"<<std::endl;
	return -1;
}
void loopClosure::genVLPlocalmap(int length, std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector,
					int num1,std::string filepath,pcl::PointCloud<pcl::PointXYZ>& bigmap){
	int upper_bound = trans_vector.size();   //防止for 越界
	int lower_bound = 1;					//防止越界
	int range = length; 					//左右取多少帧
	VLPPointCloud::Ptr cloud_add(new VLPPointCloud);
	VLPPointCloud::Ptr cloud_aft(new VLPPointCloud);
	VLPPointCloud::Ptr cloud_bef(new VLPPointCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_map(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_map_filter(new pcl::PointCloud<pcl::PointXYZ>);
	//1.从core pcd 开始往过去取pcd
	if(length == 0){
		pcl::io::loadPCDFile<VLPPoint>(file_names_[num1],*cloud_bef);
		//pcl::transformPointCloud(*cloud_bef, *cloud_aft, trans_vector[num1].matrix());
		pcl::copyPointCloud(*cloud_bef,bigmap); //转化为xyz格式
	}else{
		for(int i = num1; i>=lower_bound && num1-i < range; i--){
			std::vector<int> indices1;
			//读点云
			pcl::io::loadPCDFile<VLPPoint>(file_names_[i],*cloud_bef);
			//加个去畸变
			Eigen::Isometry3d transform;
			transform = trans_vector[i].matrix().inverse() * trans_vector[i+1].matrix();
			VLPPointCloud pointOut;
			simpleDistortion(cloud_bef,Eigen::Isometry3d (transform.matrix().inverse()),pointOut);
			pcl::transformPointCloud(pointOut, *cloud_aft, trans_vector[i].matrix());
			*cloud_add += *cloud_aft;
		}
		//2.从 core pcd 开始 往未来取pcd
		for(int i = num1 + 1; i<upper_bound && i-num1<range ;i++){
			std::vector<int> indices1;
			//读点云
			pcl::io::loadPCDFile<VLPPoint>(file_names_[i],*cloud_bef);
			//加个去畸变
			Eigen::Isometry3d transform;
			transform = trans_vector[i].inverse().matrix() * trans_vector[i+1].matrix();
			VLPPointCloud pointOut;
			simpleDistortion(cloud_bef,transform.inverse(),pointOut);
			pcl::transformPointCloud(pointOut, *cloud_aft, trans_vector[i].matrix());
			*cloud_add += *cloud_aft;
		}
		
		//转换回(0,0,0,0,0,0)
		pcl::transformPointCloud(*cloud_add, *cloud_aft, trans_vector[num1].inverse().matrix());
		//全点云时候应当加
		pcl::copyPointCloud(*cloud_aft,*xyz_map); //转化为xyz格式
/*		pcl::UniformSampling<pcl::PointXYZ> uniFilter;
		uniFilter.setRadiusSearch(0.2f); //0.1米
		pcl::PointCloud<int> keypointIndices;
		uniFilter.setInputCloud(xyz_map);
		uniFilter.compute(keypointIndices);
		pcl::copyPointCloud(*cloud_add, keypointIndices.points, *xyz_map_filter);*/
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(xyz_map);
		sor.setLeafSize(0.1,0.1,0.1);
		sor.filter(*xyz_map_filter);

//		pcl::copyPointCloud(*cloud_aft,   *xyz_map_filter);
		//mls
		// Create a KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		// Output has the PointNormal type in order to store the normals calculated by MLS
		pcl::PointCloud<pcl::PointNormal> mls_points;
		// Init object (second point type is for the normals, even if unused)
/*		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
		mls.setComputeNormals (true);
		// Set parameters
		mls.setInputCloud (xyz_map_filter);
		mls.setPolynomialOrder (2);
		mls.setSearchMethod (tree);
		mls.setSearchRadius (0.1);
		// Reconstruct
		mls.process (mls_points);
		pcl::copyPointCloud(mls_points,bigmap); //转化为xyz格式*/

//		pcl::VoxelGrid<pcl::PointXYZ> sor;
//		sor.setInputCloud(xyz_map);
//		sor.setLeafSize(0.05,0.05,0.05);
//		sor.filter(*xyz_map_filter);
/*		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(xyz_map_filter);
		outrem.setRadiusSearch(0.3);
		outrem.setMinNeighborsInRadius (5);
		outrem.filter (*xyz_map);*/
		bigmap = *xyz_map_filter;
	}
	
 
}

void
loopClosure::simpleDistortion(VLPPointCloud::Ptr pointIn, Eigen::Isometry3d transform, VLPPointCloud &pointOut){
	pointOut.clear();
	
	std::vector<pcl::PointCloud<VLPPoint> > laserCloudScans_N(16);
	Eigen::Matrix4d transInput,out,input;
	Eigen::Isometry3d transpc;
	input = Eigen::Matrix4d::Identity();//起始为0
	transInput.matrix() = transform.matrix();//终止为1
	VLPPointCloud::Ptr temp(new VLPPointCloud);
	VLPPointCloud::Ptr Individual(new VLPPointCloud);
	VLPPointCloud::Ptr Individual_bef(new VLPPointCloud);
	temp->resize(pointIn->size());
	for(int i = 0; i < pointIn->size(); i++){
		Individual->resize(1);
		Individual_bef->resize(1);
		Individual_bef->points[0] = pointIn->points[i];
		Individual->points[0] = pointIn->points[i];
		trinterp(input, transInput,pointIn->points[i].time* 10,out);
		transpc.matrix() = out.matrix();
		Eigen::Matrix4d convert;
		convert = transpc.matrix();
		convert = transpc.matrix().inverse();
		
		pcl::transformPointCloud(*Individual_bef, *Individual, convert);
		temp->points[i] = pointIn->points[i];
		temp->points[i].x = Individual->points[0].x;
		temp->points[i].y = Individual->points[0].y;
		temp->points[i].z = Individual->points[0].z;
		if(pointIn->points[i].ring >= 0 && pointIn->points[i].ring <= 16){
			laserCloudScans_N[pointIn->points[i].ring].push_back(temp->points[i]);
		}
	}
	
	for (int i = 0; i < 16; i++) {
		pointOut += laserCloudScans_N[i];
	}
}

void loopClosure::trinterp(Eigen::Matrix4d &T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T) {
	Eigen::Vector4d q0, q1, qr;
	Eigen::Vector3d p0, p1, pr;
	Eigen::Matrix3d R, R0;
	//R 为插值之后的旋转矩阵
	
	rotMat2quaternion(T0, q0);  // 位姿矩阵转换为四元数
	rotMat2quaternion(T1, q1);
	
	p0 << T0(0, 3), T0(1, 3), T0(2, 3);       // 提取出位置矩阵
	p1 << T1(0, 3), T1(1, 3), T1(2, 3);
	
	qinterp(q0, q1, r, qr);      // 进行四元数插值 10.4
	pr = p0*(1 - r) + r*p1;      // 进行位置插值
	
	quatern2rotMat(qr, R);       // 四元数转旋转矩阵
	
	T(0, 0) = R(0, 0); T(0, 1) = R(0, 1); T(0, 2) = R(0, 2); T(0, 3) = pr(0);
	T(1, 0) = R(1, 0); T(1, 1) = R(1, 1); T(1, 2) = R(1, 2); T(1, 3) = pr(1);
	T(2, 0) = R(2, 0); T(2, 1) = R(2, 1); T(2, 2) = R(2, 2); T(2, 3) = pr(2);
	T(3, 0) = 0;       T(3, 1) = 0;       T(3, 2) = 0;       T(3, 3) = 1;
}
void loopClosure::rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion) {
	
	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	double w, x, y, z;
	
	R(0, 0) = T(0, 0);
	R(0, 1) = T(0, 1);
	R(0, 2) = T(0, 2);
	R(1, 0) = T(1, 0);
	R(1, 1) = T(1, 1);
	R(1, 2) = T(1, 2);
	R(2, 0) = T(2, 0);
	R(2, 1) = T(2, 1);
	R(2, 2) = T(2, 2);
	
	double trace = R(0, 0) + R(1, 1) + R(2, 2);
	double epsilon = 1E-12;
	if (trace > epsilon)
	{
		double s = 0.5 / sqrt(trace + 1.0);
		w = 0.25 / s;
		x = -(R(2, 1) - R(1, 2)) * s;
		y = -(R(0, 2) - R(2, 0)) * s;
		z = -(R(1, 0) - R(0, 1)) * s;
		
	}
	else
	{
		if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
		{
			double s = 2.0 * sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
			w = -(R(2, 1) - R(1, 2)) / s;
			x = 0.25 * s;
			y = (R(0, 1) + R(1, 0)) / s;
			z = (R(0, 2) + R(2, 0)) / s;
			
		}
		else if (R(1, 1) > R(2, 2))
		{
			double s = 2.0 * sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
			w = -(R(0, 2) - R(2, 0)) / s;
			x = (R(0, 1) + R(1, 0)) / s;
			y = 0.25 * s;
			z = (R(1, 2) + R(2, 1)) / s;
		}
		else
		{
			double s = 2.0 * sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
			w = -(R(1, 0) - R(0, 1)) / s;
			x = (R(0, 2) + R(2, 0)) / s;
			y = (R(1, 2) + R(2, 1)) / s;
			z = 0.25 * s;
			
		}
		
	}
	q_quaternion(0) = w;
	q_quaternion(1) = x;
	q_quaternion(2) = y;
	q_quaternion(3) = z;
}
void loopClosure::qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r,
								Eigen::Vector4d &q_quaternion_interpolation) {
	
	double k0, k1, theta, sin_theta, cos_theta;
	Eigen::Vector4d q_temp;
	if ((r<0) || (r>1)){
//		std::cout << " R out of rang : " <<r<< std::endl;
	}
	
	
	cos_theta = Q1(0)*Q2(0) + Q1(1)*Q2(1) + Q1(2)*Q2(2) + Q1(3)*Q2(3);
	
	if (cos_theta < 0) {
		Q2 = -Q2;
		cos_theta = -cos_theta;
	}
	
	if ((cos_theta) > 0.9999999999) {
		k0 = 1.00 - r;
		k1 = r;
		
	}
	else {
		sin_theta = sqrt(1.00 - pow(cos_theta, 2));
		theta = atan2(sin_theta, cos_theta);
		k0 = sin((1.000 - r)*theta) / sin(theta);
		k1 = sin((r)*theta) / sin(theta);
	}
	
	q_quaternion_interpolation = k0* Q1 + k1 * Q2;
	Eigen::Quaterniond out;
	out.x() = q_quaternion_interpolation(0);
	out.y() = q_quaternion_interpolation(1);
	out.z() = q_quaternion_interpolation(2);
	out.w() = q_quaternion_interpolation(3);
	out.normalize();
	q_quaternion_interpolation(0) = out.x();
	q_quaternion_interpolation(1) = out.y();
	q_quaternion_interpolation(2) = out.z();
	q_quaternion_interpolation(3) = out.w();
}
void loopClosure::quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R) {
	Eigen::Quaterniond q;
	q.x() = q_quaternion(0);
	q.y() = q_quaternion(1);
	q.z() = q_quaternion(2);
	q.w() = q_quaternion(3);
//	R=Eigen::Matrix3d(q);
	R(0, 0) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(1), 2);
	R(0, 1) = 2.0000 * (q_quaternion(1)*q_quaternion(2) + q_quaternion(0)*q_quaternion(3));
	R(0, 2) = 2.0000 * (q_quaternion(1)*q_quaternion(3) - q_quaternion(0)*q_quaternion(2));
	R(1, 0) = 2.0000* (q_quaternion(1)*q_quaternion(2) - q_quaternion(0)*q_quaternion(3));
	R(1, 1) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(2), 2);
	R(1, 2) = 2.0000* (q_quaternion(2)*q_quaternion(3) + q_quaternion(0)*q_quaternion(1));
	R(2, 0) = 2.0000 * (q_quaternion(1)*q_quaternion(3) + q_quaternion(0)*q_quaternion(2));
	R(2, 1) = 2.0000 * (q_quaternion(2)*q_quaternion(3) - q_quaternion(0)*q_quaternion(1));
	R(2, 2) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(3), 2);
	
}
Eigen::Isometry3d loopClosure::GPSPose(int index){
	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
	pcl::PointXYZI cur_gps;
	pcl::PointXYZI next_gps;
	bool find_next =false;
	
	int next_length = 1;
	for (int i = index; i < gps.size(); ++i) {
		if (gps.size()>index + next_length){
			cur_gps = gps[index];
			next_gps = gps[index + next_length];
			float distace = sqrt((cur_gps.x-next_gps.x)*(cur_gps.x-next_gps.x) +
					(cur_gps.y-next_gps.y)*(cur_gps.y-next_gps.y) );
			if(distace>1){
				find_next = true;
				break;
			} else{
				next_length++;
			}
		}
	}
	double p1yaw=0;
	double p1x=1;
	double p1y=1;
	

	if (find_next){
		float roll,pitch,yaw,delta_x,delta_y,delta_z;
		delta_x = next_gps.x -cur_gps.x;
		delta_y = next_gps.y -cur_gps.y;
		delta_z = next_gps.z -cur_gps.z;
		yaw = atan2(delta_y,delta_x);
/*		cout<<"next: "<<next_gps.x<<" "<< next_gps.y<<endl;
		cout<<"cur: "<<cur_gps.x<<" "<< cur_gps.y<<endl;
		cout<<"gps index: "<<index<<" yaw: "<<yaw<<" degree: "<<yaw*180/M_PI<<" x: "<<delta_x<<" y: "<<delta_y<<endl;*/
		Eigen::AngleAxisd rotzp1(yaw*180/M_PI, Eigen::Vector3d::UnitZ());
		Eigen::Quaterniond q1=Eigen::Quaterniond(rotzp1);
/*		cout<<"eular angle in world axis"<<(180/M_PI)*q1.matrix().eulerAngles(0,1,2)<<endl;*/
		Eigen::Vector3d  t1= Eigen::Vector3d(next_gps.x,next_gps.y, next_gps.z);
		result = Eigen::Isometry3d(q1);
		result.pretranslate(t1);
	}
	return result;
}

Eigen::Isometry3d loopClosure::GPSLiDARExtrinsicParam(int GPSindex) {
	int LiDARIndex;
	Calibration6DOF calibrate;
	//下面的两个需要一一对应
	std::vector<Eigen::Matrix4d> gps_poses; //放gps的位置IDE
	std::vector<Eigen::Matrix4d> LiDAR_poses;//放lidar的位置的
	Eigen::Isometry3d  T_lidar2INS;
	Eigen::Vector3d arm;
	arm(2) = 0.3;//杆臂值
	//1. 挑选candidate 计算外参
	//(1) 上下边界(2)距离约束
	int range = 20;//range in meter
	int z_offset = 10;//z轴的不确定度
	//debug
	pcl::PointCloud<pcl::PointXYZI> g;
	pcl::PointCloud<pcl::PointXYZI> l;
	//往前搜索
	for (int i = GPSindex-1; i > 0; i--) {
		//两个条件 距离 足够的 gps点
		if(sqrt((gps[GPSindex].x-gps[i].x)*(gps[GPSindex].x-gps[i].x) +
		(gps[GPSindex].y-gps[i].y)+(gps[GPSindex].y-gps[i].y))>range&&LiDAR_poses.size()>40){
			break;
		} else{
			Eigen::Matrix4d temp;
			temp.setIdentity();
			temp(0,3) = gps[i].x;
			temp(1,3) = gps[i].y;
			temp(2,3) = gps[i].z;
			gps_poses.push_back(temp);
			LiDARIndex = FromGPSgetLiDARindex(i);
			LiDAR_poses.push_back(trans_vector[LiDARIndex].matrix());
			//加入z的约束
			temp(2,3) = gps[i].z+z_offset;
			gps_poses.push_back(temp);
			Eigen::Isometry3d add_z;
			add_z = trans_vector[LiDARIndex].matrix();
			add_z(2,3) = add_z(2,3)+z_offset;
			LiDAR_poses.push_back(add_z.matrix());
			//g.push_back(gps[i]);
		}
	}
	//往后搜索
	for (int i = GPSindex+1; i < gps.size(); i++) {
		if(sqrt((gps[GPSindex].x-gps[i].x)*(gps[GPSindex].x-gps[i].x) +
				(gps[GPSindex].y-gps[i].y)+(gps[GPSindex].y-gps[i].y))>range&&LiDAR_poses.size()>80){
			break;
		} else{
			Eigen::Matrix4d temp;
			temp.setIdentity();
			temp(0,3) = gps[i].x;
			temp(1,3) = gps[i].y;
			temp(2,3) = gps[i].z;
			gps_poses.push_back(temp);
			LiDARIndex = FromGPSgetLiDARindex(i);
			LiDAR_poses.push_back(trans_vector[LiDARIndex].matrix());
			//加入z的约束
			temp(2,3) = gps[i].z+z_offset;
			gps_poses.push_back(temp);
			Eigen::Isometry3d add_z;
			add_z = trans_vector[LiDARIndex].matrix();
			add_z(2,3) = add_z(2,3)+z_offset;
			LiDAR_poses.push_back(add_z.matrix());
		}
	}
	std::cout<<"number to calculate extrinstic param select in 50M : "<<LiDAR_poses.size()<<std::endl;
	pcl::PCDWriter writer;
/*	writer.write("g.pcd",g);
	writer.write("l.pcd",l);*/
	//2. 进行外参的计算
	calibrate.CalibrateGNSSLiDARICP(gps_poses,LiDAR_poses,T_lidar2INS, arm);//用icp算法确定外参
/*	std::cout<<T_lidar2INS.matrix()<<std::endl;*/
	return T_lidar2INS;
}

void loopClosure::GPSLoopClosureCalc(std::string g2o_file_path) {
	ifstream fin(g2o_file_path);
	// 设定g2o
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
	typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
	
	auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>
	        (g2o::make_unique<LinearSolverType>()));
	g2o::SparseOptimizer optimizer;            // 图模型
	optimizer.setAlgorithm(solver);            // 设置求解器
	optimizer.setVerbose(true);         // 打开调试输出
	int vertexCnt = 0, edgeCnt = 0;            // 顶点和边的数量
	while (!fin.eof()) {
		std::string name;
		fin >> name;
		if (name == "VERTEX_SE3:QUAT") {
			// SE3 顶点
			g2o::VertexSE3 *v = new g2o::VertexSE3();
			int index = 0;
			fin >> index;
			v->setId(index);
			v->read(fin);
			optimizer.addVertex(v);
			vertexCnt++;
			if (index == 0)
				v->setFixed(true);
		} else if (name == "EDGE_SE3:QUAT") {
			// SE3-SE3 边
			g2o::EdgeSE3 *e = new g2o::EdgeSE3();
			int idx1, idx2;     // 关联的两个顶点
			fin >> idx1 >> idx2;
			e->setId(edgeCnt++);
			e->setVertex(0, optimizer.vertices()[idx1]);
			e->setVertex(1, optimizer.vertices()[idx2]);
			e->read(fin);
			optimizer.addEdge(e);
		}
		if (!fin.good()) break;
	}
	
	cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;
	cout << "optimizing ..." << endl;
	optimizer.computeInitialGuess();
	optimizer.initializeOptimization();
	optimizer.optimize(30);
	
	std::vector<double> tmp1;
	for (int j = 0; j < optimizer.vertices().size(); ++j) {
		optimizer.vertex(j)->getEstimateData(tmp1);
		Eigen::Matrix4d temp1 = Eigen::Matrix4d::Identity();
		Eigen::Isometry3d temp2 = Eigen::Isometry3d::Identity();
		Eigen::Quaterniond temp3 = Eigen::Quaterniond::Identity();
		temp3.x() = tmp1[3];
		temp3.y() = tmp1[4];
		temp3.z() = tmp1[5];
		temp3.w() = tmp1[6];
		temp2.pretranslate(Eigen::Vector3d (tmp1[0],tmp1[1],tmp1[2]));
		temp2.rotate(temp3);
		optimized_lidar_odom.push_back(temp2);
		pcl::PointXYZI onePoint;
		onePoint.x = tmp1[0];
		onePoint.y = tmp1[1];
		onePoint.z = tmp1[2];
		onePoint.intensity = j;
		optimized_lidar_odom_pcd.push_back(onePoint);
	}
	pcl::PCDWriter writer;
 	writer.write("optimized_lidar_odom_pcd.pcd",optimized_lidar_odom_pcd);
 	std::cout<<"lidar odom optimized , saved as: optimized_lidar_odom_pcd.pcd"<<std::endl;
}

void loopClosure::Odom_align_TF_calc() {
	std::string pcd_name1,pcd_name2,file_path1,file_path2;
	GetFileNames(filepath,"pcd");//file_names_
	pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap_past_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap_cur_ptr(new pcl::PointCloud<pcl::PointXYZ>);
 
	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");
	ros::Publisher test_frame;
	ros::Publisher test_frame1;
	ros::Publisher test_frame2;
	ros::Publisher gps_pub;
	ros::Publisher aligened_source;
	ros::Publisher raw_target;
	ros::Publisher raw_source;
	ros::Publisher final_source;
	ros::Publisher select_gps;
	pcl::PCLPointCloud2 pcl_frame;
	test_frame = node.advertise<sensor_msgs::PointCloud2>("/local_map", 5);
	test_frame1 = node.advertise<sensor_msgs::PointCloud2>("/local_map_1", 5);
	test_frame2 = node.advertise<sensor_msgs::PointCloud2>("/local_map_origin", 5);
	gps_pub = node.advertise<sensor_msgs::PointCloud2>("/gps", 5);
	aligened_source = node.advertise<sensor_msgs::PointCloud2>("/aligened_source", 5);
	raw_target = node.advertise<sensor_msgs::PointCloud2>("/raw_target", 5);
	final_source = node.advertise<sensor_msgs::PointCloud2>("/final_source", 5);
	raw_source = node.advertise<sensor_msgs::PointCloud2>("/raw_source", 5);
	raw_source = node.advertise<sensor_msgs::PointCloud2>("/raw_source", 5);
	select_gps = node.advertise<sensor_msgs::PointCloud2>("/gps_candidate", 5);
 
	sensor_msgs::PointCloud2 to_pub_frame_linear;
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_cur_g(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_cur_g_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_tar_g(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_O_tf(new pcl::PointCloud<pcl::PointXYZ>);
	if(file_names_.size() !=optimized_lidar_odom.size()){
		std::cerr<<"文件数目和位姿数目不同!!!!"<<std::endl;
	}
	//遍历闭环的GPS点
	for (int i = 0; i < Odom_Loop_from_to.size(); ++i) {
		past   = Odom_Loop_from_to[i](0);
		cur_id = Odom_Loop_from_to[i](1);
		std::cout<<" loop candidate: "<<past<<" "<<cur_id<<std::endl;
		std::cout<<" file_names_ size(): "<<file_names_.size()<<std::endl;
		std::cout<<" optimized_lidar_odom size(): "<<optimized_lidar_odom.size()<<std::endl;
		//1.在这里找到 序列
		FindFileseq(past);
		pcd_name1 = filename;
		FindFileseq(cur_id);
		pcd_name2 = filename;
		//2.获取LiDAR pcd路径
		file_path1=filepath+pcd_name1;
		file_path2=filepath+pcd_name2;
		//3. 读取local map past 现在是local系下面的
		genVLPlocalmap(100,optimized_lidar_odom,past,filepath,*LocalMap_past_ptr);
		//4. 读取local map cur
		genVLPlocalmap(100,optimized_lidar_odom,cur_id,filepath,*LocalMap_cur_ptr);
		//5. 转到gps坐标系
		Eigen::Isometry3d init_pose = optimized_lidar_odom[cur_id].inverse()*optimized_lidar_odom[past];
		pcl::transformPointCloud(*LocalMap_past_ptr, *P_cur_g, init_pose.matrix());
		pcl::transformPointCloud(*LocalMap_cur_ptr, *P_tar_g, Eigen::Isometry3d::Identity().matrix());
		//6.对local map 进行配准
		//6.2 初始位姿 目前用的是认为两针之间odom够准
		pcl::PointCloud<pcl::PointXYZ>::Ptr Final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::Isometry3d gicp_result;
		//6.3 进行icp配准
		util t;
		t.timeCalcSet("omp: ");
		*Final_cloud = NDT_OMP(*P_cur_g,*P_tar_g,gicp_result);
		t.timeUsed();
		//7. 发布tf过的点云
		//0.gps的
		pcl::PointCloud<pcl::PointXYZI> gps_T;
		pcl::transformPointCloud(gps, gps_T, optimized_lidar_odom[cur_id].inverse().matrix());
		pcl::toPCLPointCloud2(gps_T, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		gps_pub.publish(to_pub_frame_linear);
		//1. Tar系下的tar
		pcl::toPCLPointCloud2(*P_tar_g, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		test_frame1.publish(to_pub_frame_linear);//local_map_1
		//2. Tar系下的cur
		pcl::toPCLPointCloud2(*P_cur_g, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		test_frame.publish(to_pub_frame_linear);//local_map
		//G系下的result
		pcl::toPCLPointCloud2(*Final_cloud, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		aligened_source.publish(to_pub_frame_linear);
		//odom坐标系下的 target
		pcl::toPCLPointCloud2(*LocalMap_cur_ptr, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		raw_target.publish(to_pub_frame_linear);
		//LiDAR begin 下的 aligned
		pcl::transformPointCloud(*LocalMap_past_ptr, *P_O_tf, (gicp_result).matrix());
		pcl::toPCLPointCloud2(*P_O_tf, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		final_source.publish(to_pub_frame_linear);
		//LIDAR0 下的 raw_source
		pcl::toPCLPointCloud2(*LocalMap_past_ptr, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		raw_source.publish(to_pub_frame_linear);
		//gps candidate
		pcl::transformPointCloud(select, gps_T, optimized_lidar_odom[cur_id].inverse().matrix());
		pcl::toPCLPointCloud2(gps_T, pcl_frame);
		pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
		to_pub_frame_linear.header.frame_id = "/map";
		select_gps.publish(to_pub_frame_linear);
		//7. 配准完成,恢复闭环边
		if(ndt_score<13){
			curICP = gicp_result*init_pose;
			SaveTrans(curICP);
			std::cout<<"1.add a edge!!!!!!"<<"  curr align score: "<<ndt_score<<std::endl;
			geometry_msgs::PoseStamped temp;
		} else{
			std::cout<<"2. not add a edge!!!!!!"<<"  curr align score: "<<ndt_score<<std::endl;
		}
	}
}
