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
	std::cout<<"PCD file is "<<filename<<" seq is"<<idx_file<<std::endl;
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
}

void loopClosure::SaveTrans(Eigen::Isometry3d curr) {
	Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
	std::vector<VertexSE3*> vertices;
	std::vector<EdgeSE3*> edges;
	VertexSE3 *cur = new VertexSE3;
	VertexSE3 *prev = new VertexSE3;;
	cur->setId(past);
	Eigen::Isometry3d t_v = Eigen::Isometry3d::Identity();
	cur->setEstimate(t_v);
	cur->setFixed(true);
	vertices.push_back(cur);
	prev->setId(cur_id);
	prev->setEstimate(curr);
	vertices.push_back(prev);
	EdgeSE3 *e = new EdgeSE3;
	e->setVertex(0, prev);
	e->setVertex(1, cur);
	e->setMeasurement(curr); //debug
	e->setInformation(information);
	edges.push_back(e);
	saveFile(save_g2o_path,vertices,edges);
	std::cout<<"\033[34m"<<"g2o saved as "<<save_g2o_path<<std::endl;
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

pcl::PointCloud<pcl::PointXYZ> loopClosure::GICP(const pcl::PointCloud<pcl::PointXYZ> &cloud_source,
												 const pcl::PointCloud<pcl::PointXYZ> &cloud_target,
												 Eigen::Isometry3d &icp_matrix) {
	pcl::PointCloud<pcl::PointXYZ> result;
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp; //创建ICP对象，用于ICP配准
	gicp.setTransformationEpsilon(0.01);
	gicp.setMaximumIterations(100);
	gicp.setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_source)));
	gicp.setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_target)));
	gicp.align(result);
	Eigen::Matrix4f tf_s2t = gicp.getFinalTransformation();
	icp_matrix = tf_s2t.cast<double>();
	std::cout << tf_s2t << std::endl;
	return result; // source上应用这个矩阵，就可以转过去了

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
	//todo 有可能有问题
	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>icp;
	//初始位姿
	Eigen::Isometry3d init_pose = trans_vector[past].inverse()*trans_vector[cur_id];
	init_pose = init_pose.inverse();
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud1, *temp, init_pose.matrix());
	*cloud1 = *temp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final_cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);//InputTarget不动的
	icp.setMaxCorrespondenceDistance(10);
	icp.setTransformationEpsilon(1e-2);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setMaximumIterations(120);
//  icp.align(Final_cloud);
	Eigen::Isometry3d gicp_result;
	*Final_cloud = GICP(*cloud1,*cloud2,gicp_result);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	final_cloud = Final_cloud;
	Eigen::Isometry3d icp_matrix;
	icp_matrix =gicp_result;
	std::cout<<icp_matrix*init_pose.inverse().matrix().inverse()
			 <<std::endl;
	//得到最终的transform
	curICP = icp_matrix*init_pose.inverse().matrix().inverse();
	genlocalmap(trans_vector,past,filepath,*cloud1);
	genlocalmap(trans_vector,cur_id,filepath,*cloud2);
	pcl::transformPointCloud(*cloud1, *final_cloud, curICP.matrix());
	SaveTrans(curICP);

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
