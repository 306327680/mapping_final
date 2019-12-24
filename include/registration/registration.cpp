//
// Created by echo on 2019/11/26.
//

#include "registration.h"

void registration::setParam(std::string configFileName) {
	std::ifstream ifs(configFileName.c_str());

	if (ifs.good())
	{
		icp.loadFromYaml(ifs);
	}
	else
	{
		std::cout<<"Cannot load ICP config from YAML file " << configFileName<<std::endl;
		icp.setDefault();
	}
}

void registration::setMap(pcl::PointCloud<pcl::PointXYZI> pcin) {

	Eigen::MatrixXf testCloudP(4,pcin.size());
	for (int i = 0; i < pcin.size(); ++i) {
		
		testCloudP(0,i) = pcin[i].x;
		testCloudP(1,i) = pcin[i].y;
		testCloudP(2,i) = pcin[i].z;
		testCloudP(3,i) = 1.f;
	}
	DP::Labels labels;
	labels.push_back(DP::Label("X",1));
	labels.push_back(DP::Label("Y",1));
	labels.push_back(DP::Label("Z",1));
	DP localMap(testCloudP,labels);

	icp.setMap(localMap);

}

PM::TransformationParameters  registration::setScan(pcl::PointCloud<pcl::PointXYZI> pcin) {
	PM::TransformationParameters icp_result,T_scanner_to_localMap;
	Eigen::MatrixXf testCloudP(4,pcin.size());
	for (int i = 0; i < pcin.size(); ++i) {
		
		testCloudP(0,i) = pcin[i].x;
		testCloudP(1,i) = pcin[i].y;
		testCloudP(2,i) = pcin[i].z;
		testCloudP(3,i) = 1.f;
	}
	DP::Labels labels;
	labels.push_back(DP::Label("X",1));
	labels.push_back(DP::Label("Y",1));
	labels.push_back(DP::Label("Z",1));
	DP newPointCloud(testCloudP,labels);
	
	const int dimp1(newPointCloud.features.rows());
	T_scanner_to_localMap = PM::TransformationParameters::Identity(dimp1, dimp1);
	icp_result = icp(newPointCloud, T_scanner_to_localMap);
	return icp_result;
}
//给一个点云添加normal
void registration::addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
							 pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud,*cloud_source_normals);
	
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZ>);
	searchTree->setInputCloud(cloud_source_normals);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud_source_normals);
	normalEstimator.setSearchMethod(searchTree);
	//normalEstimator.setRadiusSearch(0.05);
	normalEstimator.setKSearch(40);
	normalEstimator.compute(*normals);
	pcl::concatenateFields(*cloud_source_normals, *normals, *cloud_with_normals);
}

void registration::SetNormalICP() {
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(
			new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	icp->setMaximumIterations(100);
	icp->setMaxCorrespondenceDistance(0.02);
	icp->setTransformationEpsilon(1e-10);
	icp->setEuclideanFitnessEpsilon(1e-10);
	this->pcl_plane_plane_icp = icp;
	
}

pcl::PointCloud<pcl::PointXYZI> registration::normalIcpRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
																	pcl::PointCloud<pcl::PointXYZI>  target) {
	//todo 改成xyz
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals_temp(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZI> tfed;
	//隔断一下
	pcl::PointCloud<pcl::PointXYZI>::Ptr target1(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(target,*target1);
	addNormal(source, cloud_source_normals);
	addNormal(target1, cloud_target_normals);
	*cloud_source_normals_temp = *cloud_source_normals;
	//0. 上次位姿态*增量
	icp_init = transformation * increase;
	//1.转换点云 给一个初值
	pcl::transformPointCloud(*cloud_source_normals_temp, *cloud_source_normals, icp_init.matrix());
	pcl_plane_plane_icp->setInputSource(cloud_source_normals);
	pcl_plane_plane_icp->setInputTarget(cloud_target_normals);
	pcl_plane_plane_icp->align(*cloud_source_normals);
	//2.当前的transform 全局准确
 	transformation = icp_init * pcl_plane_plane_icp->getFinalTransformation();//上次结果(结果加预测)
	//计算不带increase的increase
	increase = increase * pcl_plane_plane_icp->getFinalTransformation();
	std::cout << "icp \n" << std::endl << pcl_plane_plane_icp->getFinalTransformation() << std::endl;
	std::cout << "increase \n" << std::endl << increase << std::endl;
	std::cout << "global \n" << std::endl << transformation << std::endl;
	
	pcl::transformPointCloud(*source, tfed, transformation.matrix());
	
	//变化量

	return tfed;
}
