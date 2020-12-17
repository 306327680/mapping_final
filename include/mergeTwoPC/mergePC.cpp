//
// Created by echo on 2020/9/16.
//

#include "mergePC.h"

void mergePC::readYaml(std::string path) {
	FILE *fh = fopen(path.c_str(), "r");
	if (fh == NULL)
	{
		std::cout << "config_file dosen't exist; wrong config_file path" << std::endl;
 
		return;
	}
	fclose(fh);
	
	cv::FileStorage fsSettings(path, cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		std::cerr << "ERROR: Wrong path to settings" << std::endl;
	}
	
	size_t NUM_OF_LASER = 2;
	TBL.resize(NUM_OF_LASER);
	cv::Mat cv_T;
	fsSettings["body_T_laser"] >> cv_T;
	for (int i = 0; i < NUM_OF_LASER; i++)
	{
		Eigen::Quaterniond q = Eigen::Quaterniond(cv_T.ptr<double>(i)[3], cv_T.ptr<double>(i)[0], cv_T.ptr<double>(i)[1], cv_T.ptr<double>(i)[2]);
		Eigen::Vector3d t = Eigen::Vector3d(cv_T.ptr<double>(i)[4], cv_T.ptr<double>(i)[5], cv_T.ptr<double>(i)[6]);
		TBL[i].setIdentity();
		TBL[i].topLeftCorner<3, 3>() = q.toRotationMatrix();
		TBL[i].topRightCorner<3, 1>() = t;
		std::cout << q.coeffs().transpose() << ", " << t.transpose() << std::endl;
		std::cout << TBL[i] << std::endl;
	}
}

VLPPointCloud mergePC::process(VLPPointCloud left, VLPPointCloud right) {
	VLPPointCloud result;
	pcl::transformPointCloud(left, left, TBL[0].cast<float>());
	pcl::transformPointCloud(right, right, TBL[1].cast<float>());
	result += left;
	result += right;
	return result;
}
