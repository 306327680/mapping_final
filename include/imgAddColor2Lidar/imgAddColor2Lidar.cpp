//
// Created by echo on 2020/4/3.
//

#include "imgAddColor2Lidar.h"

void imgAddColor2Lidar::setInternal(Eigen::Matrix3d intrinsics1, double k11, double k21, double k31, double p11, double p21) {
	intrinsics = intrinsics1;
	k1 = k11;
	k2 = k21;
	k3 = k31;
	p1 = p11;
	p2 = p21;
}

void imgAddColor2Lidar::readExInt(std::string path) {
	std::string param_path = path;
	std::ifstream params(param_path);
	std::cout << param_path << std::endl;
	params >> publish_option;
	params >> intrinsics(0,0);
	params >> intrinsics(1,1);
	params >> intrinsics(0,2);
	params >> intrinsics(1,2);
	params >> k1;
	params >> k2;
	params >> k3;
	params >> p1;
	params >> p2;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			params >> rotation(i,j);
		}
	}
	params >> translation[0];
	params >> translation[1];
	params >> translation[2];
	std::cout << publish_option << std::endl;
	std::cout << intrinsics << std::endl;
	std::cout << rotation << std::endl;
	std::cout << k1 << std::endl;
	std::cout << k2 << std::endl;
	std::cout << k3 << std::endl;
	std::cout << p1 << std::endl;
	std::cout << p2 << std::endl;
	
}

PPointXYZIRGB imgAddColor2Lidar::alignImg2LiDAR(cv::Mat mat, VLPPointCloud cloudin) {
	PPointXYZIRGB coloured_point_cloud;
	cv::Mat projected_image1;
	for (int i = 0; i < cloudin.size(); ++i) {
		PointXYZIRGB temp_point;
		temp_point.x = cloudin[i].x;
		temp_point.y = cloudin[i].y;
		temp_point.z = cloudin[i].z;
		temp_point.intensity = cloudin[i].intensity;
		
		point_3d << cloudin[i].x, cloudin[i].y, cloudin[i].z;
		point_3d_3 = rotation * point_3d + translation;//外参平移现在的点
		
		point_t3d = point_3d_3 / point_3d_3[2];//z轴归一化
		
		float r_2 = point_t3d[0] * point_t3d[0] + point_t3d[1] * point_t3d[1];
		float x_ori = point_t3d[0];
		float y_ori = point_t3d[1];
		//这里用内参啥的 投影公式
		point_t3d[0] = x_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p1*x_ori*y_ori + p2*(r_2 + 2*pow(x_ori,2));
		point_t3d[1] = y_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p2*x_ori*y_ori + p1*(r_2 + 2*pow(y_ori,2));
		
		int image_u = (int)(intrinsics(0,0)*point_t3d[0] + intrinsics(0,2));
		int image_v = (int)(intrinsics(1,1)*point_t3d[1] + intrinsics(1,2));
		
		if (1 <= image_u && image_u < mat.size().width-1 &&
			1 <= image_v && image_v < mat.size().height-1 && mat.at<cv::Vec3b>(image_u,image_v)[0] >= 0)
		{
			cv::Vec3b colour= mat.at<cv::Vec3b>(cv::Point(image_u, image_v));
			temp_point.r = colour[0];
			temp_point.g = colour[1];
			temp_point.b = colour[2];
			coloured_point_cloud.push_back(temp_point);
/*			projected_image1(cv::Rect(image_u, image_v, 2, 2)).setTo(255);*/
		}
	}
	return coloured_point_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB> imgAddColor2Lidar::pclalignImg2LiDAR(cv::Mat mat, VLPPointCloud cloudin) {
	pcl::PointCloud<pcl::PointXYZRGB> coloured_point_cloud;
	cv::Mat projected_image1;
	for (int i = 0; i < cloudin.size(); ++i) {
		pcl::PointXYZRGB temp_point;
		temp_point.x = cloudin[i].x;
		temp_point.y = cloudin[i].y;
		temp_point.z = cloudin[i].z;
		
		point_3d << cloudin[i].x, cloudin[i].y, cloudin[i].z;
		point_3d_3 = rotation * point_3d + translation;//外参平移现在的点
		
		point_t3d = point_3d_3 / point_3d_3[2];//z轴归一化
		
		float r_2 = point_t3d[0] * point_t3d[0] + point_t3d[1] * point_t3d[1];
		float x_ori = point_t3d[0];
		float y_ori = point_t3d[1];
		//这里用内参啥的 投影公式
		point_t3d[0] = x_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p1*x_ori*y_ori + p2*(r_2 + 2*pow(x_ori,2));
		point_t3d[1] = y_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p2*x_ori*y_ori + p1*(r_2 + 2*pow(y_ori,2));
		
		int image_u = (int)(intrinsics(0,0)*point_t3d[0] + intrinsics(0,2));
		int image_v = (int)(intrinsics(1,1)*point_t3d[1] + intrinsics(1,2));
		
		if (1 <= image_u && image_u < mat.size().width-1 &&
			1 <= image_v && image_v < mat.size().height-1 &&
			mat.at<cv::Vec3b>(image_u,image_v)[0] >= 0 && temp_point.x>0){
			
			cv::Vec3b colour= mat.at<cv::Vec3b>(cv::Point(image_u, image_v));
			temp_point.r = colour[2];
			temp_point.g = colour[1];
			temp_point.b = colour[0];
			coloured_point_cloud.push_back(temp_point);
/*			projected_image1(cv::Rect(image_u, image_v, 2, 2)).setTo(255);*/
		}
	}
	return coloured_point_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>
imgAddColor2Lidar::pclalignImg2LiDAR(cv::Mat mat, pcl::PointCloud<pcl::PointXYZI> cloudin) {
	pcl::PointCloud<pcl::PointXYZRGB> coloured_point_cloud;
	cv::Mat projected_image1;
	for (int i = 0; i < cloudin.size(); ++i) {
		pcl::PointXYZRGB temp_point;
		temp_point.x = cloudin[i].x;
		temp_point.y = cloudin[i].y;
		temp_point.z = cloudin[i].z;
		
		point_3d << cloudin[i].x, cloudin[i].y, cloudin[i].z;
		point_3d_3 = rotation * point_3d + translation;//外参平移现在的点
		
		point_t3d = point_3d_3 / point_3d_3[2];//z轴归一化
		
		float r_2 = point_t3d[0] * point_t3d[0] + point_t3d[1] * point_t3d[1];
		float x_ori = point_t3d[0];
		float y_ori = point_t3d[1];
		//这里用内参啥的 投影公式
		point_t3d[0] = x_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p1*x_ori*y_ori + p2*(r_2 + 2*pow(x_ori,2));
		point_t3d[1] = y_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p2*x_ori*y_ori + p1*(r_2 + 2*pow(y_ori,2));
		
		int image_u = (int)(intrinsics(0,0)*point_t3d[0] + intrinsics(0,2));
		int image_v = (int)(intrinsics(1,1)*point_t3d[1] + intrinsics(1,2));
		
		if (1 <= image_u && image_u < mat.size().width-1 &&
			1 <= image_v && image_v < mat.size().height-1 &&
			mat.at<cv::Vec3b>(image_u,image_v)[0] >= 0 && temp_point.x>0){
			
			cv::Vec3b colour= mat.at<cv::Vec3b>(cv::Point(image_u, image_v));
			temp_point.r = colour[2];
			temp_point.g = colour[1];
			temp_point.b = colour[0];
			coloured_point_cloud.push_back(temp_point);
/*			projected_image1(cv::Rect(image_u, image_v, 2, 2)).setTo(255);*/
		}
	}
	return coloured_point_cloud;
}
