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
	extrinstic_param.setIdentity();
	extrinstic_param.rotate(rotation);
	extrinstic_param.translate(translation);
}

pcl::PointCloud<PointXYZRGBI>  imgAddColor2Lidar::alignImg2LiDAR(cv::Mat mat, pcl::PointCloud<pcl::PointXYZI> cloudin) {
	pcl::PointCloud<PointXYZRGBI>  coloured_point_cloud;
	cv::Mat projected_image1;
	for (int i = 0; i < cloudin.size(); ++i) {
	 	PointXYZRGBI	  temp_point;
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
		//这里用去畸变
		point_t3d[0] = x_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p1*x_ori*y_ori + p2*(r_2 + 2*pow(x_ori,2));
		point_t3d[1] = y_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p2*x_ori*y_ori + p1*(r_2 + 2*pow(y_ori,2));
		//内参投影
		int image_u = (int)(intrinsics(0,0)*point_t3d[0] + intrinsics(0,2));
		int image_v = (int)(intrinsics(1,1)*point_t3d[1] + intrinsics(1,2));
		
		if (1 <= image_u && image_u < mat.size().width-1 &&
			1 <= image_v && image_v < mat.size().height-1 && mat.at<cv::Vec3b>(image_u,image_v)[0] >= 0)
		{
			cv::Vec3b colour= mat.at<cv::Vec3b>(cv::Point(image_u, image_v));
			temp_point.r = colour[2];
			temp_point.g = colour[1];
			temp_point.b = colour[0];
			temp_point.a = 1;
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
		//这里用内参啥的 投影公式 公式5.1
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
imgAddColor2Lidar::pclalignImg2LiDAR(cv::Mat mat, pcl::PointCloud<pcl::PointXYZI> cloudin,cv::Mat &depth){
	//去畸变
	cv::Mat image = mat;
	cv::Mat frame;
	cv::Mat imageCalibration;
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = intrinsics(0, 0);
	cameraMatrix.at<double>(0, 1) = intrinsics(0, 1);
	cameraMatrix.at<double>(0, 2) = intrinsics(0, 2);
	cameraMatrix.at<double>(1, 1) = intrinsics(1, 1);
	cameraMatrix.at<double>(1, 2) = intrinsics(1, 2);

	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0, 0) = k1;
	distCoeffs.at<double>(1, 0) = k2;
	distCoeffs.at<double>(2, 0) = k3;
	distCoeffs.at<double>(3, 0) = p1;
	distCoeffs.at<double>(4, 0) = p2;

	cv::Mat view, rview, map1, map2;
	cv::Size imageSize;
	imageSize = image.size();
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
							getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
							imageSize, CV_16SC2, map1, map2);

	remap(image, imageCalibration, map1, map2, cv::INTER_LINEAR);
	
 	mat = imageCalibration;
	depth.size() = mat.size();
	//正经程序
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
			/*if(colour[0]<250||colour[1]<250||colour[2]<250){*/
			coloured_point_cloud.push_back(temp_point);
		/*	}*/
	
/*			projected_image1(cv::Rect(image_u, image_v, 2, 2)).setTo(255);*/
		}
	}
	return coloured_point_cloud;
}

cv::Mat imgAddColor2Lidar::pcd2img(cv::Mat mat, pcl::PointCloud<pcl::PointXYZI> cloudin,cv::Mat &depth) {
	pcl::PointCloud<PointXYZRGBI>  coloured_point_cloud;
/*	cv::Mat image = mat;
	cv::Mat frame;
	cv::Mat imageCalibration;
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = intrinsics(0, 0);
	cameraMatrix.at<double>(0, 1) = intrinsics(0, 1);
	cameraMatrix.at<double>(0, 2) = intrinsics(0, 2);
	cameraMatrix.at<double>(1, 1) = intrinsics(1, 1);
	cameraMatrix.at<double>(1, 2) = intrinsics(1, 2);
	
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0, 0) = k1;
	distCoeffs.at<double>(1, 0) = k2;
	distCoeffs.at<double>(2, 0) = k3;
	distCoeffs.at<double>(3, 0) = p1;
	distCoeffs.at<double>(4, 0) = p2;
	
	cv::Mat view, rview, map1, map2;
	cv::Size imageSize;
	imageSize = image.size();
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
							getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
							imageSize, CV_16SC2, map1, map2);
	
	remap(image, imageCalibration, map1, map2, cv::INTER_LINEAR);
	mat = imageCalibration;*/
	//之前的
	cv::Mat projected_image1 =  mat;
	depth.size() = mat.size();
	cv::Mat depth_image = cv::Mat::zeros(mat.size(), CV_16UC1);
	std::cout<<depth_image.size()<<std::endl;
	for (int i = 0; i < cloudin.size(); ++i) {
		PointXYZRGBI	  temp_point;
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

		float depth_float;
		depth_float = sqrtf(temp_point.x*temp_point.x+temp_point.y*temp_point.y+temp_point.z*temp_point.z)*1000;
		//这里用去畸变 这里公式问题??
		point_t3d[0] = x_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p1*x_ori*y_ori + p2*(r_2 + 2*pow(x_ori,2));
		point_t3d[1] = y_ori*(1 + k1*r_2 + k2*pow(r_2, 2) + k3*pow(r_2, 3)) + 2*p2*x_ori*y_ori + p1*(r_2 + 2*pow(y_ori,2));
		//内参投影
		int image_u = (int)(intrinsics(0,0)*point_t3d[0] + intrinsics(0,2));
		int image_v = (int)(intrinsics(1,1)*point_t3d[1] + intrinsics(1,2));
		
		if (1 <= image_u && image_u < mat.size().width-1 &&
			1 <= image_v && image_v < mat.size().height-1 && mat.at<cv::Vec3b>(image_u,image_v)[0] >= 0)
		{
			cv::Vec3b colour= mat.at<cv::Vec3b>(cv::Point(image_u, image_v));
			temp_point.r = colour[2];
			temp_point.g = colour[1];
			temp_point.b = colour[0];
			temp_point.a = 1;
			coloured_point_cloud.push_back(temp_point);
			int intensity=0;
			if(temp_point.intensity > 35){
				intensity = 255;
			} else{
				intensity = int((temp_point.intensity/35)*255);
			}
			if(temp_point.x>0){
				projected_image1(cv::Rect(image_u, image_v, 1, 1)).setTo(
						cv::Vec3b( abs(255 - intensity), abs(127 - intensity), abs(0 - intensity)));
 		/*		if(depth_image.at<float>(image_u,image_v) != 0){
 					if (depth_image.at<float>(image_u,image_v) > depth_float){
						depth_image(cv::Rect(image_u, image_v, 2, 2)) = depth_float;
 					}
 				} else{
					depth_image(cv::Rect(image_u, image_v, 2, 2)) = depth_float;
 				}*/
				depth_image(cv::Rect(image_u, image_v, 2, 2)) = depth_float;
			}
		
/*			projected_image1.at<cv::Vec3b>(image_u,image_v)[0] = abs(255 - intensity); //blue
			projected_image1.at<cv::Vec3b>(image_u,image_v)[1] = abs(127 - intensity); //green
			projected_image1.at<cv::Vec3b>(image_u,image_v)[2] = abs(0   - intensity); //red*/
		}
	}
	depth = depth_image;
	return projected_image1;
}
