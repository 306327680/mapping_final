//
// Created by echo on 2020/4/11.
//

#include "CSVio.h"
//1.简单的储存一下
void CSVio::NavSat2CSV(std::vector<sensor_msgs::NavSatFix> gnss_pos, std::string save_path, ros::Time start_time) {
	std::ofstream outFile;
	outFile.open("test.csv", std::ios::out);
	outFile<<"altitude,latitude,longitude,conv0,conv1,conv2,conv3,conv4,conv5,conv6,conv7,conv8,status,service,seq,stamp\n";
	for (int i = 0; i < gnss_pos.size(); ++i) {
		outFile << gnss_pos[i].longitude << ',';
		outFile << gnss_pos[i].latitude << ',';
		outFile << std::setprecision(10)<< gnss_pos[i].altitude << ',';
		outFile << gnss_pos[i].position_covariance[0] << ',';
		outFile << gnss_pos[i].position_covariance[1] << ',';
		outFile << gnss_pos[i].position_covariance[2] << ',';
		outFile << gnss_pos[i].position_covariance[3] << ',';
		outFile << gnss_pos[i].position_covariance[4] << ',';
		outFile << gnss_pos[i].position_covariance[5] << ',';
		outFile << gnss_pos[i].position_covariance[6] << ',';
		outFile << gnss_pos[i].position_covariance[7] << ',';
		outFile << gnss_pos[i].position_covariance[8] << ',';
		outFile << (int)gnss_pos[i].status.status << ',';
		outFile << gnss_pos[i].status.service << ',';
		outFile << gnss_pos[i].header.seq << ',';
		outFile << (gnss_pos[i].header.stamp-start_time).toSec() << '\n';
	}
}
//2.转换到lla坐标下
void CSVio::NavSat2CSVLLA(std::vector<sensor_msgs::NavSatFix> gnss_pos, std::string save_path, ros::Time start_time,sensor_msgs::NavSatFix LLA) {
	std::ofstream outFile;
	outFile.open("gps.csv", std::ios::out);
	outFile<< std::setprecision(10)<<"x,y,z,conv0,conv1,conv2,conv3,conv4,conv5,conv6,conv7,conv8,status,service,seq,stamp,"
	<<LLA.longitude<<","<<LLA.latitude<<","<<LLA.altitude<<"\n";
	gt.lla_origin_ = gt.GpsMsg2Eigen(LLA);//设定起点
	
	for (int i = 0; i < gnss_pos.size(); ++i) {
		gt.updateGPSpose(gnss_pos[i]);
		outFile << gt.gps_pos_(0) << ',';
		outFile << gt.gps_pos_(1) << ',';
		outFile << gt.gps_pos_(2) << ',';
		outFile << gnss_pos[i].position_covariance[0] << ',';
		outFile << gnss_pos[i].position_covariance[1] << ',';
		outFile << gnss_pos[i].position_covariance[2] << ',';
		outFile << gnss_pos[i].position_covariance[3] << ',';
		outFile << gnss_pos[i].position_covariance[4] << ',';
		outFile << gnss_pos[i].position_covariance[5] << ',';
		outFile << gnss_pos[i].position_covariance[6] << ',';
		outFile << gnss_pos[i].position_covariance[7] << ',';
		outFile << gnss_pos[i].position_covariance[8] << ',';
		outFile << (int)gnss_pos[i].status.status << ',';
		outFile << gnss_pos[i].status.service << ',';
		outFile << gnss_pos[i].header.seq << ',';
		outFile << (gnss_pos[i].header.stamp-start_time).toSec() << '\n';
	}
}

void CSVio::Lidar2CSV(std::vector<nav_msgs::Odometry> odoms, std::string path, ros::Time start_time) {
	std::ofstream outFile;
	outFile.open("LiDAR_pose.csv", std::ios::out);
	outFile<< std::setprecision(10)<<"x,y,z,q_x,q_y,q_z,q_w,v_x,v_y,v_z,r_x,r_y,r_z,"<<"\n";
	for (int i = 0; i < odoms.size(); ++i) {
		outFile << odoms[i].pose.pose.position.x << ',';
		outFile << odoms[i].pose.pose.position.y << ',';
		outFile << odoms[i].pose.pose.position.z << ',';
		outFile << odoms[i].pose.pose.orientation.x << ',';
		outFile << odoms[i].pose.pose.orientation.y << ',';
		outFile << odoms[i].pose.pose.orientation.z << ',';
		outFile << odoms[i].pose.pose.orientation.w << ',';
		outFile << odoms[i].twist.twist.linear.x << ',';
		outFile << odoms[i].twist.twist.linear.y << ',';
		outFile << odoms[i].twist.twist.linear.z << ',';
		outFile << odoms[i].twist.twist.angular.x << ',';
		outFile << odoms[i].twist.twist.angular.y << ',';
		outFile << odoms[i].twist.twist.angular.z << ',';
		outFile << odoms[i].header.seq << ',';
		outFile << (odoms[i].header.stamp-start_time).toSec() << '\n';
	}
}

void CSVio::LiDARsaveOnePose(Eigen::Isometry3d pose, ros::Time cur_time) {
	trans_vector.push_back(pose);
	time_seq.push_back(cur_time);
}

void CSVio::LiDARsaveAll(std::string path) {
	std::vector<nav_msgs::Odometry> odoms;
	//转换格式
	for (int i = 0; i < trans_vector.size(); ++i) {
		nav_msgs::Odometry odom_temp;
		tf::Transform tf_test;
		tf_test.setIdentity();
		tf::transformEigenToTF(trans_vector[i], tf_test);
		tf::poseTFToMsg(tf_test, odom_temp.pose.pose);
		odoms.push_back(odom_temp);
	}
	
	std::ofstream outFile;
	outFile.open("LiDAR_pose.csv", std::ios::out);
	outFile<< std::setprecision(10)<<"x,y,z,q_x,q_y,q_z,q_w,v_x,v_y,v_z,r_x,r_y,r_z,time,"<<time_seq[0]<<"\n";
	for (int i = 0; i < odoms.size(); ++i) {
		outFile << odoms[i].pose.pose.position.x << ',';
		outFile << odoms[i].pose.pose.position.y << ',';
		outFile << odoms[i].pose.pose.position.z << ',';
		outFile << odoms[i].pose.pose.orientation.x << ',';
		outFile << odoms[i].pose.pose.orientation.y << ',';
		outFile << odoms[i].pose.pose.orientation.z << ',';
		outFile << odoms[i].pose.pose.orientation.w << ',';
		outFile << odoms[i].twist.twist.linear.x << ',';
		outFile << odoms[i].twist.twist.linear.y << ',';
		outFile << odoms[i].twist.twist.linear.z << ',';
		outFile << odoms[i].twist.twist.angular.x << ',';
		outFile << odoms[i].twist.twist.angular.y << ',';
		outFile << odoms[i].twist.twist.angular.z << ',';
		outFile << (time_seq[i]-time_seq[0]).toSec() << '\n';
	}
}
