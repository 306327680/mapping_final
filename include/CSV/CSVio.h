//
// Created by echo on 2020/4/11.
//

#ifndef PCD_COMPARE_CSVIO_H
#define PCD_COMPARE_CSVIO_H

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <fstream>
#include "GPS/gpsTools.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
class CSVio {
public:
	CSVio(){};
	//1.gps 存为csv格式
	//start_time 是这个bag最早的时间戳
	void NavSat2CSV(std::vector<sensor_msgs::NavSatFix> gnss_pos, std::string save_path,ros::Time start_time);
	//2.转换成lla格式
	void NavSat2CSVLLA(std::vector<sensor_msgs::NavSatFix> gnss_pos, std::string save_path,ros::Time start_time,sensor_msgs::NavSatFix LLA);
	//3.LiDAR pose 储存,直接以odom方式存
	void Lidar2CSV(std::vector<nav_msgs::Odometry> odoms,std::string path,ros::Time start_time); //主要存PQV 时间戳
	//4.Lidar 一个一个存
	void LiDARsaveOnePose(Eigen::Isometry3d pose,ros::Time cur_time);
	//4.1 Lidar 全存进去
	void LiDARsaveAll(std::string path);
	//5. imu2csv
	void IMU2CSV(std::vector<sensor_msgs::Imu> IMUs,std::string save_path,ros::Time start_time);
	//5.1 read imu to eigen
	void ReadImuCSV(std::string read_path,std::vector<Eigen::VectorXd> &result);
private:
	gpsTools gt;
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trans_vector;
	std::vector<ros::Time> time_seq;
};


#endif //PCD_COMPARE_CSVIO_H
