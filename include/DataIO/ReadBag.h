//
// Created by echo on 2019/11/26.
//
//雷达外参:距离0.547m 斜0.618 高0.2876
#ifndef PCD_COMPARE_READBAG_H
#define PCD_COMPARE_READBAG_H


#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <Eigen/Core>
#include "GPS/gpsTools.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "tools/util.h"
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <utility>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <CSV/CSVio.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include "mergeTwoPC/mergePC.h"
#include "g2oIO/PoseGraphIO.h"
class ReadBag {
public:
	ReadBag(){bag_strat_time.init();};
	void getPath(std::string path);
	//0. jlx那个topic
	void gnssLiDARExtrinsicParameters (std::string path);
	void saveRTK2PCD (std::string path,std::string savepath);//普通gps读取
	//1. 输入pcd路径读取LLA下面的惯导坐标+时间戳 和pcd计算好的 pcd的先不加_因为现在没有数据gps和lidar对齐的
	void gnssPCDExtrinsicParameters (std::string path,std::vector<std::pair<Eigen::Isometry3d,double>> & gps_pose , Eigen::Vector3d &lla_origin);
	std::vector<Eigen::Vector3d> Eigen_encoder;
	std::vector<Eigen::Vector3d> Eigen_GPS;
	gpsTools gpstools;
	pcl::PointCloud<pcl::PointXYZINormal> encoder_pcd;
	pcl::PointCloud<pcl::PointXYZINormal> gps_pcd;
	pcl::PointCloud<pcl::PointXYZINormal> cpt_pcd;
	//2. 这里转换gps惯导等的数据
	void readCPT(sensor_msgs::NavSatFix input);
	//3. 这里转换雷达等的数据
	void readHesai(std::string path);
	void readVLP16(std::string path,std::string save_path);
	void readVLP16WoTime(std::string path,std::string save_path);
	void readTopRobosense(std::string path,std::string save_path);
	void readRobo128(std::string path,std::string save_path);
	void readPandarXT32(std::string path,std::string save_path);
	//4. 读取图片
	void readcamera(std::string path,std::string save_path);
	void readStereoCamera(std::string path,std::string save_path);
	void readCalibratedCamera(std::string path,std::string cali_path,std::string save_path);
	//5. readImu
	void readImu(std::string path,std::string save_path);
	//6. readINS and LiDAR 华测给定gga的
	void readINS(std::string path,std::string save_path);
private:
	rosbag::Bag bag;
	nav_msgs::Odometry encoder_odom_;
	sensor_msgs::NavSatFix gps_pos_;
	pcl::PCDWriter writer;
	geometry_msgs::QuaternionStamped gps_head_;
	std::string gps_msg_pos_ = "/novatel718d/pos";
	std::string gps_msg_head_ = "/novatel718d/heading";
	std::string endocder_ = "/golfcar/odom";
	std::string endocder_raw_ = "/golfcar/odom_raw";//不一定对
	std::string imu_ = "rion";//想不起来了
	std::string camera1_ = "cam1_";
	std::string camera2_ = "cam2_";
	std::string camera3_ = "cam3_";
	std::string cpt_navsat = "/cpt/ins_fix";
	pcl::PointCloud<PPoint> hesai_pcd;
	pcl::PointCloud<VLPPoint> vlp_pcd;
	pcl::PointCloud<pcl::PointXYZI> robosense_pcd;
	mypcdCloud pcdtosave;
	VLPPointCloud vlp_pcdtosave;
	RoboPointCLoud robo_pcdtosave;
	//外外参标定的两个topic
	std::string lidarodom = "/odom_mapped";
	std::string gps_calibrate = "/ins_linsin_odom";
	ros::Time bag_strat_time;
	//转换csv
	CSVio csvio;
	//LiDAR 第一帧时间
	ros::Time lidar_first;
};


#endif //PCD_COMPARE_READBAG_H
