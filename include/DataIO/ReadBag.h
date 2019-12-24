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

#include <pcl_conversions/pcl_conversions.h>
class ReadBag {
public:
	ReadBag(){bag_strat_time.init();};
	void getPath(std::string path);
	void gnssLiDARExtrinsicParameters (std::string path);
	std::vector<Eigen::Vector3d> Eigen_encoder;
	std::vector<Eigen::Vector3d> Eigen_GPS;
	gpsTools gpstools;
	pcl::PointCloud<pcl::PointXYZINormal> encoder_pcd;
	pcl::PointCloud<pcl::PointXYZINormal> gps_pcd;
	pcl::PointCloud<pcl::PointXYZINormal> cpt_pcd;
	void readCPT(sensor_msgs::NavSatFix input);
	void readHesai(std::string path);
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
	//外外参标定的两个topic
	std::string lidarodom = "/odom_mapped";
	std::string gps_calibrate = "/ins_linsin_odom";
	ros::Time bag_strat_time;
};


#endif //PCD_COMPARE_READBAG_H