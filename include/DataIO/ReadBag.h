//
// Created by echo on 2019/11/26.
//

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
#include <boost/foreach.hpp>
#include <geometry_msgs/QuaternionStamped.h>

class ReadBag {
public:
	ReadBag(){};
	void getPath(std::string path);

private:
	rosbag::Bag bag;
	nav_msgs::Odometry encoder_odom_;
	sensor_msgs::NavSatFix gps_pos_;
	geometry_msgs::QuaternionStamped gps_head_;
	std::string gps_msg_pos_ = "/novatel718d/pos";
	std::string gps_msg_head_ = "/novatel718d/heading";
	std::string endocder_ = "/golfcar/odom";
	std::string endocder_raw_ = "/golfcar/odom_raw";//不一定对
	std::string imu_ = "rion";//想不起来了
	std::string camera1_ = "cam1_";
	std::string camera2_ = "cam2_";
	std::string camera3_ = "cam3_";
};


#endif //PCD_COMPARE_READBAG_H
