//
// Created by echo on 2020/4/11.
//

#ifndef PCD_COMPARE_CSVIO_H
#define PCD_COMPARE_CSVIO_H

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
class CSVio {
public:
	CSVio(){};
	//1.gps 存为csv格式
	void NavSat2CSV(std::vector<sensor_msgs::NavSatFix> gnss_pos, std::string save_path,ros::Time start_time);
	

};


#endif //PCD_COMPARE_CSVIO_H
