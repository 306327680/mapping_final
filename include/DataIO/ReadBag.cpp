//
// Created by echo on 2019/11/26.
//

#include "ReadBag.h"

//todo 这里的lidar的时间戳有问题,应该还是固定的时间
void ReadBag::getPath(std::string path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	
	//可以加挺多topic的?
	topics.push_back(std::string(lidarodom));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	//读广场的也就几秒
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
		if (bag_strat_time.isZero()){
			bag_strat_time = s->header.stamp;
		}
		if(fabs(s->pose.pose.position.x)<10000) {
			if (s != NULL) {
				pcl::PointXYZINormal temp;
				temp.x = s->pose.pose.position.x;
				temp.y = s->pose.pose.position.y;
				temp.z = s->pose.pose.position.z;
				temp.intensity = s->header.stamp.toSec()-bag_strat_time.toSec();
				Eigen_encoder.emplace_back(s->pose.pose.position.x, s->pose.pose.position.y, s->pose.pose.position.z);
				encoder_pcd.push_back(temp);
			}
		}
	}
	topics.clear();
	topics.push_back(std::string(gps_calibrate));
	rosbag::View view1(bag, rosbag::TopicQuery(topics));
	bag_strat_time.init();
	BOOST_FOREACH(rosbag::MessageInstance const m, view1) {
					nav_msgs::Odometry::ConstPtr gps = m.instantiate<nav_msgs::Odometry>();
					
			if (bag_strat_time.isZero()){
						bag_strat_time = gps->header.stamp;
			}
			if (gps != NULL) {
				if(fabs(gps->pose.pose.position.x)<10000){
					pcl::PointXYZINormal temp;
					temp.x = gps->pose.pose.position.x;
					temp.y = gps->pose.pose.position.y;
					temp.z = gps->pose.pose.position.z;
					temp.intensity = gps->header.stamp.toSec() - bag_strat_time.toSec();
					Eigen_GPS.emplace_back(gps->pose.pose.position.x, gps->pose.pose.position.y,gps->pose.pose.position.z);
					gps_pcd.push_back(temp);
				}
			}
	}
	//这个是一起以前的
	/*	topics.push_back(std::string(endocder_));
	topics.push_back(std::string(gps_msg_pos_));
 
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	//读广场的也就几秒
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
		if (s != NULL){
		 	Eigen_encoder.emplace_back(s->pose.pose.position.x,s->pose.pose.position.y,s->pose.pose.position.z);
		}
		sensor_msgs::NavSatFix::ConstPtr gps = m.instantiate<sensor_msgs::NavSatFix>();
		if(gps != NULL){
			gpstools.updateGPSpose(*gps);
			Eigen_GPS.emplace_back(gpstools.gps_pos_);
		}
	}
*/
bag.close();
	std::cout<<"read bag finish! "<<Eigen_GPS.size()<<" "<<Eigen_encoder.size()<<std::endl;
}
/*
 *
header:
  seq: 11902
  stamp:
    secs: 1574931531
    nsecs: 757029056
  frame_id: "cpt"
status:
  status: 0
  service: 1
latitude: 22.5336322945
longitude: 113.945080181
altitude: 8.4084
position_covariance: [0.045125999318016086, 0.09610083080179557, 0.3187239400720881, 0.09610083080179557, 0.20465739973337754, 0.6787580530127076, 0.3187239400720881, 0.6787580530127076, 2.2511401744076007]
position_covariance_type: 0
 经纬度
 * */
void ReadBag::readCPT(sensor_msgs::NavSatFix input) {

}

void ReadBag::readHesai(std::string path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	bool pcd_start = false;
	double time_begin;
 	float maxintensity=0;
	float minintensity=1000;
	float setMinIntensity = 0.0000000000000000000000000000000000000000014349296274686126806258990933;
	float setMaxIntensity = 0.0000000000000000000000000000000000000000017922607358714410337114501370;
	float diff_intensity = setMaxIntensity-setMinIntensity;
	double timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/points_raw"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	//读广场的也就几秒
	int inter_times =0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					pcdtosave.clear();
					std::stringstream pcd_save;
					pcd_save.precision(18);
					sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
					pcl::PCLPointCloud2 pcl_pc2;
			
		
					pcl::fromROSMsg(*s,hesai_pcd);
					std::cout<<"pcd size: "<<hesai_pcd.size()<<std::endl;
 					if(!pcd_start){
						time_begin = hesai_pcd[0].timestamp;
 					}
		
					for (int i = 0; i < hesai_pcd.size(); ++i) {
						//std::printf("%f\n",hesai_pcd[i].timestamp);
						mypcd temp;
						temp.x = hesai_pcd[i].x;
						temp.y = hesai_pcd[i].y;
						temp.z = hesai_pcd[i].z;
						temp.intensity = int(256*(hesai_pcd[i].intensity - setMinIntensity)/diff_intensity);
						temp.timestamp = hesai_pcd[i].timestamp - time_begin;
						temp.ring = hesai_pcd[i].ring;
						pcdtosave.push_back(temp);
						//用来找最大最小intensity的
			/*			if(maxintensity<hesai_pcd[i].intensity){
							maxintensity=hesai_pcd[i].intensity;
						}
						if(minintensity>hesai_pcd[i].intensity){
							minintensity=hesai_pcd[i].intensity;
						}*/
					}
					pcdtosave.width = pcdtosave.size();
					//用来找最大最小intensity的
/*					std::printf("maxintensity: %f minintensity: %f\n",maxintensity,minintensity);
					std::cout.precision(70);
					std::cout<<"maxintensity: "<<std::fixed<<maxintensity<<" minintensity: "<<minintensity<<std::endl;*/
					timestamp = s->header.stamp.toSec();
					pcd_save<<"map/"<<timestamp<<".pcd";
					std::cout<<pcd_save.str()<<" size: "<<pcdtosave.size()<<" times: "<<inter_times<<std::endl;
					writer.write(pcd_save.str(),pcdtosave,true);
					std::cout<<pcd_save.str()<<" saved " <<std::endl;
					inter_times++;
				}
}

void ReadBag::gnssLiDARExtrinsicParameters(std::string path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	
	//可以加挺多topic的?
	topics.push_back(std::string(lidarodom));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	Eigen::Isometry3d EP ;
	//读广场的也就几秒
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
					if (bag_strat_time.isZero()){
						bag_strat_time = s->header.stamp;
					}
					if(fabs(s->pose.pose.position.x)<10000) {
						if (s != NULL) {
							EP.setIdentity();
							pcl::PointXYZINormal temp;
							Eigen::Quaterniond q_t;
							q_t.x() = s->pose.pose.orientation.x;
							q_t.y() = s->pose.pose.orientation.y;
							q_t.z() = s->pose.pose.orientation.z;
							q_t.w() = s->pose.pose.orientation.w;
							
							EP.pretranslate(Eigen::Vector3d(s->pose.pose.position.x,s->pose.pose.position.y,s->pose.pose.position.z));
							EP.rotate(q_t);
							EP.pretranslate(Eigen::Vector3d(-0.547,0,0));
							std::cout<<EP.matrix()<<std::endl;
							temp.x = EP(0,3);
							temp.y = EP(1,3);
							temp.z = EP(2,3);
							temp.intensity = s->header.stamp.toSec()-bag_strat_time.toSec();
							Eigen_encoder.emplace_back(s->pose.pose.position.x, s->pose.pose.position.y, s->pose.pose.position.z);
							encoder_pcd.push_back(temp);
						}
					}
				}
	topics.clear();
	topics.push_back(std::string(gps_calibrate));
	rosbag::View view1(bag, rosbag::TopicQuery(topics));
	bag_strat_time.init();
	BOOST_FOREACH(rosbag::MessageInstance const m, view1) {
					nav_msgs::Odometry::ConstPtr gps = m.instantiate<nav_msgs::Odometry>();
					
					if (bag_strat_time.isZero()){
						bag_strat_time = gps->header.stamp;
					}
					if (gps != NULL) {
						if(fabs(gps->pose.pose.position.x)<10000){
							pcl::PointXYZINormal temp;
							temp.x = gps->pose.pose.position.x;
							temp.y = gps->pose.pose.position.y;
							temp.z = gps->pose.pose.position.z;
							temp.intensity = gps->header.stamp.toSec() - bag_strat_time.toSec();
							Eigen_GPS.emplace_back(gps->pose.pose.position.x, gps->pose.pose.position.y,gps->pose.pose.position.z);
							gps_pcd.push_back(temp);
						}
					}
				}
 
	bag.close();
	std::cout<<"read bag finish! "<<Eigen_GPS.size()<<" "<<Eigen_encoder.size()<<std::endl;
}
