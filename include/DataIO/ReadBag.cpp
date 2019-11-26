//
// Created by echo on 2019/11/26.
//

#include "ReadBag.h"

void ReadBag::getPath(std::string path) {
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	//可以加挺多topic的?
	topics.push_back(std::string(endocder_));
/*	topics.push_back(std::string("numbers"));*/
 
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
		if (s != NULL)
			int  a=0;
			//std::cout << s->pose.pose.position.x << std::endl;
	/*	第二个topic的提取方式
	 * std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
		if (i != NULL)
        	std::cout << i->data << std::endl;*/

	}

bag.close();
}
