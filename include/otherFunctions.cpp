//
// Created by echo on 2020/4/11.
//

#include "otherFunctions.h"

int main_function::getParam(int argc, char **argv) {
	std::cout
			<< "设置建图模式: 1.g2o+pcd的传统模式(pcd+g2o路径) 2.point to plane ICP (需要提供pcd路径) 3.bpreal ground mapping (pcd+g2o路径)"
			<< "4. 使用编码器和GPS LiDAR 建图" << "5. LiDAR gps 外参标定" << "6. NDT maping" << std::endl;
	cin >> status;
	std::cout << "status: " << status << std::endl;
	if (status == 1 || status == 3) {
		if (argc != 3 && argc != 5) {
			std::cout
					<< "argument error! argument number has to be 3/5! The first one should be pcd path the second should be g2o path"
					<< std::endl;
			std::cout
					<< "./pcd_reader /media/echo/35E4DE63622AD713/fushikang/loop_pcd_single /media/echo/35E4DE63622AD713/fushikang/lihaile.g2o "
					<< std::endl;
			std::cout
					<< "/media/echo/DataDisc/1_pcd_file/pku_bin /media/echo/DataDisc/2_g2o/pku/4edges_out.g2o 500 12210"
					<< std::endl;
			return (-1);
		}
		filepath = argv[1];
		g2o_path = argv[2];
		if (argc == 5) {
			start_id = atoi(argv[3]);
			end_id = atoi(argv[4]);
		}
		std::cout << "start_id " << start_id << " end_id " << end_id;
	}
	if (status == 2 || status == 6) {
		if (argc != 2) {
			std::cout << "argument error! argument number has to be 2! The first one should be pcd path"
					  << std::endl;
			std::cout << "e.g.: \n ./pcd_reader /media/echo/35E4DE63622AD713/fushikang/loop_pcd_single"
					  << std::endl;
			std::cout << "输入pcd路径: " << std::endl;
			cin >> filepath;
			cout << filepath << endl;
		} else {
			filepath = argv[1];
		}
	}
}

ros::Time main_function::fromPath2Time(std::string s) {
	std::vector<std::string> vStr;
	ros::Time cur_time;
	boost::split( vStr, s, boost::is_any_of( "./" ), boost::token_compress_on );
	std::vector<std::string> buffer;
	for( std::vector<std::string>::iterator it = vStr.begin(); it != vStr.end(); ++ it )
		buffer.push_back(*it);
	cur_time.sec = std::atoi(buffer[buffer.size()-3].c_str());
	int nsec_check = std::atoi(buffer[buffer.size()-2].c_str());
	if(nsec_check<100000000){
		cur_time.nsec = nsec_check*10;
	}else{
		cur_time.nsec = nsec_check;
	}
	std::cout<<cur_time.sec<<" "<<cur_time.nsec<<endl;
	return cur_time;
}

bool main_function::GetFileNames(const std::string directory, const std::string suffix) {
	file_names_.clear();
	DIR *dp;
	struct dirent *dirp;
	dp = opendir(directory.c_str());
	if (!dp) {
		std::cerr << "cannot open directory:" << directory << std::endl;
		return false;
	}
	std::string file;
	while (dirp = readdir(dp)) {
		file = dirp->d_name;
		if (file.find(".") != std::string::npos) {
			file = directory + "/" + file;
			if (suffix == file.substr(file.size() - suffix.size())) {
				file_names_.push_back(file);
			}
		}
	}
	closedir(dp);
	std::sort(file_names_.begin(), file_names_.end());
	
	if (file_names_.empty()) {
		std::cerr << "directory:" << directory << "is empty" << std::endl;
		return false;
	}
	std::cerr << "路径: " << directory << " 有" << file_names_.size() << "个pcd文件" << std::endl;
	return true;
}

bool main_function::GetPNGFileNames(const std::string directory, const std::string suffix) {
	PNG_file_names_.clear();
	DIR *dp;
	struct dirent *dirp;
	dp = opendir(directory.c_str());
	if (!dp) {
		std::cerr << "cannot open directory:" << directory << std::endl;
		return false;
	}
	std::string file;
	while (dirp = readdir(dp)) {
		file = dirp->d_name;
		if (file.find(".") != std::string::npos) {
			file = directory + "/" + file;
			if (suffix == file.substr(file.size() - suffix.size())) {
				PNG_file_names_.push_back(file);
			}
		}
	}
	closedir(dp);
	std::sort(PNG_file_names_.begin(), PNG_file_names_.end());
	
	if (PNG_file_names_.empty()) {
		std::cerr << "directory:" << directory << "is empty" << std::endl;
		return false;
	}
	std::cerr << "路径: " << directory << " 有" << PNG_file_names_.size() << "个PNG文件" << std::endl;
	return true;
}

bool main_function::FindFileseq(int64_t seq)  {
	int64_t idx_file = seq;
	if (idx_file > file_names_.size() - 1) {
		return INT64_MAX;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	uint64_t idx_seperator = file_names_[idx_file].find_last_of("/");
	filename = file_names_[idx_file].substr(idx_seperator + 1);
	std::cout << "PCD file is " << filename << " seq is" << idx_file << std::endl;
	return true;
}

