//
// Created by echo on 19-3-6.
//

#include "util.h"

void util::timeUsed() {
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now() - _now_ms);
    std::cout<<name_global<<" time is :"<< duration.count()/1e9<<std::endl;

}

void util::timeCalcSet(std::string name) {
    _now_ms = std::chrono::high_resolution_clock ::now();
    name_global = name;

}

void util::timeCalcSet() {
    _now_ms = std::chrono::high_resolution_clock ::now();
}

bool util::GetFileNames(const std::string directory, const std::string suffix) {
    file_names_.clear();
    DIR *dp;
    struct dirent *dirp;
    dp = opendir(directory.c_str());
    if (!dp) {
        std::cerr<<"cannot open directory:"<<directory<<std::endl;
        return false;
    }
    std::string file;
    while(dirp = readdir(dp)){
        file = dirp->d_name;
        if (file.find(".")!=std::string::npos){
            file = directory + "/" + file;
            if (suffix == file.substr(file.size() - suffix.size())){
                file_names_.push_back(file);
            }
        }
    }
    closedir(dp);
    std::sort(file_names_.begin(),file_names_.end());
    
    if(file_names_.empty()){
        std::cerr<<"directory:"<<directory<<"is empty"<<std::endl;
        return false;
    }
    return true;
}

std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
util::getEigenPoseFromg2oFile(std::string &g2ofilename) {
    std::ifstream fin(g2ofilename);
    if (!fin) {
        std::cerr << "file " << g2ofilename << " does not exist." << std::endl;
        std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT;     //todo
        vT.resize(1);
        vT[0] = Eigen::Matrix4d::Identity(4, 4);
        return vT;
    }
    
    
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT;
    while (!fin.eof()) {
        std::string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            g2o::VertexSE3 v;
            
            int index = 0;
            fin >> index;
            v.setId(index);
            v.read(fin);
            Eigen::Isometry3d T = v.estimate();
            vT.push_back(T);
        } else if (name == "EDGE_SE3:QUAT") {
            continue;
        } else
            continue;
        if (!fin.good()) break;
    }
    std::cout << "read total " << vT.size() << " vertices\n";
    poses = vT;
    return vT;
}
//todo 回头写一个kmeans的
void util::GetPointCloudBeam(pcl::PointCloud<pcl::PointXYZI> pc_in, pcl::PointCloud<pcl::PointXYZINormal>& pc_out) {
	//RSBPEARL
	float Rx_ = 0.01697;
	float Ry_ = -0.0085;
	float Rz_ = 0.12644;
	pc_out.clear();
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef (new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_pub (new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr select_points (new pcl::PointCloud<pcl::PointXYZI>());

	*cloud_bef = pc_in;
	std::vector<int> indices1;
	std::vector<int> beam_kind;
	std::vector<int> beam_kind_unique;
	std::vector<int> temp;
	//找到线对应的角度
	//
	double verticalAngle;
	if(1){
		//找到线束分类 64线 atan2 5389 bpreal 90 arcsin()
		for (int j = 0; j < cloud_bef->size(); j = j+1) {
			
			double theta_h = -atan2(cloud_bef->points[j].y,cloud_bef->points[j].x);
			double theta_v =  (atan(((cloud_bef->points[j].z-Rz_)*cos(theta_h))/(cloud_bef->points[j].x-Rx_*cos(theta_h))));
			beam_kind.push_back(int(theta_v*180.0 / M_PI));
		}
		beam_kind_unique = unique_element_in_vector(beam_kind);
		//排序
		std::sort (beam_kind_unique.begin(), beam_kind_unique.end());
		std::cout<<"beam amount: "<<beam_kind_unique.size()<<std::endl;
		
		//第一次range大小
		for (int j = 0; j < cloud_bef->size(); ++j) {
			pcl::PointXYZI temp;
			temp.x = pc_in[j].x;
			temp.y = pc_in[j].y;
			temp.z = pc_in[j].z;
			temp.intensity = pc_in[j].intensity;

			double theta_h = -atan2(temp.y,temp.x);
			double theta_v =  (atan(((temp.z-Rz_)*cos(theta_h))/(temp.x-Rx_*cos(theta_h))));
			theta_v =  theta_v*180.0/M_PI;
			
			for (int i = 0; i < beam_kind_unique.size(); ++i) {
				if(int(theta_v) == beam_kind_unique[i]){
					pcl::PointXYZINormal temp1;
					
					temp1.x = pc_in[j].x;
					temp1.y = pc_in[j].y;
					temp1.z = pc_in[j].z;
					temp1.normal_x = i;
					temp1.intensity = temp.intensity;
					pc_out.push_back(temp1);
				}
			}
		}
	}
}
