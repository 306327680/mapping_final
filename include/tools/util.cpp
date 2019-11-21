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
void util::GetPointCloudBeam(pcl::PointCloud<pcl::PointXYZINormal> pc_in, pcl::PointCloud<pcl::PointXYZINormal>& pc_out) {
	//RSBPEARL
	float Rx_ = 0.01697;
	float Ry_ = -0.0085;
	float Rz_ = 0.12644;
	pc_out.clear();
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_bef (new pcl::PointCloud<pcl::PointXYZINormal>());

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
			if(fabs(cloud_bef->points[j].y)>1e-10&&fabs(cloud_bef->points[j].z)>1e-10&&fabs(cloud_bef->points[j].x)>1e-10){
				double theta_h = -atan2(cloud_bef->points[j].y,cloud_bef->points[j].x);
				double theta_v =  (atan(((cloud_bef->points[j].z-Rz_)*cos(theta_h))/(cloud_bef->points[j].x-Rx_*cos(theta_h))));
				beam_kind.push_back(int(theta_v*180.0 / M_PI));
			}
		}
		beam_kind_unique = unique_element_in_vector(beam_kind);
		//排序
		std::sort (beam_kind_unique.begin(), beam_kind_unique.end());
		std::cout<<"beam amount: "<<beam_kind_unique.size()<<std::endl;
		beam_pcd.clear();
		beam_pcd.resize(beam_kind_unique.size());
	/*	for (int k = 0; k < beam_kind_unique.size(); ++k) {
			std::cout<<beam_kind_unique[k]<<std::endl;
		}*/
		//第一次range大小
		for (int j = 0; j < cloud_bef->size(); ++j) {
			pcl::PointXYZI temp;
			temp.x = pc_in[j].x;
			temp.y = pc_in[j].y;
			temp.z = pc_in[j].z;
			temp.intensity = pc_in[j].intensity;

			double theta_h = -atan2(temp.y,temp.x);
			double theta_v =  (atan(((temp.z-Rz_)*cos(theta_h))/(temp.x-Rx_*cos(theta_h))));
			theta_v =  theta_v * 180.0/M_PI;
			
			for (int i = 0; i < beam_kind_unique.size(); ++i) {
				if(int(theta_v) == beam_kind_unique[i]){
					pcl::PointXYZINormal temp1;
					temp1.x = pc_in[j].x;
					temp1.y = pc_in[j].y;
					temp1.z = pc_in[j].z;
					temp1.normal_x = i;//线束
					temp1.normal_y = pc_in[j].normal_y;//时间
					temp1.intensity = pc_in[j].intensity;
					temp1.normal_z = (pc_in[j].z - Rz_)/ sin(theta_v*M_PI/180); //距离
					pc_out.push_back(temp1);
					beam_pcd[i].push_back(temp1);
				}
			}
		}
	}
}



void util::GetBeamEdge(pcl::PointCloud<pcl::PointXYZINormal> pc_in, pcl::PointCloud<pcl::PointXYZINormal>& pc_out) {
	pc_out.clear();
	float diffX,diffY,diffZ;
	std::vector<float> eigenValue;
	
/*	cv::Mat matA0 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
	cv::Mat matX0 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));
	
	for (int i = 0; i < beam_pcd.size(); ++i) {
		for (int j = 5; j < beam_pcd[i].size()-5; ++j) {
			for (int k = 0; k < 10; ++k) {
				matA0.at<float>(k, 0) = beam_pcd[i].points[j-k].x;
				matA0.at<float>(k, 1) = beam_pcd[i].points[j-k].y;
				matA0.at<float>(k, 2) = beam_pcd[i].points[j-k].z;
			}
	
			cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);
			
			eigenValue.clear();
			float pa = fabs(matX0.at<float>(0, 0));
			float pb = fabs(matX0.at<float>(1, 0));
			float pc = fabs(matX0.at<float>(2, 0));
			float pd = 1;
			
			float ps = sqrt(pa * pa + pb * pb + pc * pc);
			pa /= ps; pb /= ps; pc /= ps; pd /= ps;
			eigenValue.push_back(pa);
			eigenValue.push_back(pb);
			eigenValue.push_back(pc);
			std::sort(eigenValue.begin(),eigenValue.end());
			beam_pcd[i][j].normal_z = eigenValue[2];
			pc_out.push_back(beam_pcd[i][j]);
		}
	}*/
	
	for (int i = 0; i < beam_pcd.size(); ++i) {
/*		for (int j = 5; j < beam_pcd[i].size()-5; ++j)
			{
			
					float diffX = beam_pcd[i][j- 5].x + beam_pcd[i][j- 4].x
								  + beam_pcd[i][j- 3].x + beam_pcd[i][j- 2].x
								  + beam_pcd[i][j- 1].x - 10 * beam_pcd[i][j].x
								  + beam_pcd[i][j+ 1].x + beam_pcd[i][j+ 2].x
								  + beam_pcd[i][j+ 3].x + beam_pcd[i][j+ 4].x
								  + beam_pcd[i][j+ 5].x;
					float diffY = beam_pcd[i][j- 5].y + beam_pcd[i][j- 4].y
								  + beam_pcd[i][j- 3].y + beam_pcd[i][j- 2].y
								  + beam_pcd[i][j- 1].y - 10 * beam_pcd[i][j].y
								  + beam_pcd[i][j+ 1].y + beam_pcd[i][j+ 2].y
								  + beam_pcd[i][j+ 3].y + beam_pcd[i][j+ 4].y
								  + beam_pcd[i][j+ 5].y;
					float diffZ = beam_pcd[i][j- 5].z + beam_pcd[i][j- 4].z
								  + beam_pcd[i][j- 3].z + beam_pcd[i][j- 2].z
								  + beam_pcd[i][j- 1].z - 10 * beam_pcd[i][j].z
								  + beam_pcd[i][j+ 1].z + beam_pcd[i][j+ 2].z
								  + beam_pcd[i][j+ 3].z + beam_pcd[i][j+ 4].z
								  + beam_pcd[i][j+ 5].z;
					//曲率计算
		 
				
			beam_pcd[i][j].intensity = (diffX * diffX + diffY * diffY + diffZ * diffZ)/beam_pcd[i][j].normal_z;
			pc_out.push_back(beam_pcd[i][j]);
			
			}*/
			for (int j = 1; j < beam_pcd[i].size()-1; ++j){
				Eigen::Vector3d a,b,c,ba,bc;
				a<<beam_pcd[i][j-1].x,beam_pcd[i][j-1].y,beam_pcd[i][j-1].z;
				b<<beam_pcd[i][j].x,beam_pcd[i][j].y,beam_pcd[i][j].z;
				c<<beam_pcd[i][j+1].x,beam_pcd[i][j+1].y,beam_pcd[i][j+1].z;
				ba = b-a;
				bc = b-c;
				double babc = ba.dot(bc);
				double bc2 = bc.dot(bc);
				double ba2 = ba.dot(ba);
				beam_pcd[i][j].intensity = sqrt(ba2-(babc*babc/bc2)) ;
				//第一个是 当前雷达高度,第二个是 当前的曲率,第三个是线数
				if(beam_pcd[i][j-1].x<-0.8&&beam_pcd[i][j].intensity >0.01&&beam_pcd[i][j].normal_x<20){
					pc_out.push_back(beam_pcd[i][j]);
				}
			}
		}
	
}