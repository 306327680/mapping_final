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
