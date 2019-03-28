//
// Created by echo on 19-3-18.
//

#ifndef PCD_COMPARE_MAPPING_H
#define PCD_COMPARE_MAPPING_H
#include <iostream>
#include <fstream>
#include <string>

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix标准函数定义*/
#include <sys/types.h>  /**/
#include <sys/stat.h>   /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
using namespace std;
using namespace g2o;
class mapping {
public:
	//变量:
	vector<VertexSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _vertices;
	vector<EdgeSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _odometryEdges;
	vector<EdgeSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _edges;
	//保存文件名
	string _outFilename = "LidarOdom.g2o";
	//函数:
	mapping() = default;
	//1.保存每次优化后的位姿结果
	bool saveg2o(Eigen::Isometry3d curr);
	//2. 保存最终g2o文件
	bool savefile();
	//3.
	
private:
	vector<int> _vertxfromnum;
	vector<int> _vertextonum;
	//存两个闭环边的tf
	vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _trans_loopclosure;
	int _vertexnum = 0;
	Eigen::Matrix<double, 6, 6> _information = Eigen::Matrix<double, 6, 6>::Identity();
};


#endif //PCD_COMPARE_MAPPING_H
