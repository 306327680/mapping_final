//
// Created by echo on 2019/11/11.
//

#include "PoseGraphIO.h"

void PoseGraphIO::getPose() {

}

void PoseGraphIO::saveGraph(std::string g2o_path) {
	g2o::SparseOptimizer optimizer;     // 图模型
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
	typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
	auto solver = new g2o::OptimizationAlgorithmLevenberg(
			g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
	optimizer.setAlgorithm(solver);   // 设置求解器
	optimizer.setVerbose(true);       // 打开调试输出
	//把全部的pose存起来
	Eigen::Matrix<double, 6, 6> _information = Eigen::Matrix<double, 6, 6>::Identity();//信息矩阵

	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _odom_buffer_t;
	_odom_buffer_t = _odom_buffer;
	for (int i = 0; i < _odom_buffer_t.size(); ++i) {
		
		g2o::VertexSE3 *v = new g2o::VertexSE3();
		
		if (i == 0){
			v->setFixed(true);
		}
		//顶点
		v->setId(i);
		v->setEstimate(_odom_buffer_t[i]);
		optimizer.addVertex(v);
		
		// 生成边
		if (i > 0) {
			Eigen::Isometry3d t_e = _odom_buffer_t[i-1].inverse() * _odom_buffer_t[i] ;
			g2o::EdgeSE3 *e = new g2o::EdgeSE3();
			e->setVertex(0, optimizer.vertices()[i-1]); //debug
			e->setVertex(1, optimizer.vertices()[i]); //debug
			e->setMeasurement(t_e); //debug
			e->setInformation(_information);
			optimizer.addEdge(e);
		}
		
	}
/*	optimizer.initializeOptimization();
	optimizer.optimize(30);*/
	optimizer.save("/home/echo/test_ws/g2o/result.g2o");
	std::cout << "result saved!" << std::endl;
}

void PoseGraphIO::insertPose(Eigen::Isometry3d pose) {
	
	_odom_buffer.push_back(pose);
}
