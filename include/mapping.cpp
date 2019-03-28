//
// Created by echo on 19-3-18.
//

#include "mapping.h"

bool mapping::saveg2o(Eigen::Isometry3d curr) {
		
		VertexSE3 *v = new VertexSE3;
		v->setId(_vertexnum++);
		
		//添加顶点
		Eigen::Isometry3d t_v;
		//当前的t
		t_v = curr;
		v->setEstimate(t_v);
		if (_vertexnum == 1) {
			v->setFixed(true);
		}
		
		_vertices.push_back(v);
		
		// 生成边
		if (_vertexnum > 1) {
			
			VertexSE3 *prev = _vertices[_vertexnum - 2]; // debug
			VertexSE3 *cur = _vertices[_vertexnum - 1];
			
			Eigen::Isometry3d t_e = prev->estimate().inverse() * cur->estimate();
			
			Eigen::Isometry3d ttmmpp = t_e; // debug
			EdgeSE3 *e = new EdgeSE3;
			
			e->setVertex(0, prev); //debug
			e->setVertex(1, cur); //debug
			e->setMeasurement(ttmmpp); //debug
			e->setInformation(_information);
			
			//正常边两个都有的
			_odometryEdges.push_back(e);
			_edges.push_back(e);
		}
}

bool mapping::savefile() {
	if (_vertices.size() > 0) {
		//ROS_ERROR("SAVING G2O");
		ofstream fileOutputStream;
		//cerr << "Writing into " << _outFilename << endl;
		fileOutputStream.open(_outFilename.c_str());
		
		string vertexTag = Factory::instance()->tag(_vertices[0]);
		string edgeTag = Factory::instance()->tag(_edges[0]);
		//todo 去除下划线的操作? _outFilename = "-" ->cout
		ostream &fout = _outFilename != "-" ? fileOutputStream : cout;
		
		for (size_t i = 0; i < _vertices.size(); ++i) {
			VertexSE3 *v = _vertices[i];
			fout << vertexTag << " " << v->id() << " ";
			//fout << vertexTag << " " << _vertices[i]->id() << " ";
			v->write(fout);
			//_vertices[i]->write(fout);
			fout << endl;
		}
		
		for (size_t i = 0; i < _edges.size(); ++i) {
			fout << edgeTag << " "
				 << static_cast<VertexSE3 *>(_edges[i]->vertex(0))->id()
				 << " " << static_cast<VertexSE3 *>(_edges[i]->vertex(1))->id()
				 << " ";
			_edges[i]->write(fout);
			fout << endl;
		}
		return true;
	} else {
		return false;
	}
}
