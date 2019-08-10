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

void mapping::setup() {
	laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
	laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
	laserCloudCornerStack.reset(new pcl::PointCloud<PointType>());
	laserCloudSurfStack.reset(new pcl::PointCloud<PointType>());
//滤波用temp点
	laserCloudCornerStack2.reset(new pcl::PointCloud<PointType>());
	laserCloudSurfStack2.reset(new pcl::PointCloud<PointType>());
	laserCloudOri.reset(new pcl::PointCloud<PointType>());
	coeffSel.reset(new pcl::PointCloud<PointType>());
	laserCloudSurround.reset(new pcl::PointCloud<PointType>());
	laserCloudSurround2.reset(new pcl::PointCloud<PointType>());
	laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
	laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
	laserCloudFullRes .reset(new pcl::PointCloud<PointType>());
	laserCloudCornerArray->resize(laserCloudNum);
	laserCloudSurfArray->resize(laserCloudNum);
	laserCloudCornerArray2->resize(laserCloudNum);
	laserCloudSurfArray2->resize(laserCloudNum);
}

void mapping::getPoint(pcl::PointCloud<PointType> corner, pcl::PointCloud<PointType> surface,
					   pcl::PointCloud<PointType> full_cloud, Eigen::Isometry3d Initpose) {
	*laserCloudCornerLast = corner;
	*laserCloudSurfLast = surface;
	*laserCloudFullRes = full_cloud;
	double roll, pitch, yaw;
	//不太对?
	yaw=atan2(Initpose(1,0),Initpose(0,0));
	pitch=atan2(-Initpose(2,0),sqrt(Initpose(2,1)*Initpose(2,1) + Initpose(2,2)*Initpose(2,2)));
	roll=atan2(Initpose(2,1),Initpose(2,2));
	//这个应该对
	transformSum.rot_x = roll;
	transformSum.rot_y = pitch;
	transformSum.rot_z = yaw;
	//todo 这个不一定对
	transformSum.pos.x() = static_cast<float>(Initpose(0, 3));
	transformSum.pos.y() = static_cast<float>(Initpose(1, 3));
	transformSum.pos.z() = static_cast<float>(Initpose(2, 3));
	
}

void mapping::transformAssociateToMap() {
	Vector3 v0 = transformBefMapped.pos -  transformSum.pos;
	Vector3 v1 = rotateY( v0, -(transformSum.rot_y) );
	Vector3 v2 = rotateX( v1, -(transformSum.rot_x) );
	transformIncre.pos = rotateZ( v2, -(transformSum.rot_z) );
	
	float sbcx = transformSum.rot_x.sin();
	float cbcx = transformSum.rot_x.cos();
	float sbcy = transformSum.rot_y.sin();
	float cbcy = transformSum.rot_y.cos();
	float sbcz = transformSum.rot_z.sin();
	float cbcz = transformSum.rot_z.cos();
	
	float sblx = transformBefMapped.rot_x.sin();
	float cblx = transformBefMapped.rot_x.cos();
	float sbly = transformBefMapped.rot_y.sin();
	float cbly = transformBefMapped.rot_y.cos();
	float sblz = transformBefMapped.rot_z.sin();
	float cblz = transformBefMapped.rot_z.cos();
	
	float salx = transformAftMapped.rot_x.sin();
	float calx = transformAftMapped.rot_x.cos();
	float saly = transformAftMapped.rot_y.sin();
	float caly = transformAftMapped.rot_y.cos();
	float salz = transformAftMapped.rot_z.sin();
	float calz = transformAftMapped.rot_z.cos();
	
	float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
				- cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
							 - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
				- cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
							 - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
	transformTobeMapped.rot_x = -asin(srx);
	
	float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
						 - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
				   - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
								+ (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
				   + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
								+ (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
	float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
						 - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
				   + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
								+ (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
				   - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
								+ (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
	transformTobeMapped.rot_y = atan2(srycrx / transformTobeMapped.rot_x.cos(),
									  crycrx / transformTobeMapped.rot_x.cos());
	
	float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
												 - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
				   - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
												   - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
				   + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
	float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
												 - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
				   - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
												   - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
				   + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
	transformTobeMapped.rot_z = atan2(srzcrx / transformTobeMapped.rot_x.cos(),
									  crzcrx / transformTobeMapped.rot_x.cos());
	
	Vector3 v3;
	v1 = rotateZ( transformIncre.pos,  transformTobeMapped.rot_z);
	v2 = rotateX( v1,  transformTobeMapped.rot_x);
	v3 = rotateY( v2,  transformTobeMapped.rot_y);
	transformTobeMapped.pos = transformAftMapped.pos - v3;
}

void mapping::transformUpdate() {
	if (imuPointerLast >= 0) {
		float imuRollLast = 0, imuPitchLast = 0;
		while (imuPointerFront != imuPointerLast) {
			if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
				break;
			}
			imuPointerFront = (imuPointerFront + 1) % imuQueLength;
		}
		
		if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
			imuRollLast = imuRoll[imuPointerFront];
			imuPitchLast = imuPitch[imuPointerFront];
		} else {
			int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
			float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack])
							   / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
			float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod)
							  / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
			
			imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
			imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
		}
		
		transformTobeMapped.rot_x = 0.998 * transformTobeMapped.rot_x.value() + 0.002 * imuPitchLast;
		transformTobeMapped.rot_z = 0.998 * transformTobeMapped.rot_z.value() + 0.002 * imuRollLast;
	}
	
	transformBefMapped = transformSum;
	transformAftMapped = transformTobeMapped;
}

void mapping::pointAssociateToMap(PointType const *const pi, PointType *const po) {
	Vector3 v1 = rotateZ( *pi, transformTobeMapped.rot_z);
	Vector3 v2 = rotateX(  v1, transformTobeMapped.rot_x);
	Vector3 v3 = rotateY(  v2, transformTobeMapped.rot_y);
	v3 += transformTobeMapped.pos;
	
	po->x = v3.x();
	po->y = v3.y();
	po->z = v3.z();
	po->intensity = pi->intensity;
}

void mapping::pointAssociateTobeMapped(const PointType *const pi, PointType *const po) {
	Vector3 v0 = Vector3(*pi) - transformTobeMapped.pos;
	Vector3 v1 = rotateY( v0, -transformTobeMapped.rot_y);
	Vector3 v2 = rotateX( v1, -transformTobeMapped.rot_x);
	Vector3 v3 = rotateZ( v2, -transformTobeMapped.rot_z);
	
	po->x = v3.x();
	po->y = v3.y();
	po->z = v3.z();
	po->intensity = pi->intensity;
}
