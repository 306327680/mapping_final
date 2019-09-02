//
// Created by echo on 2019/8/8.
//

#include "lmOptimizationSufraceCorner.h"


bool lmOptimizationSufraceCorner::surfConstraint(pcl::PointCloud<pcl::PointXYZI> surfNear, pcl::PointXYZI curPoint,
												 Eigen::Vector4d& result) {

	matA0 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
	matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
	matX0 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));
	
	for (int j = 0; j < 5; j++) {//地图最近的5个点
		matA0.at<float>(j, 0) = surfNear[j].x;
		matA0.at<float>(j, 1) = surfNear[j].y;
		matA0.at<float>(j, 2) = surfNear[j].z;
	}
	//todo 这个qr分解没法处理纯平面上面的
	cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);//solve函数，用来解线性方程 A*X=B qr分解方法求得
	
	float pa = matX0.at<float>(0, 0);
	float pb = matX0.at<float>(1, 0);
	float pc = matX0.at<float>(2, 0);//各个轴的法向量 拟合平面
	float pd = 1;
	
	std::cout<<"**解线性方程 A*X=B qr分解方法求得,B 为-1的矩阵,解出来为平面法向量: \n"<<matX0<<std::endl;
	
	float ps = sqrt(pa * pa + pb * pb + pc * pc); //归一化
	pa /= ps; pb /= ps; pc /= ps; pd /= ps;
	
	bool planeValid = true;
	for (int j = 0; j < 5; j++) {
		if (fabs(pa * surfNear[j].x +
				 pb * surfNear[j].y +
				 pc * surfNear[j].z + pd) > 0.2) {//点面距离距离太大就不要了
			planeValid = false;
			return false;
		}
	}
	
	if (planeValid) {
		float pd2 = pa * curPoint.x + pb * curPoint.y + pc * curPoint.z + pd; //点面距
		
		float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(curPoint.x * curPoint.x
												  + curPoint.y * curPoint.y + curPoint.z * curPoint.z));//当前扫描到并且选择的点越远越不信
		
		result(0) = s * pa;//方向
		result(1) = s * pb;
		result(2) = s * pc;
		result(3) = s * pd2;//大小
		_constraint.push_back(std::make_pair(result,curPoint));
		std::cout<<"***1. surface 约束产生的当前点到面朝向的单位向量 px: "<<pa<<" py: "<<pb<<" pz: "
		<<pc<<"*** 点线距: pd2: "<<pd2<<" lm error项: "<<s*pd2<<std::endl;
		
		if (s <= 0.1) {
			return false;
		} else {
			return true;
		}
	}
}

bool lmOptimizationSufraceCorner::cornerConstraint(pcl::PointCloud<pcl::PointXYZI> cornerNear, pcl::PointXYZI curPoint,
												   Eigen::Vector4d &result) {
	matA1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
	matD1 = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
	matV1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
	
	pcl::PointXYZI  pointProj, coeff;

	
	float cx = 0, cy = 0, cz = 0;
	for (int j = 0; j < 5; j++) {
		cx += cornerNear[j].x;
		cy += cornerNear[j].y;
		cz += cornerNear[j].z;
	}
	cx /= 5; cy /= 5;  cz /= 5;//4.求得质心坐标
	
	float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
	for (int j = 0; j < 5; j++) {
		float ax = cornerNear[j].x - cx;
		float ay = cornerNear[j].y - cy;
		float az = cornerNear[j].z - cz;
		//5. 均方差
		a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
		a22 += ay * ay; a23 += ay * az;
		a33 += az * az;
	}
	a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5; //到质心的均方差
	//6. matA1 均方差矩阵
	matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
	matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
	matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;
	
	cv::eigen(matA1, matD1, matV1);
	std::cout <<"当前的几个点构成的 到质心的均方差 的 特征值" << matD1 << std::endl;
	std::cout <<"当前的几个点构成的 到质心的均方差 的 特征向量"<< matV1 << std::endl;
	
	if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {//在一条直线上面的显著性
		
		float x0 = curPoint.x;
		float y0 = curPoint.y;
		float z0 = curPoint.z;
		float x1 = cx + 0.1 * matV1.at<float>(0, 0);
		float y1 = cy + 0.1 * matV1.at<float>(0, 1);
		float z1 = cz + 0.1 * matV1.at<float>(0, 2);
		float x2 = cx - 0.1 * matV1.at<float>(0, 0);
		float y2 = cy - 0.1 * matV1.at<float>(0, 1);
		float z2 = cz - 0.1 * matV1.at<float>(0, 2);
		
		float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
						  + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
							* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
						  + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
							* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
		
		float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
		
		float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
					+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;
		
		float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
					 - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
		
		float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
					 + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
		
		float ld2 = a012 / l12;//点线距
		
		float s = 1 - 0.9 * fabs(ld2);
		std::cout<<"**1. corner 约束产生的当前点到边朝向的单位向量: x: "<<la<<" y: "<<lb<<" z: "<<lc<<" 点线距: ld2: "<<ld2<<" s*ld2: "<<s*ld2<<std::endl;
		
		if (s > 0.1) {
			result(0) = s * la;//方向
			result(1) = s * lb;
			result(2) = s * lc;
			result(3) = s * ld2;//大小
			_constraint.push_back(std::make_pair(result,curPoint));
			return true;
		}else {
			return false;
		}
	}
	return false;
}
using namespace std;
bool lmOptimizationSufraceCorner::LMoptimization(int interationTimes) {
	
	float srx = sin(_lidar_pose[0]);//当前pose的orientation roll pitch yaw
	float crx = cos(_lidar_pose[0]);
	float sry = sin(_lidar_pose[1]);
	float cry = cos(_lidar_pose[1]);
	float srz = sin(_lidar_pose[2]);
	float crz = cos(_lidar_pose[2]);
	
	//只测试两个约束点
	cv::Mat matA(2, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat matAt(6, 2, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat matB(2, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
	
	pcl::PointXYZI  pointProj, coeff,pointOri;
	pcl::PointXYZI coeffSel;
	for (int i = 0; i < _constraint.size(); i++) {
		coeff.x = _constraint[i].first(0);//每组优化的方向和大小
		coeff.y = _constraint[i].first(1);
		coeff.z = _constraint[i].first(2);
		coeff.intensity = _constraint[i].first(3);
		pointOri = _constraint[i].second;
		float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
					+ (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z)          * coeff.y
					+ (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;//约束转到当前坐标系下?
		
		float ary = ((cry*srx*srz - crz*sry)*pointOri.x
					 + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
					+ ((-cry*crz - srx*sry*srz)*pointOri.x
					   + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;
		
		float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
					+ (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
					+ ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
		cout<<"转化后的优化方向: "<<i<<endl;
		cout<<"arx: 1"<<arx<<endl;
		cout<<"ary: "<<ary<<endl;
		cout<<"arz: "<<arz<<endl;
		matA.at<float>(i, 0) = arx;
		matA.at<float>(i, 1) = ary;
		matA.at<float>(i, 2) = arz;
		matA.at<float>(i, 3) = coeff.x;
		matA.at<float>(i, 4) = coeff.y;
		matA.at<float>(i, 5) = coeff.z;
		matB.at<float>(i, 0) = -coeff.intensity;//error大小
		std::cout<<"当前的优化向量 \n: "<<matA<<std::endl;
	}
	std::cout<<"总的优化向量 \n: "<<matA<<std::endl;
	cv::transpose(matA, matAt);
	matAtA = matAt * matA;
	matAtB = matAt * matB;
	cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);//AX = B
	std::cout<<"matA: \n"  <<matA  <<std::endl;
	std::cout<<"matB: \n"  <<matB  <<std::endl;
	std::cout<<"matAtA: \n"<<matAtA<<std::endl;
	std::cout<<"matAtB: \n"<<matAtB<<std::endl;
	std::cout<<"matX: \n"  <<matX  <<std::endl;
	
	
	if (interationTimes == 0) {
		cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
		cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
		cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
		
		cv::eigen(matAtA, matE, matV);
		matV.copyTo(matV2);
		
		isDegenerate = false;
		float eignThre[6] = {100, 100, 100, 100, 100, 100};
		for (int i = 5; i >= 0; i--) {
			if (matE.at<float>(0, i) < eignThre[i]) {
				for (int j = 0; j < 6; j++) {
					matV2.at<float>(i, j) = 0;
				}
				isDegenerate = true;
			} else {
				break;
			}
		}
		matP = matV.inv() * matV2;
	}
	
	if (isDegenerate) {
		cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
		matX.copyTo(matX2);
		matX = matP * matX2;
	}
	//更新当前位姿
	_lidar_pose[0] += matX.at<float>(0, 0);
	_lidar_pose[1] += matX.at<float>(1, 0);
	_lidar_pose[2] += matX.at<float>(2, 0);
	_lidar_pose[3] += matX.at<float>(3, 0);
	_lidar_pose[4] += matX.at<float>(4, 0);
	_lidar_pose[5] += matX.at<float>(5, 0);
	
	float deltaR = sqrt(
			pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
			pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
			pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
	float deltaT = sqrt(
			pow(matX.at<float>(3, 0) * 100, 2) +
			pow(matX.at<float>(4, 0) * 100, 2) +
			pow(matX.at<float>(5, 0) * 100, 2));
	
	if (deltaR < 0.05 && deltaT < 0.05) {
		return true;
	}
	return false;

}

void lmOptimizationSufraceCorner::currentPose(Eigen::Isometry3d pose) {
	Eigen::Vector3d RPY = pose.rotation().eulerAngles(2, 1, 0);//RPY
	Eigen::Vector3d XYZ = pose.translation();
	_lidar_pose.push_back(RPY[0]);
	_lidar_pose.push_back(RPY[1]);
	_lidar_pose.push_back(RPY[2]);
	_lidar_pose.push_back(XYZ[0]);
	_lidar_pose.push_back(XYZ[1]);
	_lidar_pose.push_back(XYZ[2]);
}

void lmOptimizationSufraceCorner::eigen2RPYXYZ(Eigen::Isometry3d pose, std::vector<double> & vector) {
	vector.clear();
	Eigen::Vector3d t(1, 0, 0);
	Sophus::SE3 temp;
	Sophus::SO3 rotate(pose.rotation());
	Eigen::Vector3d RPY = pose.rotation().eulerAngles(2, 1, 0);//RPY
	Eigen::Vector3d XYZ = pose.translation();
	vector.push_back(RPY[0]);
	vector.push_back(RPY[1]);
	vector.push_back(RPY[2]);
	vector.push_back(XYZ[0]);
	vector.push_back(XYZ[1]);
	vector.push_back(XYZ[2]);
}

void lmOptimizationSufraceCorner::RPYXYZ2eigen(std::vector<double> &vector, Eigen::Isometry3d &pose) {
	Eigen::Matrix4d lidarodomT;
	lidarodomT.setIdentity();
	lidarodomT(0,3) = vector[3];
	lidarodomT(1,3) = vector[4];
	lidarodomT(2,3) = vector[5];
	lidarodomT(3,3) = 1;
	Eigen::AngleAxisd rollAngle (vector[0], Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(vector[1], Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle  (vector[2], Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond quaternion;
	quaternion=rollAngle*pitchAngle*yawAngle;
	pose.matrix() = lidarodomT.matrix();
	pose.rotate(quaternion);
}

