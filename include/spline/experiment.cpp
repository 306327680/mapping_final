//
// Created by echo on 19-7-22.
//实验性的函数接口

#include "experiment.h"

double experiment::calculateTransDistLidar(Eigen::Isometry3d cur) {
	Eigen::Isometry3d temp;
	temp = cur.inverse() *_last_Lidar_pose;
	if(sqrt(temp(0,3)*temp(0,3)+temp(1,3)*temp(1,3)<1)){
		_lidarDistance += sqrt(temp(0,3)*temp(0,3)+temp(1,3)*temp(1,3));
	} else{
		std::cerr<<sqrt(temp(0,3)*temp(0,3)+temp(1,3)*temp(1,3))<<std::endl;
	}

	_last_Lidar_pose = cur;
	return _lidarDistance;
}

double experiment::calculateTransDistOdom(Eigen::Isometry3d cur) {
	Eigen::Isometry3d temp;
	temp = cur.inverse() *_last_Odom_pose;
	if(sqrt(temp(0,3)*temp(0,3)+temp(1,3)*temp(1,3)<1)){
		_odomDistance += sqrt(temp(0,3)*temp(0,3)+temp(1,3)*temp(1,3));
	}
	_last_Odom_pose = cur;
	return _odomDistance;
}

double experiment::getOdomErrorRate() {
	begin_count++;
	if(_odomDistance>=0&&_lidarDistance>=0){
		if((_odomDistance/_lidarDistance>1.1 || _odomDistance/_lidarDistance < 0.9) && begin_count > 500){
			std::cerr<<"rubbish encoder, current rate is : "<<_odomDistance/_lidarDistance<<" _odomDistance: "
			<<_odomDistance<<" _lidarDistance: "<<_lidarDistance<<std::endl;
			
		}
		return _odomDistance/_lidarDistance;
	} else {
		return -1;
	}
}

bool experiment::lidarStatusCheck( pcl::PointCloud<pcl::PointXYZ> input) {
	double averange;
	for (int i = 0; i < input.size(); ++i) {
		if(!std::isnan(sqrt(input[i].x*input[i].x + input[i].y * input[i].y + input[i].z * input[i].z)))
		averange += sqrt(input[i].x*input[i].x + input[i].y * input[i].y + input[i].z * input[i].z);
	}
	averange = averange/input.size();

	if(averange<5){
		std::cerr<<"range: "<<averange<<std::endl;
		return false;
	} else{
		return true;
	}
}

Eigen::Isometry3d experiment::ReOrthogonalization(Eigen::Isometry3d input) {
	Eigen::Isometry3d result;
	Eigen::Matrix3d rotation(input.rotation());
	Eigen::Matrix4d se3;
	Eigen::Matrix4d diff;
	Eigen::Quaterniond Quat;
	Eigen::Matrix3d rotation_normal;
	se3 = input.matrix();
	Quat = rotation;
	Quat.normalize();
	rotation_normal = Quat;
	result.setIdentity();
	result.rotate(rotation_normal);
	Eigen::Vector3d trans (se3(0,3),se3(1,3),se3(2,3));
	result.translation() = trans;
	diff = result.matrix()*input.inverse().matrix();

	return result;
}

Eigen::Isometry3d experiment::ReOrthogonalizationQR(Eigen::Isometry3d input) {
	return Eigen::Isometry3d();
}


//Prediction 部分
void Prediction::rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion) {
	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	double w, x, y, z;
	
	R(0, 0) = T(0, 0);
	R(0, 1) = T(0, 1);
	R(0, 2) = T(0, 2);
	R(1, 0) = T(1, 0);
	R(1, 1) = T(1, 1);
	R(1, 2) = T(1, 2);
	R(2, 0) = T(2, 0);
	R(2, 1) = T(2, 1);
	R(2, 2) = T(2, 2);
	
	double trace = R(0, 0) + R(1, 1) + R(2, 2);
	double epsilon = 1E-12;
	if (trace > epsilon)
	{
		double s = 0.5 / sqrt(trace + 1.0);
		w = 0.25 / s;
		x = -(R(2, 1) - R(1, 2)) * s;
		y = -(R(0, 2) - R(2, 0)) * s;
		z = -(R(1, 0) - R(0, 1)) * s;
		
	}
	else
	{
		if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
		{
			double s = 2.0 * sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
			w = -(R(2, 1) - R(1, 2)) / s;
			x = 0.25 * s;
			y = (R(0, 1) + R(1, 0)) / s;
			z = (R(0, 2) + R(2, 0)) / s;
			
		}
		else if (R(1, 1) > R(2, 2))
		{
			double s = 2.0 * sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
			w = -(R(0, 2) - R(2, 0)) / s;
			x = (R(0, 1) + R(1, 0)) / s;
			y = 0.25 * s;
			z = (R(1, 2) + R(2, 1)) / s;
		}
		else
		{
			double s = 2.0 * sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
			w = -(R(1, 0) - R(0, 1)) / s;
			x = (R(0, 2) + R(2, 0)) / s;
			y = (R(1, 2) + R(2, 1)) / s;
			z = 0.25 * s;
			
		}
		
	}
	q_quaternion(0) = w;
	q_quaternion(1) = x;
	q_quaternion(2) = y;
	q_quaternion(3) = z;
}

void Prediction::quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R) {
	Eigen::Quaterniond q;
	q.x() = q_quaternion(0);
	q.y() = q_quaternion(1);
	q.z() = q_quaternion(2);
	q.w() = q_quaternion(3);
//	R=Eigen::Matrix3d(q);
	R(0, 0) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(1), 2);
	R(0, 1) = 2.0000 * (q_quaternion(1)*q_quaternion(2) + q_quaternion(0)*q_quaternion(3));
	R(0, 2) = 2.0000 * (q_quaternion(1)*q_quaternion(3) - q_quaternion(0)*q_quaternion(2));
	R(1, 0) = 2.0000* (q_quaternion(1)*q_quaternion(2) - q_quaternion(0)*q_quaternion(3));
	R(1, 1) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(2), 2);
	R(1, 2) = 2.0000* (q_quaternion(2)*q_quaternion(3) + q_quaternion(0)*q_quaternion(1));
	R(2, 0) = 2.0000 * (q_quaternion(1)*q_quaternion(3) + q_quaternion(0)*q_quaternion(2));
	R(2, 1) = 2.0000 * (q_quaternion(2)*q_quaternion(3) - q_quaternion(0)*q_quaternion(1));
	R(2, 2) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(3), 2);
	
}

void Prediction::trinterp(Eigen::Matrix4d &T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T) {
	Eigen::Vector4d q0, q1, qr;
	Eigen::Vector3d p0, p1, pr;
	Eigen::Matrix3d R, R0;
	//R 为插值之后的旋转矩阵
	
	rotMat2quaternion(T0, q0);  // 位姿矩阵转换为四元数
	rotMat2quaternion(T1, q1);
	
	p0 << T0(0, 3), T0(1, 3), T0(2, 3);       // 提取出位置矩阵
	p1 << T1(0, 3), T1(1, 3), T1(2, 3);
	
	qinterp(q0, q1, r, qr);      // 进行四元数插值 10.4
	pr = p0*(1 - r) + r*p1;      // 进行位置插值
	
	quatern2rotMat(qr, R);       // 四元数转旋转矩阵
	
	T(0, 0) = R(0, 0); T(0, 1) = R(0, 1); T(0, 2) = R(0, 2); T(0, 3) = pr(0);
	T(1, 0) = R(1, 0); T(1, 1) = R(1, 1); T(1, 2) = R(1, 2); T(1, 3) = pr(1);
	T(2, 0) = R(2, 0); T(2, 1) = R(2, 1); T(2, 2) = R(2, 2); T(2, 3) = pr(2);
	T(3, 0) = 0;       T(3, 1) = 0;       T(3, 2) = 0;       T(3, 3) = 1;
}

void Prediction::qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r,
						 Eigen::Vector4d &q_quaternion_interpolation) {
	double k0, k1, theta, sin_theta, cos_theta;
	Eigen::Vector4d q_temp;
	if ((r<0) || (r>1)){
//		std::cout << " R out of rang : " <<r<< std::endl;
	}
	
	cos_theta = Q1(0)*Q2(0) + Q1(1)*Q2(1) + Q1(2)*Q2(2) + Q1(3)*Q2(3);
	
	if (cos_theta < 0) {
		Q2 = -Q2;
		cos_theta = -cos_theta;
	}
	
	if ((cos_theta) > 0.9999999999) {
		k0 = 1.00 - r;
		k1 = r;
		
	}
	else {
		sin_theta = sqrt(1.00 - pow(cos_theta, 2));
		theta = atan2(sin_theta, cos_theta);
		k0 = sin((1.000 - r)*theta) / sin(theta);
		k1 = sin((r)*theta) / sin(theta);
	}
	
	q_quaternion_interpolation = k0* Q1 + k1 * Q2;
	Eigen::Quaterniond out;
	out.x() = q_quaternion_interpolation(0);
	out.y() = q_quaternion_interpolation(1);
	out.z() = q_quaternion_interpolation(2);
	out.w() = q_quaternion_interpolation(3);
	out.normalize();
	q_quaternion_interpolation(0) = out.x();
	q_quaternion_interpolation(1) = out.y();
	q_quaternion_interpolation(2) = out.z();
	q_quaternion_interpolation(3) = out.w();
}

Eigen::Isometry3d Prediction::motion_b_spline(double rate, Eigen::Isometry3d trans) {
	Eigen::Isometry3d out;
	Eigen::Matrix4d out_4d;
	out = Eigen::Isometry3d::Identity();
	trinterp(out.matrix(),trans.matrix(),rate,out_4d);
	out = out_4d;
	return out;
}


Eigen::Isometry3d SplineFusion::cumulativeForm(Eigen::Isometry3d T_1,Eigen::Isometry3d T_2,
											   Eigen::Isometry3d T_3,Eigen::Isometry3d T_4, double u) {
	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
	Sophus::SE3 cur;
	SE3Eigen2Sophus(T_1,t1);
	SE3Eigen2Sophus(T_2,t2);
	SE3Eigen2Sophus(T_3,t3);
	SE3Eigen2Sophus(T_4,t4);
	cur =   t1*
			Sophus::SE3::exp(((5 + 3*u - 3*u*u + u*u*u) / 6)*fromAtoB(t1,t2).log())*
			Sophus::SE3::exp(((1 + 3*u + 3*u*u - 2*u*u*u) / 6)*fromAtoB(t2,t3).log())*
			Sophus::SE3::exp(((u*u*u)/6)*fromAtoB(t3,t4).log());
	result = cur.matrix();
	return result;
	
}

void SplineFusion::SE3Eigen2Sophus(Eigen::Isometry3d e, Sophus::SE3 & s) {
	Sophus::SE3 t1;
	Eigen::Matrix4d temp;
	temp = e.matrix();
	Eigen::Vector3d t(temp(0,3),temp(1,3),temp(2,3));
	t1 = Sophus::SE3(e.rotation(),t);
	s = t1;
}

Sophus::SE3 SplineFusion::fromAtoB(Sophus::SE3 a, Sophus::SE3 b) {
	return Sophus::SE3(a.inverse()*b);
}

void SplineFusion::test() {
	Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/90, Eigen::Vector3d(0,0,1)).toRotationMatrix();
	Eigen::Vector3d t(1,0,0);
	Eigen::Isometry3d temp_pose1;
	Eigen::Isometry3d temp_pose2;
	Eigen::Isometry3d temp_pose3;
	Eigen::Isometry3d temp_pose4;
	pcl::PointXYZ temp1;
	temp_pose1.setIdentity();
	temp_pose1.rotate(Eigen::AngleAxisd(M_PI/8, Eigen::Vector3d(0,3,1)).toRotationMatrix());
	temp_pose1.translate(Eigen::Vector3d(0,0,0));

	temp_pose2.setIdentity();
	temp_pose2.rotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d(0,4,1)).toRotationMatrix());
	temp_pose2.translate(Eigen::Vector3d(0,0,1));

	temp_pose3.setIdentity();
	temp_pose3.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,5,1)).toRotationMatrix());
	temp_pose3.translate(Eigen::Vector3d(0,0,2));

	temp_pose4.setIdentity();
	temp_pose4.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,3,1)).toRotationMatrix());
	temp_pose4.translate(Eigen::Vector3d(0,0,3));

	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose1,temp_pose2,temp_pose3,temp_pose4,i/100.0);
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		after.push_back(temp1);
	}
//	pcl::io::savePCDFile("temp.pcd",after);
}
