//
// Created by echo on 2020/8/7.
//

#ifndef PCD_COMPARE_IMU_TOOLS_H
#define PCD_COMPARE_IMU_TOOLS_H
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#define Kp 10.0f                        // 这里的KpKi是用于调整加速度计修正陀螺仪的速度
#define Ki 0.008f
#define halfT 0.001f             // 采样周期的一半，用于求解四元数微分方程时计算角增量
class imu_tools {
public:
	imu_tools(){};
	void imu_intergration_euler(Eigen::VectorXd imu_mesurement,Eigen::Vector3d &vel_in,Eigen::Quaterniond &q_in,double period_time = 0.01);//1.输入当前的measurement 和当前的pq 返回积分后的pq
	int voidIMUupdate(float gx, float gy, float gz, float ax, float ay, float az);//g表陀螺仪，a表加计
	float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 初始姿态四元数，由上篇博文提到的变换四元数公式得来
	float exInt = 0, eyInt = 0, ezInt = 0;    //当前加计测得的重力加速度在三轴上的分量//与用当前姿态计算得来的重力在三轴上的分量的误差的积分
	Eigen::Vector3d gravity,gravity_w,bias_gyro;
private:
};


#endif //PCD_COMPARE_IMU_TOOLS_H
