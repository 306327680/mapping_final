//
// Created by echo on 19-3-11.
//

#include "lidarOdom.h"

void lidarOdom::initializationValue() {
	//1.初始化各种指针
	cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
	cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
	surfPointsFlat.reset(new pcl::PointCloud<PointType>());
	surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());
	laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
	laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
	laserCloudOri.reset(new pcl::PointCloud<PointType>());
	coeffSel.reset(new pcl::PointCloud<PointType>());
	laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
	imuTrans.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

void lidarOdom::getPC(pcl::PointCloud<PointType>::Ptr cornerPointsSharp_out,
					  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp_out,
					  pcl::PointCloud<PointType>::Ptr surfPointsFlat_out,
					  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat_out,
					  pcl::PointCloud<PointType>::Ptr laserCloudFullRes_out) {
	if(systemReady){
		*laserCloudCornerLast = *cornerPointsSharp;
		*laserCloudSurfLast = *surfPointsLessFlat;
	}
	// 这俩安顿下
	*laserCloudFullRes = *laserCloudFullRes_out;
	*cornerPointsSharp = *cornerPointsSharp_out;
	*cornerPointsLessSharp = *cornerPointsLessSharp_out;
	*surfPointsFlat = *surfPointsFlat_out;
	*surfPointsLessFlat = *surfPointsLessFlat_out;
/*	writer_pcd.write("0laserCloudFullRes.pcd",*laserCloudFullRes);
	writer_pcd.write("1cornerPointsSharp.pcd",*cornerPointsSharp);
	writer_pcd.write("2cornerPointsLessSharp.pcd",*cornerPointsLessSharp);
	writer_pcd.write("4surfPointsFlat.pcd",*surfPointsFlat);
	writer_pcd.write("5surfPointsLessFlat.pcd",*surfPointsLessFlat);*/
	
	
}
//这个函数应该差不多 问题不大
void lidarOdom::cornerConstraint() {
	//todo intensity 都得改了
	laserCloudOri->clear();
	coeffSel->clear();
	std::vector<int> pointSearchInd(1);
	std::vector<float> pointSearchSqDis(1);
	std::vector<int> indices;
	//回头主函数里面的 不应该出现在这里
	
	pcl::removeNaNFromPointCloud(*cornerPointsSharp,*cornerPointsSharp, indices);
	int cornerPointsSharpNum = cornerPointsSharp->points.size();
	
	pcl::removeNaNFromPointCloud(*laserCloudCornerLast,*laserCloudCornerLast, indices);
	kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
/*	std::cout<<"cornerPointsSharpNum: "<<cornerPointsSharpNum<<std::endl;*/
	test_result.resize(cornerPointsSharpNum);
	for (int i = 0; i < cornerPointsSharpNum; i++) {
		//c.1 转换每一个点到 当前的位姿 进行抗畸变 pointSel 为抗畸变之后的点云
		TransformToStart(&cornerPointsSharp->points[i], &pointSel);
		
		//c.2 迭代满5次 更新线约束的候选点
		//没仔细看
		if (iterCount % 5 == 0) {//#***更新线约束的候选点
			
			//d.1 建立kdtree 1代表只找一个出来 第一个表示围绕这个点
			// 第三个表示 在所有点云中被选中的序号 第四个表示被选中的距离
			//laserCloudCornerLast 在上次的所有的点中找到当前扫描中的一个点的最近的一个点
			
			kdtreeCornerLast.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
			//设置初值
			int closestPointInd = -1, minPointInd2 = -1;
			//d.2 如果最近的点的距离小于25m(?)
			if (pointSearchSqDis[0] < 25) {
				closestPointInd = pointSearchInd[0];
				//e.1 取出该点的线数
				int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].beam);
				float pointSqDis, minPointSqDis2 = 25;
				//e.2  closestPointInd是上次扫描中最近的点的序号 cornerPointsSharpNum是这次的总点数
				for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
					//f.1 如果这之后的特征(同上次扫描的)比上次扫描到的最近的点的(线束)距离大3 跳出循环
					if (int(laserCloudCornerLast->points[j].beam) > closestPointScan + 2.5) {
						break;
					}
					//f.2 上次扫描的(特征)点到这一次扫描的平方距离
					pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
								 (laserCloudCornerLast->points[j].x - pointSel.x) +
								 (laserCloudCornerLast->points[j].y - pointSel.y) *
								 (laserCloudCornerLast->points[j].y - pointSel.y) +
								 (laserCloudCornerLast->points[j].z - pointSel.z) *
								 (laserCloudCornerLast->points[j].z - pointSel.z);
					
					if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
						//f.3 如果是 当前线+1 / 当前线 +2 的话 找到最小距离的点 和 那个点的序号
						if (pointSqDis < minPointSqDis2) {
							minPointSqDis2 = pointSqDis;
							minPointInd2 = j;
						}
					}
				}
				//e.3 找到 当前线-1 / 当前线 -2 最小距离的点和序号
				for (int j = closestPointInd - 1; j >= 0; j--) {
					if (int(laserCloudCornerLast->points[j].beam) < closestPointScan - 2.5) {
						break;
					}
					
					pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
								 (laserCloudCornerLast->points[j].x - pointSel.x) +
								 (laserCloudCornerLast->points[j].y - pointSel.y) *
								 (laserCloudCornerLast->points[j].y - pointSel.y) +
								 (laserCloudCornerLast->points[j].z - pointSel.z) *
								 (laserCloudCornerLast->points[j].z - pointSel.z);
					
					if (int(laserCloudCornerLast->points[j].beam) < closestPointScan) {
						if (pointSqDis < minPointSqDis2) {
							minPointSqDis2 = pointSqDis;
							minPointInd2 = j;
						}
					}
				}
			}
			//用来构建线约束的两个点的序号(上次扫描的)
			pointSearchCornerInd1[i] = closestPointInd;
			pointSearchCornerInd2[i] = minPointInd2;
		}
		
		
		//c.3 不满5次也要进行的 lm? ****线约束****
		if (pointSearchCornerInd2[i] >= 0) {
			//d.1 得到 构建线约束的两个点
			tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
			tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];
			//d.2 当前帧的点,上次构建线特征的两个点
			float x0 = pointSel.x;
			float y0 = pointSel.y;
			float z0 = pointSel.z;
			float x1 = tripod1.x;
			float y1 = tripod1.y;
			float z1 = tripod1.z;
			float x2 = tripod2.x;
			float y2 = tripod2.y;
			float z2 = tripod2.z;
			//向量OA = (x0 - x1, y0 - y1, z0 - z1),
			// 向量OB = (x0 - x2, y0 - y2, z0 - z2)，
			// 向量AB = （x1 - x2, y1 - y2, z1 - z2）
			//向量OA OB的向量积(即叉乘)为：
			//|  i      j      k  |
			//|x0-x1  y0-y1  z0-z1|
			//|x0-x2  y0-y2  z0-z2|
			//模为：
			float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							  * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							  + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
								* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
							  + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
								* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
			//两个最近距离点之间的距离，即向量AB的模
			float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
			//AB方向的单位向量与OAB平面的单位法向量的向量积在各轴上的分量（d的方向）
			//x轴分量i
			float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
						+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;
			//y轴分量j
			float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
						 - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
			//z轴分量k
			float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
						 + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
			//点线距 //点到线的距离，d = |向量OA 叉乘 向量OB|/|AB|
			float ld2 = a012 / l12;
			
			float s = 1;// 阻尼因子//权重计算，距离越大权重越小，距离越小权重越大，得到的权重范围<=1
			if (iterCount >= 5) {//5次迭代之后开始增加权重因素
				s = 1 - 1.8 * fabs(ld2); //点线越近权重越大
			}
			
			coeff.x = s * la;
			coeff.y = s * lb;
			coeff.z = s * lc;
			coeff.intensity = s * ld2;
			
			if (s > 0.1 && ld2 != 0) {//fabs(ld2) <0.5 && ld2!=0 点线距
				//才选中进行优化 存入满足条件的这次的点云
				laserCloudOri->push_back(cornerPointsSharp->points[i]);
				coeffSel->push_back(coeff);
			}
		}
	}
	//保存测试结果
/*
	writer_pcd.write("0laserCloudFullRes.pcd",*laserCloudFullRes);
	writer_pcd.write("1laserCloudCornerLast.pcd",*laserCloudCornerLast);
	writer_pcd.write("2laserCloudOri.pcd",*laserCloudOri);
	writer_pcd.write("coeffSel.pcd",*coeffSel);*/
}

void lidarOdom::surfConstraint() {
	int surfPointsFlatNum = surfPointsFlat->points.size();
	/*std::cout<<surfPointsFlatNum<<" "<<surfPointsLessFlat->points.size()<<std::endl;
	util time;*/
	std::vector<int> pointSearchInd(1);
	std::vector<float> pointSearchSqDis(1);
	//b.2 对每一个surf 进行迭代
	
	kdtreeSurfLast.setInputCloud(laserCloudSurfLast);
	for (int i = 0; i < surfPointsFlatNum; i++) {
		//c.1 抗畸变
		TransformToStart(&surfPointsFlat->points[i], &pointSel);
		//c.2 迭代满5次
		if (iterCount % 5 == 0) {
			//d.1 建立kdtree 1代表只找一个出来 第一个表示围绕这个点 每5次更新下临近点
			// 第三个表示 在所有点云中被选中的序号 第四个表示被选中的距离
			
			kdtreeSurfLast.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
			int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
			//d.2 找到 kdtree中的临近点 找到距离小于25的点(均方差)
			if (pointSearchSqDis[0] < 25) {
				//e.1 最近的点的id
				closestPointInd = pointSearchInd[0];
				//e.2 取出来当前的线数
				int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].beam);
				float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
		
				for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
					if (int(laserCloudSurfLast->points[j].beam) > closestPointScan + 2.5) {
						break;
					}
					pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
								 (laserCloudSurfLast->points[j].x - pointSel.x) +
								 (laserCloudSurfLast->points[j].y - pointSel.y) *
								 (laserCloudSurfLast->points[j].y - pointSel.y) +
								 (laserCloudSurfLast->points[j].z - pointSel.z) *
								 (laserCloudSurfLast->points[j].z - pointSel.z);
					
					if (int(laserCloudSurfLast->points[j].beam) <= closestPointScan) {
						if (pointSqDis < minPointSqDis2) {
							minPointSqDis2 = pointSqDis;
							minPointInd2 = j;
						}
					} else {
						if (pointSqDis < minPointSqDis3) {
							minPointSqDis3 = pointSqDis;
							minPointInd3 = j;
						}
					}
				}

				for (int j = closestPointInd - 1; j >= 0; j--) {
					if (int(laserCloudSurfLast->points[j].beam) < closestPointScan - 2.5) {
						break;
					}
					
					pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
								 (laserCloudSurfLast->points[j].x - pointSel.x) +
								 (laserCloudSurfLast->points[j].y - pointSel.y) *
								 (laserCloudSurfLast->points[j].y - pointSel.y) +
								 (laserCloudSurfLast->points[j].z - pointSel.z) *
								 (laserCloudSurfLast->points[j].z - pointSel.z);
					
					if (int(laserCloudSurfLast->points[j].beam) >= closestPointScan) {
						if (pointSqDis < minPointSqDis2) {
							minPointSqDis2 = pointSqDis;
							minPointInd2 = j;
						}
					} else {
						if (pointSqDis < minPointSqDis3) {
							minPointSqDis3 = pointSqDis;
							minPointInd3 = j;
						}
					}
				}
			}
			//构成面的3个点
			pointSearchSurfInd1[i] = closestPointInd;//kd-tree最近距离点,-1表示未找到满足要求的点
			pointSearchSurfInd2[i] = minPointInd2;//同一线号(或者)上的距离最近的点，-1表示未找到满足要求的点
			pointSearchSurfInd3[i] = minPointInd3;//不同线号上的距离最近的点，-1表示未找到满足要求的点
		}
		//c.3 迭代不论几次 计算约束
		if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
			tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];//A点
			tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];//B点
			tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];//C点
			//向量AB = (tripod2.x - tripod1.x, tripod2.y - tripod1.y, tripod2.z - tripod1.z)
			//向量AC = (tripod3.x - tripod1.x, tripod3.y - tripod1.y, tripod3.z - tripod1.z)
			//平面的4个参数
			
			// 向量AB AC的向量积（即叉乘），得到的是法向量
			//x轴方向分向量i
			float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
					   - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
			//y轴方向分向量j
			float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
					   - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
			//z轴方向分向量k
			float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
					   - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
			float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);
			//法向量的模
			float ps = sqrt(pa * pa + pb * pb + pc * pc);
			//pa pb pc为法向量单位向量各方向上的分量
			pa /= ps;
			pb /= ps;
			pc /= ps;
			pd /= ps;
			//距离//点到面的距离：向量OA与与法向量的点积除以法向量的模
			float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
			//unused
			pointProj = pointSel;
			pointProj.x -= pa * pd2;
			pointProj.y -= pb * pd2;
			pointProj.z -= pc * pd2;
			
			float s = 1;//同理计算权重
			if (iterCount >= 5) {
				s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
													+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));
			}
			//coeff 偏导数乘以系数(好像不对,不是偏导数) //考虑权重
			coeff.x = s * pa;
			coeff.y = s * pb;
			coeff.z = s * pc;
			coeff.intensity = s * pd2;
			
			if (s > 0.1 && pd2 != 0) {
				laserCloudOri->push_back(surfPointsFlat->points[i]);
				coeffSel->push_back(coeff);
			}
		}
		
	
	}
}

bool lidarOdom::frameOptiLM() {
	//b.3 点面结束 开始迭代 进行lm优化 匹配到的点的个数(即存在多少个约束)
	int pointSelNum = laserCloudOri->points.size();
	if (pointSelNum < 10) {
		return true;//b.4 特征点太少准备满5次 重新选取特征点
	}
	
	Eigen::Matrix<float,Eigen::Dynamic,6> matA(pointSelNum, 6);
	Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,pointSelNum);
	Eigen::Matrix<float,6,6> matAtA;
	Eigen::VectorXf matB(pointSelNum);
	Eigen::Matrix<float,6,1> matAtB;
	Eigen::Matrix<float,6,1> matX;
	
	for (int i = 0; i < pointSelNum; i++) { //pointSelNum 所有约束的集合
		//c.1 准备lm优化 计算matA,matB矩阵
		pointOri = laserCloudOri->points[i];// 当前时刻点坐标
		coeff = coeffSel->points[i];		// 该点所对应的偏导数(系数点了,不是偏导数)
		//pointSelNum是对应有多少个约束
		// 构建Jaccobian矩阵
		/*
			 * 采用Levenberg-Marquardt计算
			 * 首先建立当前时刻Lidar坐标系下提取到的特征点与点到直线/平面
			 * 的约束方程。而后对约束方程求对坐标变换(3旋转+3平移)的偏导
			 * 公式参见论文(2)-(8)
			 */
		float s = 1;
		
		float srx = sin(s * transform.rot_x.value());
		float crx = cos(s * transform.rot_x.value());
		float sry = sin(s * transform.rot_y.value());
		float cry = cos(s * transform.rot_y.value());
		float srz = sin(s * transform.rot_z.value());
		float crz = cos(s * transform.rot_z.value());
		float tx = s * transform.pos.x();
		float ty = s * transform.pos.y();
		float tz = s * transform.pos.z();
		//rotate x
		float arx = (-s*crx*sry*srz*pointOri.x + s*crx*crz*sry*pointOri.y + s*srx*sry*pointOri.z
					 + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
					+ (s*srx*srz*pointOri.x - s*crz*srx*pointOri.y + s*crx*pointOri.z
					   + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
					+ (s*crx*cry*srz*pointOri.x - s*crx*cry*crz*pointOri.y - s*cry*srx*pointOri.z
					   + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;
		
		float ary = ((-s*crz*sry - s*cry*srx*srz)*pointOri.x
					 + (s*cry*crz*srx - s*sry*srz)*pointOri.y - s*crx*cry*pointOri.z
					 + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx)
					 + s*tz*crx*cry) * coeff.x
					+ ((s*cry*crz - s*srx*sry*srz)*pointOri.x
					   + (s*cry*srz + s*crz*srx*sry)*pointOri.y - s*crx*sry*pointOri.z
					   + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry)
					   - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;
		
		float arz = ((-s*cry*srz - s*crz*srx*sry)*pointOri.x + (s*cry*crz - s*srx*sry*srz)*pointOri.y
					 + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
					+ (-s*crx*crz*pointOri.x - s*crx*srz*pointOri.y
					   + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
					+ ((s*cry*crz*srx - s*sry*srz)*pointOri.x + (s*crz*sry + s*cry*srx*srz)*pointOri.y
					   + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;
		//trans x
		float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y
					- s*(crz*sry + cry*srx*srz) * coeff.z;
		
		float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y
					- s*(sry*srz - cry*crz*srx) * coeff.z;
		
		float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;
		
		float d2 = coeff.intensity;
		
		matA(i, 0) = arx;
		matA(i, 1) = ary;
		matA(i, 2) = arz;
		matA(i, 3) = atx;
		matA(i, 4) = aty;
		matA(i, 5) = atz;
		matB(i, 0) = -0.05 * d2;
	}
	matAt = matA.transpose();
	matAtA = matAt * matA;
	matAtB = matAt * matB;
	//b.5 特征值分解 (1)对矩阵没啥要求 (2)小中矩阵较快 (3)大矩阵较慢 (4)精度很高
	matX = matAtA.colPivHouseholderQr().solve(matAtB);
	//b.6 检测是不是第一次迭代
	if (iterCount == 0) {
		Eigen::Matrix<float,1,6> matE;
		Eigen::Matrix<float,6,6> matV;
		Eigen::Matrix<float,6,6> matV2;
		
		Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
		matE = esolver.eigenvalues().real();
		matV = esolver.eigenvectors().real();
		
		matV2 = matV;
		
		isDegenerate = false;
		float eignThre[6] = {10, 10, 10, 10, 10, 10};
		for (int i = 5; i >= 0; i--) {
			if (matE(0, i) < eignThre[i]) {
				for (int j = 0; j < 6; j++) {
					matV2(i, j) = 0;
				}
				isDegenerate = true;//若有特征值 < 10 退化 若有大于等于10 就 跳出
			} else {//特征值太小，则认为处在兼并环境中，发生了退化
				break;
			}
		}
		matP = matV.inverse() * matV2;//计算P矩阵
	}
	//b.7 退化的话 //如果发生退化，只使用预测矩阵P计算
	if (isDegenerate) {
		Eigen::Matrix<float,6,1> matX2;
		matX2 = matX;
		matX = matP * matX2;
	}
	//累加每次迭代的旋转平移量
	transform.rot_x = transform.rot_x.value() + matX(0, 0);
	transform.rot_y = transform.rot_y.value() + matX(1, 0);
	transform.rot_z = transform.rot_z.value() + matX(2, 0);
	transform.pos.x() += matX(3, 0);
	transform.pos.y() += matX(4, 0);
	transform.pos.z() += matX(5, 0);
	//判断是否非数字
	if( std::isnan(transform.rot_x.value()) ) transform.rot_x = Angle();
	if( std::isnan(transform.rot_y.value()) ) transform.rot_y = Angle();
	if( std::isnan(transform.rot_z.value()) ) transform.rot_z = Angle();
	
	if( std::isnan(transform.pos.x()) ) transform.pos.x() = 0.0;
	if( std::isnan(transform.pos.y()) ) transform.pos.y() = 0.0;
	if( std::isnan(transform.pos.z()) ) transform.pos.z() = 0.0;
	
	deltaR = sqrt(//计算旋转平移量，如果很小就停止迭代
			pow(rad2deg(matX(0, 0)), 2) +
			pow(rad2deg(matX(1, 0)), 2) +
			pow(rad2deg(matX(2, 0)), 2));
	deltaT = sqrt(
			pow(matX(3, 0) * 100, 2) +
			pow(matX(4, 0) * 100, 2) +
			pow(matX(5, 0) * 100, 2));
/*	std::cout<<"times: "<<iterCount<<" x: "<<transform.pos.x()<<" y: "<<
			 transform.pos.y()<<" z: "<<transform.pos.z()<<std::endl;
	std::cout<<" x: "<<transform.rot_x.value()<<" y: "<<
			 transform.rot_y.value()<<" z: "<<transform.rot_z.value()<<std::endl;*/
    //收敛的条件
	if (deltaR < 0.1 && deltaT < 0.1) {
		return false;
	}

}
