//
// Created by echo on 19-3-14.
//

#include "beam_separate.h"
void featureExtraction::getCircleCenter(PointTypeSm &p1, PointTypeSm &p2, PointTypeSm &p3,
					 float &x0,float &y0,float &z0,float &r){
	double  k11, k12, k13, k14;
	k11 = (p1.x - p3.y)*(p2.z - p3.z) - (p2.y - p3.y)*(p1.z - p3.z);
	k12 = (p2.x - p3.x)*(p1.z - p3.z) - (p1.x - p3.x)*(p2.z - p3.z);
	k13 = (p1.x - p3.x)*(p2.y - p3.y) - (p2.x - p3.x)*(p1.y - p3.y);
	k14 = -(k11*p3.x + k12*p3.y + p3.z);
	
	double  k21, k22, k23, k24;
	k21 = p2.x - p1.x;
	k22 = p2.y - p1.y;
	k23 = p2.z - p1.z;
	k24 = ((pow(p2.x, 2) - pow(p1.x, 2) + pow(p2.y, 2) - pow(p1.y, 2) + pow(p2.z, 2) - pow(p1.z, 2))) / 2.0;
	
	double  k31, k32, k33, k34;
	k31 = p3.x - p2.x;
	k32 = p3.y - p2.y;
	k33 = p3.z - p2.z;
	k34 = ((pow(p3.x, 2) - pow(p2.x, 2) + pow(p3.y, 2) - pow(p2.y, 2) + pow(p3.z, 2) - pow(p2.z, 2))) / 2.0;
	//求圆心半径
	
	x0 = (float)(-k14*k22*k33 - k24*k32*k13 - k34*k12*k23 + k13*k22*k34 + k33*k12*k24 + k23*k32*k14)
		 / (k11*k22*k33 + k12*k23*k31 + k21*k32*k13 - k13*k22*k31 - k12*k21*k33 - k23*k31*k11);
	y0 = (float)(-k11*k24*k33 - k21*k34*k13 - k14*k23*k31 + k13*k24*k31 + k23*k34*k11 + k14*k21*k33)
		 / (k11*k22*k33 + k12*k23*k31 + k21*k32*k13 - k13*k22*k31 - k12*k21*k33 - k23*k31*k11);
	z0 = (float)(-k11*k22*k34 - k21*k32*k14 - k12*k24*k31 + k14*k22*k31 + k12*k21*k34 + k24*k32*k11)
		 / (k11*k22*k33 + k12*k23*k31 + k21*k32*k13 - k13*k22*k31 - k12*k21*k33 - k23*k31*k11);
	PointTypeSm P0;
	P0.x = x0;
	P0.y = y0;
	P0.z = z0;
	r = 1/(float)sqrt(pow(p1.x - P0.x, 2) + pow(p1.y - P0.y, 2) + (pow(p1.z - P0.z, 2)));
}

//1.特征提取
void featureExtraction::calcFeature(pcl::PointCloud<PointXYZIBS>::Ptr &laserCloud) {
	//todo 内存泄漏
	surfPointsLessFlat->clear();
	cornerPointsSharp->clear();
	cornerPointsLessSharp->clear();
	surfPointsFlat->clear();
	int scanCount = -1;
	float pointWeight = -2 * CURVATURE_REGION;
	//大循环 1 计算每个点的曲度,可以参照公式
	//这个应该没有问题
	for (int i = CURVATURE_REGION; i < laserCloud->size() - CURVATURE_REGION; i++) {
		//todo 会有问题? 2019/5/14
		float diffX = pointWeight * laserCloud->points[i].x;
		float diffY = pointWeight * laserCloud->points[i].y;
		float diffZ = pointWeight * laserCloud->points[i].z;
		
		for (int j = 1; j <= CURVATURE_REGION; j++) {
			//左5 右5 取点 求和
			if(i + j < laserCloud->size()){
				diffX += laserCloud->points[i + j].x + laserCloud->points[i - j].x;
				diffY += laserCloud->points[i + j].y + laserCloud->points[i - j].y;
				diffZ += laserCloud->points[i + j].z + laserCloud->points[i - j].z;
			}
		}
		// i = 0 -5
		//todo 这个算曲率的有问题 应该改进
		cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
		cloudSortInd[i] = i;
		cloudNeighborPicked[i] = 0;
		cloudLabel[i] = SURFACE_LESS_FLAT;
		
		if (int(laserCloud->points[i].beam) != scanCount) {//排除线束跳边得到扫描起始停止点
			scanCount = int(laserCloud->points[i].beam);//更新线束数
			if (scanCount > 0 && scanCount < N_SCAN) {
				scanStartInd[scanCount] = i + CURVATURE_REGION;
				//cout<<scanStartInd[scanCount]<<endl;
				scanEndInd[scanCount - 1] = i - CURVATURE_REGION;
			}
		}
	}
	//重新设置首尾的大小
	scanStartInd[0] = CURVATURE_REGION;
	scanEndInd.back() = (int)laserCloud->size() - CURVATURE_REGION;
	//1.1算下真正的半径(实验用)
/*	pcl::PointCloud<pcl::PointXYZ> center;
	center.resize(laserCloud->size());
	for (int i = 1; i < laserCloud->size() - 1; i++) {
		getCircleCenter(laserCloud->points[i -1],laserCloud->points[i],laserCloud->points[i + 1]
				,center[i].x,center[i].y,center[i].z,laserCloud->points[i].curvature);
	}
	writer_pcd.write("/home/echo/center.pcd",center);*/
	
	//循环所有点 大循环 2****************
	//2. 选择靠谱点
	
	for (int i = CURVATURE_REGION; i < laserCloud->size() - CURVATURE_REGION - 1; i++) {
		float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
		float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
		float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
		float diff = X_INDEX * diffX *diffX +
					 Y_INDEX * diffY * diffY +
					 Z_INDEX * diffZ * diffZ;
		//前后的点差的太多的话
		if (diff > 0.1) {
			
			float depth1 = laserCloud->points[i].range;
			float depth2 = laserCloud->points[i+1].range;
			
			if (depth1 > depth2) {
				diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
				diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
				diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;
				
				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
					for (int j = 0; j <= CURVATURE_REGION; j++) {
						cloudNeighborPicked[i - j] = 1;
						laserCloud->points[i - j].NeighborPicked = 1;
					}
				}
			} else {
				diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
				diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
				diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;
				
				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
					for (int j = CURVATURE_REGION + 1; j > 0 ; j--) {
						cloudNeighborPicked[i + j] = 1;
						laserCloud->points[i + j].NeighborPicked = 1;
					}
				}
			}
		}
		
		float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
		float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
		float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
		float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
		
		float dis = laserCloud->points[i].range * laserCloud->points[i].range;
		//右边的点比当前的点变化大的话 pick 感觉这个是对比较远的点的.
		if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
			cloudNeighborPicked[i] = 1;
			laserCloud->points[i].NeighborPicked = 1;
		}
	}
	//3 第三大部分
	//3. 分离特征点 感觉这里不是很对
	
	for (int i = 0; i < N_SCAN; i++) {
		pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
		for (int j = 0; j < N_FEATURE_REGIONS; j++) {
			//子区域中开始和结束的 计算每个子区域中的开始结束点?
			int sp = (scanStartInd[i] * (N_FEATURE_REGIONS - j) + scanEndInd[i] * j) / N_FEATURE_REGIONS;
			int ep = (scanStartInd[i] * (N_FEATURE_REGIONS - 1 - j) + scanEndInd[i] * (j + 1)) / N_FEATURE_REGIONS - 1;
			//好像没问题
			
			for (int k = sp + 1; k <= ep; k++) {
				for (int l = k; l >= sp + 1; l--) {
					//当前的曲率比t-1小 cloudSortInd是 每个点的序号 每一个区域 排序一遍 交换排序 冒泡排序
					if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
						int temp = cloudSortInd[l - 1];
						cloudSortInd[l - 1] = cloudSortInd[l];
						cloudSortInd[l] = temp;
//						排序曲率
					}
				}
			}
			
			int largestPickedNum = 0;
			for (int k = ep; k >= sp; k--) {//最大的cloudSortInd放最大的曲率
				//判断是平面的点 是pick的 点 并且曲率大于阈值
				int ind = cloudSortInd[k]; //最大的曲率 的 index
				if (cloudNeighborPicked[ind] == 0 &&
					cloudCurvature[ind] > SURFACE_CURVATURE_THRESHOLD) {//没被pick(满足条件)并且大于平面去了阈值
					
					largestPickedNum++;//统计现在 大于阈值的点的个数
					if (largestPickedNum <= MAX_CORNER_SHARP) {
						//一个区域选两个最尖锐的
						cloudLabel[ind] = CORNER_SHARP;
						cornerPointsSharp->push_back(laserCloud->points[ind]);
						laserCloud->points[ind].smooth = 5;
						/*cornerPointsLessSharp->push_back(laserCloud->points[ind]);
						laserCloud->points[ind].smooth = 10;*/
					} else if (largestPickedNum <= MAX_CORNER_LESS_SHARP) {
						cloudLabel[ind] = CORNER_LESS_SHARP;
						cornerPointsLessSharp->push_back(laserCloud->points[ind]);
						laserCloud->points[ind].smooth = 10;
					} else {
						break;
					}
					
					cloudNeighborPicked[ind] = 1;
					laserCloud->points[ind].NeighborPicked = 1;
					for (int l = 1; l <= CURVATURE_REGION; l++) {
						float diffX = laserCloud->points[ind + l].x
									  - laserCloud->points[ind + l - 1].x;
						float diffY = laserCloud->points[ind + l].y
									  - laserCloud->points[ind + l - 1].y;
						float diffZ = laserCloud->points[ind + l].z
									  - laserCloud->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}
						
						cloudNeighborPicked[ind + l] = 1;
						laserCloud->points[ind + l].NeighborPicked = 1;
					}
					for (int l = -1; l >= -CURVATURE_REGION; l--) {
						float diffX = laserCloud->points[ind + l].x
									  - laserCloud->points[ind + l + 1].x;
						float diffY = laserCloud->points[ind + l].y
									  - laserCloud->points[ind + l + 1].y;
						float diffZ = laserCloud->points[ind + l].z
									  - laserCloud->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}
						
						cloudNeighborPicked[ind + l] = 1;
						laserCloud->points[ind + l].NeighborPicked = 1;
					}
				}
			}
			
			int smallestPickedNum = 0;
			for (int k = sp; k <= ep; k++) {
				int ind = cloudSortInd[k];
				//判断是边的点 pick的点 并且小于阈值
				if (cloudNeighborPicked[ind] == 0 &&
					cloudCurvature[ind] < SURFACE_CURVATURE_THRESHOLD) {
					cloudLabel[ind] = SURFACE_FLAT;
					surfPointsFlat->push_back(laserCloud->points[ind]);
					laserCloud->points[ind].smooth = 20;
					smallestPickedNum++;
					if (smallestPickedNum >= MAX_SURFACE_FLAT) {
						break;
					}
					
					cloudNeighborPicked[ind] = 1;
					laserCloud->points[ind].NeighborPicked = 1;
					for (int l = 1; l <= CURVATURE_REGION; l++) {
						float diffX = laserCloud->points[ind + l].x
									  - laserCloud->points[ind + l - 1].x;
						float diffY = laserCloud->points[ind + l].y
									  - laserCloud->points[ind + l - 1].y;
						float diffZ = laserCloud->points[ind + l].z
									  - laserCloud->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}
						
						cloudNeighborPicked[ind + l] = 1;
						laserCloud->points[ind + l].NeighborPicked = 1;
					}
					for (int l = -1; l >= -CURVATURE_REGION; l--) {//1
						if( (ind + l)>=0 &&(ind + l + 1)<laserCloud->size()){
							float diffX = laserCloud->points[ind + l].x
										  - laserCloud->points[ind + l + 1].x;
							float diffY = laserCloud->points[ind + l].y
										  - laserCloud->points[ind + l + 1].y;
							float diffZ = laserCloud->points[ind + l].z
										  - laserCloud->points[ind + l + 1].z;
							if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
								break;
							}
							
							cloudNeighborPicked[ind + l] = 1;
							laserCloud->points[ind + l].NeighborPicked = 1;
						}
					}
				}
			}
			
			for (int k = sp; k <= ep; k++) {
				if (cloudLabel[k] == SURFACE_LESS_FLAT) {//收集全部的平面的点
					//基本就是其他点了
					surfPointsLessFlatScan->push_back(laserCloud->points[k]);
					laserCloud->points[k].smooth = 15;
				}
			}
		}
		pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
		pcl::VoxelGrid<PointType> downSizeFilter;
		downSizeFilter.setInputCloud(surfPointsLessFlatScan);
		downSizeFilter.setLeafSize(LESS_FLAT_FILTER_SIZE, LESS_FLAT_FILTER_SIZE, LESS_FLAT_FILTER_SIZE);
		downSizeFilter.filter(surfPointsLessFlatScanDS);
		*surfPointsLessFlat = surfPointsLessFlatScanDS;
	}
}

//1.2 重新特征提取
void featureExtraction::calcFeature_mine(pcl::PointCloud<PointTypeSm>::Ptr &laserCloud) {

}



void featureExtraction::calculateSmoothness(pcl::PointCloud<PointTypeBeam>::Ptr segmentedCloud_bef,
											pcl::PointCloud<PointTypeSm>::Ptr &segmentedCloud) {
	int cloudSize = segmentedCloud_bef->points.size();
	segmentedCloud->clear();
	segmentedCloud->resize(cloudSize);
	for (int i = 0; i < cloudSize; i++) {
		segmentedCloud->points[i].x = segmentedCloud_bef->points[i].x;
		segmentedCloud->points[i].y = segmentedCloud_bef->points[i].y;
		segmentedCloud->points[i].z = segmentedCloud_bef->points[i].z;
		segmentedCloud->points[i].intensity = segmentedCloud_bef->points[i].intensity;
		segmentedCloud->points[i].beam = segmentedCloud_bef->points[i].beam;
		segmentedCloud->points[i].range = segmentedCloud_bef->points[i].range;
		segmentedCloud->points[i].pctime = segmentedCloud_bef->points[i].pctime;
	}
	
	for (int i = 5; i < cloudSize - 5; i++) {
		float diffRange = 0;
		diffRange = segmentedCloud->points[i-5].range + segmentedCloud->points[i-4].range
					+ segmentedCloud->points[i-3].range + segmentedCloud->points[i-2].range
					+ segmentedCloud->points[i-1].range - segmentedCloud->points[i].range * 10
					+ segmentedCloud->points[i+1].range + segmentedCloud->points[i+2].range
					+ segmentedCloud->points[i+3].range + segmentedCloud->points[i+4].range
					+ segmentedCloud->points[i+5].range;
	}
	
}

bool featureExtraction::GetFileNames(const std::string directory, const std::string suffix,
									 std::vector<std::string> &file_names_) {
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
		std::cout<<"**GetFileNames Get PCD number is : "<<file_names_.size()<<std::endl;
		return true;
}

void
featureExtraction::copyPCD(pcl::PointCloud<pcl::PointXYZI>::
        Ptr cloud_bef, pcl::PointCloud<PointTypeBeam>::Ptr &test) {
	test->points.resize(cloud_bef->size());
	for (size_t i = 0; i < cloud_bef->points.size(); i++) {
		test->points[i].x = cloud_bef->points[i].x;
		test->points[i].y = cloud_bef->points[i].y;
		test->points[i].z = cloud_bef->points[i].z;
		test->points[i].intensity = cloud_bef->points[i].intensity;
	}
}

void featureExtraction::checkorder(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef,
								   pcl::PointCloud<PointTypeBeam>::Ptr out) {
	//--声明变量区
	double verticalAngle, horizonAngle, range, beam_min, beam_max;
	//用来存放每个beam的最大index 最小index
	std::vector<int> max_index(N_SCAN);
	std::vector<int> min_index(N_SCAN);
	PointTypeBeam thisPoint;
	float rowIdn, columnIdn, index, cloudSize;
	//--变量声明完毕
	out->points.resize(cloud_bef->size());
	
	//a.0 计算旋转区间
	float startOrientation = 0;
	float endOrientation = 0;
	if(out->size()>0){
		startOrientation = (float)-atan2(out->points[0].y, out->points[0].x);
		//todo 这个计算有问题>>>>
		endOrientation   = (float)-atan2(out->points.back().y,
										 out->points.back().x) + 2 * M_PI;
		if (endOrientation - startOrientation > 3 * M_PI) {
			endOrientation -= 2 * M_PI;
		} else if (endOrientation - startOrientation < M_PI)
			endOrientation += 2 * M_PI;
	} else{
		std::cerr<<"wrong"<<std::endl;
	}
	
	
	//a. 第一个for lego的确定beam的函数
	for (size_t i = 1; i < out->size(); ++i){
		out->points[i].x = cloud_bef->points[i].x;
		out->points[i].y = cloud_bef->points[i].y;
		out->points[i].z = cloud_bef->points[i].z;
		out->points[i].intensity = cloud_bef->points[i].intensity;
		thisPoint.x = out->points[i].x;
		thisPoint.y = out->points[i].y;
		thisPoint.z = out->points[i].z;
		verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y))
				* 180 / M_PI;
		rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
		if (rowIdn < 0 || rowIdn >= N_SCAN)
			continue;
		horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
		columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
		if (columnIdn >= Horizon_SCAN)
			columnIdn -= Horizon_SCAN;
		if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
			continue;
		range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
		//lego加的什么鬼
		out->points[i].beam = (double)rowIdn ;//+ (double)columnIdn / 10000.0;
		out->points[i].beam = (int)(out->points[i].beam/Div);
		out->points[i].range = range;
		//找到最大的开始结束点
		if(out->points[i].beam != out->points[i-1].beam){
			out->points[i].range = -100;
			out->points[i -1].range = 200;
		
			float startOrientation_new = (float)-atan2(out->points[i].y, out->points[i].x);
			float endOrientation_new   = static_cast<float>(-atan2(out->points[i - 1].y,
																   out->points[i - 1].x)
															+ 2 * M_PI);
			if (endOrientation_new - startOrientation_new > 3 * M_PI) {
				endOrientation_new -= 2 * M_PI;
			} else if (endOrientation_new - startOrientation_new < M_PI)
				endOrientation_new += 2 * M_PI;
			if(startOrientation_new<startOrientation){
				startOrientation = startOrientation_new;
			}
			if(endOrientation_new > endOrientation){
				endOrientation = endOrientation_new;
			}
		}
	}//确认beam结束
	//b. 计算旋转时间
	int flip = 1;
	bool smaller = 0 ,bigger = 0;
	
	//可以在下面计算时间补偿了
	//当end大于360的时候,条件很难满足,
	//todo 判断当前的 一直大于大于当前? 小于其实就翻转 diff取最大? 或者averange
	//todo 感觉还是有问题的, 现在是尽量避免了
	/*if(orientationDiff < endOrientation - startOrientation){
		orientationDiff = endOrientation - startOrientation;
		std::cout<<orientationDiff<<std::endl;
	}*/
	orientationDiff = endOrientation - startOrientation;
	//第五个for
	for (size_t i = 1; i < cloud_bef->points.size(); i++) {
		float last_angle = (float)-atan2(out->points[i-1].y, out->points[i-1].x); //前一个角度
		float cur_angle = (float)-atan2(out->points[i].y, out->points[i].x); //当前角度
		//(1)检查线束跳变(左右)
		//跳变情况1. 换线 2. 噪音
		if(out->points[i].beam == out->points[i-1].beam){
			//(2)检查角度跳变
			out->points[i].pctime = cur_angle - startOrientation;//起始点调整为0
			if(out->points[i].pctime < 0){
				out->points[i].pctime = out->points[i].pctime + 2*M_PI;
			}
			//分割线 过了起点了转了360度了
			if(out->points[i].pctime < out->points[i-1].pctime && !smaller){
				smaller = 1;
			}
			if(smaller){
				if(out->points[i].pctime<endOrientation){
					out->points[i].pctime = out->points[i].pctime+ 2*M_PI;
				}
			}
			//分割线
		}else{
			//设置新的起点
			//如果起始点是 2pi以外就有问题了, 不过起始点在那里也只能通过 序号很大来判断, 那时候点也很少了
			smaller = 0;
			out->points[i].pctime = cur_angle - startOrientation; //-3.1381 -
		}
	}
	//todo这个整合进去
	for (size_t i = 1; i < cloud_bef->points.size(); i++) {
		out->points[i].pctime =  out->points[i].pctime / orientationDiff;
		//防止时间越界
		if(out->points[i].pctime>1){
			if(out->points[i].pctime>1.0001){
				/*std::cout<<out->points[i].pctime<<" >> " <<i<<std::endl;*/
			}
			out->points[i].pctime = 1;
		}else if(out->points[i].pctime<0){
			out->points[i].pctime = 0;
		}
	}
	out->width = 1;
	out->height = out->points.size();
}

void featureExtraction::rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion) {
	
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
	// eg: 旋转矩阵转为四元数
	//  q = Eigen::Quaterniond(R);
	// eg: Eigen 库中四元数转化为旋转矩阵，
	//	 Eigen::Matrix3d R = Eigen::Matrix3d(q);
	//	 cout << "R=\n" << R << endl << endl;
	}

void featureExtraction::qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r,
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

void featureExtraction::trinterp(Eigen::Matrix4d &T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T) {
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

void featureExtraction::quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R) {
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

void featureExtraction::adjustDistortion(pcl::PointCloud<PointTypeBeam>::Ptr pointIn,
										 pcl::PointCloud<PointTypeBeam>::Ptr &pointOut,
										 Eigen::Isometry3d transform) {
	pointOut->clear();
 
	std::vector<pcl::PointCloud<PointTypeBeam> > laserCloudScans_N(N_SCAN);
	Eigen::Matrix4d transInput,out,input;
	Eigen::Isometry3d transpc;
	input = Eigen::Matrix4d::Identity();//起始为0
	transInput.matrix() = transform.matrix();//终止为1
	pcl::PointCloud<PointTypeBeam>::Ptr temp(new pcl::PointCloud<PointTypeBeam>);
	pcl::PointCloud<PointTypeBeam>::Ptr Individual(new pcl::PointCloud<PointTypeBeam>);
	pcl::PointCloud<PointTypeBeam>::Ptr Individual_bef(new pcl::PointCloud<PointTypeBeam>);
	temp->resize(pointIn->size());
	for(int i = 0; i < pointIn->size(); i++){
		Individual->resize(1);
		Individual_bef->resize(1);
		Individual_bef->points[0] = pointIn->points[i];
		Individual->points[0] = pointIn->points[i];
		trinterp(input, transInput,pointIn->points[i].pctime,out);
		transpc.matrix() = out.matrix();
		Eigen::Matrix4d convert;
		convert = transpc.matrix();
		convert = transpc.matrix().inverse();

		pcl::transformPointCloud(*Individual_bef, *Individual, convert);
		temp->points[i] = pointIn->points[i];
		temp->points[i].x = Individual->points[0].x;
		temp->points[i].y = Individual->points[0].y;
		temp->points[i].z = Individual->points[0].z;
		if(pointIn->points[i].beam >= 0 && pointIn->points[i].beam <= N_SCAN){
			laserCloudScans_N[pointIn->points[i].beam].push_back(temp->points[i]);
		}
	}
		
	for (int i = 0; i < N_SCAN; i++) {
		*pointOut += laserCloudScans_N[i];
	}
}

PointCloud_T featureExtraction::ICP(const PointCloud_T &cloud_source, const PointCloud_T &cloud_target,
									Eigen::Isometry3d &icp_matrix, double distance){
	PointCloud_T result;
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp; //创建ICP对象，用于ICP配准
	//gicp.setRotationEpsilon()
	pcl::IterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI>icp;
	icp.setInputSource(PointCloudConstPtr_T(new PointCloud_T(cloud_source)));
	icp.setInputTarget(PointCloudConstPtr_T(new PointCloud_T(cloud_target)));//InputTarget不动的
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(1e-10);
	icp.setMaximumIterations(distance);
	icp.align(result);
	icp_matrix = icp.getFinalTransformation().cast<double>();
	return result; // source上应用这个矩阵，就可以转过去了
}

PointCloud_T featureExtraction::GICP(const PointCloud_T &cloud_source, const PointCloud_T &cloud_target,
									 Eigen::Isometry3d &icp_matrix, double distance) {
	PointCloud_T result;
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp; //创建ICP对象，用于ICP配准
	//gicp.setRotationEpsilon()
	
	gicp.setTransformationEpsilon(0.01);
	gicp.setMaximumIterations (100);
	gicp.setMaxCorrespondenceDistance(distance);
	gicp.setInputSource(PointCloudConstPtr_T(new PointCloud_T(cloud_source)));
	gicp.setInputTarget(PointCloudConstPtr_T(new PointCloud_T(cloud_target)));
	gicp.align(result);
	Eigen::Matrix4f tf_s2t = gicp.getFinalTransformation();
	icp_matrix = tf_s2t.cast<double>();
	std::cout<<tf_s2t<<std::endl;
	return result; // source上应用这个矩阵，就可以转过去了
}

void featureExtraction::SplitString(const std::string &s, std::vector<std::string> &v, const std::string &c) {
	std::string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while(std::string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2-pos1));
		
		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if(pos1 != s.length())
		v.push_back(s.substr(pos1));
}


