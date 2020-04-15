# mapping
用来生成地图啥的
### 1. 该项目的说明
2019/5/15 ver 1.0
用来测试主流的建图算法 loam lego-loam a-loam 的前端部分的 特征提取 lidar-odom lidar-mapping等部分.

建图模式目前分为三种,

1.g2o+pcd的传统模式(pcd+g2o路径)

2.point to plane ICP (需要提供pcd路径) :
准备用来做一个前端,目前还没实现好,准备试试是libpointmatcher 或者pcl

3.bpreal ground mapping (pcd+g2o路径)
:这个目前通过一个点的左右向量点乘的得到当前的点的曲率,大于一定值认为是边缘

4.LOAM 的建图方法(todo)

5.point to plane icp
11/27 准备完成

### 2. 文件说明
#### 1.pcd_reader 主要是顺序读取pcd 进行测试
#### 2.util 就是各种测试用的工具
#### 3.beam_separate 是loam的特征提取部分
#### 4.FP_PC 是用来滤除动态物体的部分
#### 5.lidar_odom 是loam的lidar odometry
#### 6.mapping 是 loam的 mapping 部分
3.2 设置cube center的位置,不太清楚3.3在干什么
#### 7.lego_Feature 是lego 的 滤波 特征提取部分
#### 8. a_loam_odom 是ceres 优化的lidar odom
#### 9. v-loam(to be continue)
#### 10. Ekf(to be continue)
#### 11. particle filter
#### 12. lmOptimizationSufraceCorner
用来构建mapping 的 corner 和 surface的约束
### 3. 文件夹说明
#### 1. spline
采用se3 上的b样条插值,进行了去畸变
#### 2. g2oIO
进行位姿文件的读取转化
#### 3.GPS 
里面放了GPGGA解析啊, 什么LLA坐标转换之类的功能
#### 4.registration
里面放着最近点云配准的算法
#### 5.DataIO
里面放着各种数据存储和格式, eg: 阅读bag的格式,自己存储位姿的方式,GPGGA的保存等等.

(1) 现在阅读bag的已经完成了
#### 6.Distortion
得到点云线数,以及线性去畸变
#### 7.VO
 using OpenCV calcOpticalFlowPyrLK for feature tracking:
https://github.com/ZhenghaoFei/visual_odom.git
#### 8.registration
这个地方就放普通的icp接口,或者ndt之类的
#### 9.GPS_constraint_mapping
这里用gps构建约束,并且自动读取接近的点云位置,然后自动添加闭环的约束.
#### 10. 6DOfcalib
这里是用来放 6DOF - 3DOF 或者 6DOF-6DOF 基于优化的外参标定的库/eg: IMU-LiDAR/LiDAR-Camera/Camera-IMU
#### imgAddColir2Lidar
这里是将图像投影到点云的相关函数



11/27 准备完成

### 4. Todo
1.1 首先完成 loam 的特征提取调参

1.2 loam 的 mapping 部分完成/ 现在使用 a-loam?

1.3 lego 的Feature extraction 部分的重构

1.4 lego 的 odom 部分 看看如何分成两组优化一个位姿.

2.1 11/16 进行 point to plane icp 的构建,1128 还在写/现在完成了,回头试试 libpointmatcher

3.1 Visual odom 添加这个部分进行测试

4.0 惯导建图过程

4.1 外参标定的过程/惯导(cpt)还是得改下驱动,现在的时间戳不对回头我看一下,现在用size凑合一下

4.2 现在外参标定是3DOF-6DOF 貌似算法不收敛

4.3 现在为了方便地图扩展的话, LLA的坐标拼接的功能可以加一下,就是所有的LLA之间的位姿转换.

5.1 目前的g2o文件有个问题, 就是没有时间戳,现在看看要不要改数据的格式 2020/3/10.

6.1 通过eth的论文的进行去除地图中的动态物体, 需要重构数据格式

7.1 关于建图本身,这个现在有一个问题,就是local map的选取,如果 是采用运动距离大于多少保留关键帧还是?有没有其他1更好的保留关键帧的方法,
比如切割组合这样的,切割成小块再在小块中滤波更新,或者分割出再更新?

8.0 先把移动端的建图设备的外参挑出来

9.0 csv的时候,发现时间有问题,bag转pcd时候检查一下

### 5. 程序说明
#### 1.速腾bperal 的地面边线提取
在util.h中的 tools.GetPointCloudBeam(); 和 tools.GetBeamEdge();
输入的是PointXYZINormal格式的点云,其通道定义为: 
1. normal_y 为 timestamp(每个点的到达时间)
2. GetPointCloudBeam 中返回 normal x 指的是当前的点云的线数.
3. GetBeamEdge 中 intensity 为 其edge显著度: 保留范围为
#### 2.标定LiDAR-GPS外参的基本流程
1. 转换GPS到LLA,并且存为PCD/intensity放关于第一帧的时间戳增量
2. 进行LiDAR的 odom, LiDAR的odom存为G2O格式,添加一个txt或者文档放时间戳和G2o的关系
3. LiDARGNSScalibration 中读取位置,调整并且对齐.
4. 进行优化
### 6. 文件路径及文件夹
1.
