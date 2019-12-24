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

11/27 准备完成

### 4. Todo
1. 首先完成 loam 的特征提取调参
2. loam 的 mapping 部分完成
3. lego 的Feature extraction 部分的重构
4. lego 的 odom 部分 看看如何分成两组优化一个位姿.
5. 11/16 进行 point to plane icp 的构建,1128 还在写
6. Visual odom
7. 还是得改下驱动,现在的时间戳不对回头我看一下,现在用size凑合一下
