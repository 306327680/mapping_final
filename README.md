# mapping
用来生成地图啥的
###1. 该项目的说明
2019/5/15 ver 1.0
用来测试主流的建图算法 loam lego-loam a-loam 的前端部分的 特征提取 lidar-odom lidar-mapping等部分.
###2. 文件说明
####1.pcd_reader 主要是顺序读取pcd 进行测试
####2.util 就是各种测试用的工具
####3.beam_separate 是loam的特征提取部分
####4.FP_PC 是用来滤除动态物体的部分
####5.lidar_odom 是loam的lidar odometry
####6.mapping 是 loam的 mapping 部分
3.2 设置cube center的位置,不太清楚3.3在干什么
####7.lego_Feature 是lego 的 滤波 特征提取部分
####8. a_loam_odom 是ceres 优化的lidar odom
####9. v-loam(to be continue)
####10. Ekf(to be continue)
####11. particle filter
####12. lmOptimizationSufraceCorner
用来构建mapping 的 corner 和 surface的约束
###3. Todo
1. 首先完成 loam 的特征提取调参
2. loam 的 mapping 部分完成
3. lego 的Feature extraction 部分的重构
4. lego 的 odom 部分 看看如何分成两组优化一个位姿.
