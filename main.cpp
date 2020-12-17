// written by echo
//


#include "qt/mainwindow.h"
#include <QApplication>
//G2O_USE_TYPE_GROUP(slam2d);
//生成所有的特征地图

//intensity 的 edge 可不可以检测的到
int main(int argc,char** argv){
	main_function m;
	//todo 这里可以去掉ros
	ros::init(argc, argv, "map");
	//获得参数
	m.getParam(argc,argv);
	//得到所有的pcd名字
	m.GetIntFileNames(argv[1],"pcd");
	//读取png 彩色图
//	m.GetPNGFileNames("/media/echo/DataDisc2/shandong/pic","png");
	//qt
	
/*	QApplication a(argc, argv);
	MainWindow w;
	w.main_functions.getParam(argc,argv);
	w.main_functions.GetFileNames(w.main_functions.filepath,"pcd");
	w.show();
	return a.exec();*/
	//
	
	switch(m.status)
	{
		//0. VLP+IMU+RTK 存成相应的文件
		case 0:
			m.readAndSaveHesai("/media/echo/DataDisc/9_rosbag/zed_pandar64_ins/Hesai_back_afternoon_2.bag");
			break;
			//-1 建立lidar odom
		case -1:
			m.setStartEnd();
			m.point2planeICP();//普通点面icp
			cout << "point to plane ICP finish!" << endl;
			break;
			//-2 自动闭环
		case -2:
			m.lc.autoMaticLoopClosure("/home/echo/shandong_ceshichang/test.g2o","ss","/media/echo/DataDisc2/shandong/pcd",
 				"/home/echo/shandong_ceshichang/test.csv","/home/echo/shandong_ceshichang/LiDAR_pose.csv");
			cout << "point to plane ICP finish!" << endl;
			break;
			//-3 生成彩色地图
		case -3:
			m.GetPNGFileNames("/home/echo/2_bag/2_ziboHandHold/qisheng/left","png");
			m.GetIntFileNames("/home/echo/2_bag/2_ziboHandHold/qisheng/pcd","pcd");
			m.trans_vector = m.getEigenPoseFromg2oFile("/home/echo/2_bag/2_ziboHandHold/qisheng/g2o/LiDAR_Odom.g2o");
			m.start_id = 400;
			m.end_id = 18500;
			m.filepath = "/home/echo/2_bag/2_ziboHandHold/qisheng/pcd";
			m.genColormap(m.trans_vector,"/home/echo/2_bag/2_ziboHandHold/GO/ex_params3.txt"); //5.1 带颜色的pcd
			break;
			
		case 1 :
			//todo 没修好 段错误
			m.setStartEnd();
			m.g2omapping();
			cout << "g2o mapping start！" << endl;
			break;
		case 2 :
			//2. 对于何塞雷达的建图 需要有时间戳 和线数信息
			m.setStartEnd();
			cout << "point to plane ICP start:" << endl;
			//读bag的功能测试ok
			//readAndSaveHesai("/media/echo/DataDisc/9_rosbag/rsparel_64_ins/2019-11-06-20-43-12_0.bag");
			m.point2planeICP();//普通点面icp
			cout << "point to plane ICP finish!" << endl;
			break;
		case 3 ://3. 可通行区域建图
			m.traversableMapping();
			cout << "traversable Mapping finish!" << endl;
			break;
		case 4 ://4. 利用编码器建图
			m.encoderMapping();
			break;
		case 5://5. Calibrating the extrinsic parameters
			m.LiDARGNSScalibration("/home/echo/small_fov.g2o","/media/echo/DataDisc/3_program/mapping/cmake-build-debug/gnss.pcd");
			m.LiDARGNSSMapping();
			break;
		case 6: //6. ndt建图
			m.setStartEnd();
			m.NDTmapping();
			break;
		case 7://7. 从bag中读bag 的 raw data(包括vlp等)
			cout << "read pcd:" << endl;
			m.readAndSaveHesai("/media/echo/DataDisc/9_rosbag/11_shandongchechang/udist.bag");
			cout << "read pcd finish:" << endl;
			break;
		case 8://8. 测试新写的函数
			m.testFunction();
			break;
			//imu建图
		case 9:
			m.setStartEnd();
			m.getStereoFileNames("/home/echo/2_bag/2_ziboHandHold/qisheng");
		    m.GetPNGFileNames("/home/echo/2_bag/2_ziboHandHold/qisheng/left","png");
	/*		m.start_id = 4200;
			m.end_id = 4350;*/
			m.IMUMapping("/home/echo/2_bag/2_ziboHandHold/qisheng/imu/imu.csv","/home/echo/2_bag/2_ziboHandHold/qisheng/pcd");
			break;
		case 10:
			//10. q去畸变
			m.cameraDistortion( "/home/echo/5_png/out3loop/left_png","/home/echo/5_png/out3loop/left_png_undist/","/home/echo/5_png/out3loop/camera_left.txt");
			break;
		case 11:
			//11. 格式转化,用于不同的雷达型号 保留ring 和timestamp等信息
			m.readAndSaveHesai("/media/echo/DataDisc/9_rosbag/rsparel_64_ins/2019-11-06-20-43-12_0.bag");
			break;
		case 12://带tracking 的 icp
			//11. 格式转化,用于不同的雷达型号 保留ring 和timestamp等信息
			m.setStartEnd();
			m.lidarOdomWithTracking();//普通点面icp
			break;
		case 13:
			m.PCmap2GridMap("/media/echo/DataDisc/5_map/fsk_19/map.pcd");
			break;
		default :
			cout << "无效输入" << endl;
	}
	return(0);
}
