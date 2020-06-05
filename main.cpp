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
	m.GetFileNames(m.filepath,"pcd");
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
		//1. g2o+pcd拼图
		case 1 :
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
		case 7://7. 从bag中读何塞rawdata
			cout << "read pcd:" << endl;
			m.readAndSaveHesai("/media/echo/DataDisc/9_rosbag/zed_pandar64_ins/Hesai_back_afternoon_2.bag");
			cout << "read pcd finish:" << endl;
			break;
		case 8://8. 测试新写的函数
			m.testFunction();
			break;
		case 9:
			m.rslidarmapping();
			break;
		case 10:
			//10. 格式转化,用于不同的雷达型号 保留ring 和timestamp等信息
			m.readAndSaveHesai("/media/echo/DataDisc/9_rosbag/rsparel_64_ins/2019-11-06-20-43-12_0.bag");
			break;
		case 11:
			//10. 格式转化,用于不同的雷达型号 保留ring 和timestamp等信息
			m.readAndSaveHesai("/media/echo/DataDisc/9_rosbag/rsparel_64_ins/2019-11-06-20-43-12_0.bag");
			break;
		default :
			cout << "无效输入" << endl;
	}
	return(0);
}
