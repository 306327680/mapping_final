//
// Created by echo on 2020/3/19.
//

#ifndef PCD_COMPARE_MAINWINDOW_H
#define PCD_COMPARE_MAINWINDOW_H


#include <QMainWindow>
#include "otherFunctions.h"

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();
	float x=0,y=0,z=0,roll=0,pitch=0,yaw=0;
	main_function main_functions;
private slots:
	
	void on_pushButton_clicked();
	
	void on_pushButton_2_clicked();
	//标定外参
	void on_pushButton_3_clicked();
	
	void on_spinBox_valueChanged(int arg1);
	
	void on_mapping_Button_clicked();//3建图
	
	void on_g2o_mapping_botton_clicked();
	
	void on_spinBox_2_valueChanged(int arg1);

	void on_pushButton_4_clicked();
	//1.标定用参数

	void on_pushButton_5_clicked(){
		x += 0.0001;//加1mm
		std::cout<<"x+: "<<x<<std::endl;
	};
	void on_pushButton_6_clicked(){
		x -= 0.0001;//加1mm
		std::cout<<"x-: "<<x<<std::endl;
	};
	void on_pushButton_7_clicked(){
		y += 0.0001;//加1mm
		std::cout<<"y+: "<<y<<std::endl;
	};
	void on_pushButton_8_clicked(){
		y -= 0.0001;//加1mm
		std::cout<<"y-: "<<y<<std::endl;
	};
	void on_pushButton_9_clicked(){
		z -= 0.0001;//加1mm
		std::cout<<"z-: "<<z<<std::endl;
	};
	void on_pushButton_10_clicked(){
		z += 0.0001;//加1mm
		std::cout<<"z+: "<<z<<std::endl;
	};
	void on_pushButton_11_clicked(){
		roll += 0.001;//度
		std::cout<<"roll+: "<<roll<<" degree"<<std::endl;
	};
	void on_pushButton_12_clicked(){
		pitch += 0.001;//度
		std::cout<<"pitch+: "<<pitch<<" degree"<<std::endl;
	};
	void on_pushButton_13_clicked(){
		yaw += 0.001;//度
		std::cout<<"yaw+: "<<yaw<<" degree"<<std::endl;
	};
	void on_pushButton_14_clicked(){
		roll -= 0.001;//度
		std::cout<<"roll-: "<<roll<<" degree"<<std::endl;
	};
	void on_pushButton_15_clicked(){
		pitch -= 0.001;//度
		std::cout<<"pitch-: "<<pitch<<" degree"<<std::endl;
	};
	void on_pushButton_16_clicked(){
		yaw -= 0.001;//度
		std::cout<<"yaw-: "<<yaw<<" degree"<<std::endl;
	};
private:
	Ui::MainWindow *ui;
};

#endif //PCD_COMPARE_MAINWINDOW_H
