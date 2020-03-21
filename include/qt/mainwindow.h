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
	main_function main_functions;
private slots:
	
	void on_pushButton_clicked();
	
	void on_pushButton_2_clicked();
	
	void on_pushButton_3_clicked();
	
	void on_spinBox_valueChanged(int arg1);
	
	void on_mapping_Button_clicked();
	
	void on_g2o_mapping_botton_clicked();
	
	void on_spinBox_2_valueChanged(int arg1);
	
	void on_pushButton_4_clicked();

private:
	Ui::MainWindow *ui;
};

#endif //PCD_COMPARE_MAINWINDOW_H
