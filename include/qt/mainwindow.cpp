//
// Created by echo on 2020/3/19.
//

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <QFormLayout>
#include <QtGlobal>
#include <QObject>
#include <QSlider>
#include <QSpinBox>
#include <QWidget>
#include <QString>
#include <qdir.h>
#include <qfiledialog.h>

#include <QPainter>
MainWindow::MainWindow(QWidget *parent) :
		QMainWindow(parent),
		ui(new Ui::MainWindow)
{
	ui->setupUi(this);
}

MainWindow::~MainWindow()
{
	delete ui;
}
//1.选择文件夹
void MainWindow::on_pushButton_clicked()
{
	QString srcDirPath = QFileDialog::getExistingDirectory(
			this, "choose PCD Directory",
			"/");
	if (srcDirPath.isEmpty())
	{
		return;
	}
	
	if (!srcDirPath.isEmpty())
		ui->plainTextEdit->appendPlainText(srcDirPath);
	//显示到下面的框里面
	QString message1;
	message1.append("PCD Directory: ");
	message1.append( srcDirPath);
	ui-> textBrowser->append(message1);
	
}
//2. 选择文件
void MainWindow::on_pushButton_2_clicked()
{
	QString curPath=QDir::currentPath();//获取系统当前目录
	//获取应用程序的路径
	QString dlgTitle="choose g2o Document"; //对话框标题
	QString filter="g2o(*.g2o*)"; //文件过滤器
	QString aFileName=QFileDialog::getOpenFileName(this,dlgTitle,curPath,filter);
	if (!aFileName.isEmpty())
		ui->plainTextEdit_2->appendPlainText(aFileName);
	//显示到下面的框里面
	QString message1;
	message1.append("g2o file Directory: ");
	message1.append( aFileName);
	ui-> textBrowser->append(message1);
}
//3.0.起始id
void MainWindow::on_spinBox_valueChanged(int arg1)
{
	QString message;
	message.append("Start PCD ID: ");
	//显示到下面的框里面
	message.append( QString::number(arg1));
	ui-> textBrowser->append(message);

 
	
}
//3建图
void MainWindow::on_mapping_Button_clicked()
{
	QString srcDirPath = QFileDialog::getExistingDirectory(this, "Mapping save g2o path", "/");
	
	if (srcDirPath.isEmpty())
	{
		return;
	}
	
	if (!srcDirPath.isEmpty())
		ui->plainTextEdit_4->appendPlainText(srcDirPath);
	QString message;
	message.append("Mapping save g2o path: ");
	message.append( srcDirPath);
	ui-> textBrowser->append(message);
	//pcd的存放路径
	QString pcdDirPath = QFileDialog::getExistingDirectory(this, "Mapping save PCD path", "/");
	if (pcdDirPath.isEmpty())
	{
		return;
	}
	if (!pcdDirPath.isEmpty())
		ui->plainTextEdit_4->appendPlainText(pcdDirPath);
	QString message1;
	message1.append("Mapping save PCD path: ");
	message1.append( pcdDirPath);
	ui-> textBrowser->append(message1);
	
}
//4.拼图
void MainWindow::on_g2o_mapping_botton_clicked()
{
	QString srcDirPath = QFileDialog::getExistingDirectory(
			this, "PCD Map map save path",
			"/");
	if (srcDirPath.isEmpty())
	{
		return;
	}
	
	if (!srcDirPath.isEmpty())
		ui->plainTextEdit_4->appendPlainText(srcDirPath);
	//显示到下面的提示框中
	QString message;
	message.append("Mapping save PCD path: ");
	message.append( srcDirPath);
	ui-> textBrowser->append(message);
}
//终止ID
void MainWindow::on_spinBox_2_valueChanged(int arg1)
{
	QString message;
	message.append("End PCD ID: ");
	message.append( QString::number(arg1));
	ui-> textBrowser->append(message);
	//ui->textBrowser->moveCursor(QTextCursor::End);
}

void MainWindow::on_pushButton_3_clicked()
{

}


void MainWindow::on_pushButton_4_clicked()
{
	QString srcDirPath = QFileDialog::getExistingDirectory(
			this, "Save converted bag to :",
			"/");
	if (srcDirPath.isEmpty())
	{
		return;
	}
	
	if (!srcDirPath.isEmpty())
		ui->plainTextEdit_4->appendPlainText(srcDirPath);
	//显示到下面的提示框中
	QString message;
	message.append("Save converted bag to Dir: ");
	message.append( srcDirPath);
	ui-> textBrowser->append(message);
	
}
