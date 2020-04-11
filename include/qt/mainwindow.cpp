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
//1.选择pcd文件夹
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
	main_functions.filepath = srcDirPath.toStdString();
	QString message1;
	message1.append("PCD Directory: ");
	message1.append( QString::fromStdString(main_functions.filepath));
	ui-> textBrowser->append(message1);
	main_functions.GetFileNames(main_functions.filepath,"pcd");
	message1.clear();
	message1.append("Directory has : ");
	message1.append(QString::number(main_functions.file_names_.size()));
	message1.append(" PCDs ");
	ui-> textBrowser->append(message1);
}
//2. 选择g2o文件
void MainWindow::on_pushButton_2_clicked()
{
	QString curPath=QDir::currentPath();//获取系统当前目录
	//获取应用程序的路径
	QString dlgTitle="choose g2o Document"; //对话框标题
	QString filter="g2o(*.g2o*)"; //文件过滤器
	QString aFileName=QFileDialog::getOpenFileName(this,dlgTitle,curPath,filter);
	if (!aFileName.isEmpty())
		ui->plainTextEdit_2->appendPlainText(aFileName);
	
	main_functions.g2o_path = aFileName.toStdString();
	//显示到下面的框里面
	QString message1;
	message1.append("g2o file Directory: ");
	message1.append( QString::fromStdString(main_functions.g2o_path));
	ui-> textBrowser->append(message1);
	//读取g2o文件到 vector中;
	main_functions.trans_vector = main_functions.getEigenPoseFromg2oFile(main_functions.g2o_path);
	//显示g2o文件的数目
	message1.clear();
	message1.append("g2o file size: ");
	message1.append( QString::number(main_functions.trans_vector.size()));
	ui-> textBrowser->append(message1);
}
//3.0.起始id
void MainWindow::on_spinBox_valueChanged(int arg1)
{

	//赋值
	main_functions.start_id = arg1;
	//显示到下面的框里面
	QString message;
	message.append("Start PCD ID: ");
	message.append( QString::number(main_functions.start_id));
	ui-> textBrowser->append(message);
	
}
//3.1 终止ID
void MainWindow::on_spinBox_2_valueChanged(int arg1)
{
	//赋值
	main_functions.end_id = arg1;
	//显示到下面的框里面
	QString message;
	message.append("End PCD ID: ");
	message.append( QString::number(arg1));
	ui-> textBrowser->append(message);
	//ui->textBrowser->moveCursor(QTextCursor::End);
}

//3建图
void MainWindow::on_mapping_Button_clicked()
{
	//获取现在的pcd路径
	QString srcDirPath = QFileDialog::getExistingDirectory(this, "Mapping save pcd and g2o path", "/");
	
	if (srcDirPath.isEmpty())
	{
		return;
	}
	//g2o存放路径
	if (!srcDirPath.isEmpty())
		ui->plainTextEdit_4->appendPlainText(srcDirPath);
	QString message;
	message.append("Mapping save pcd and g2o path: ");
	message.append( srcDirPath);
	ui-> textBrowser->append(message);
	//重新构建g2o
	std::stringstream g2o_save;
	g2o_save<<srcDirPath.toStdString()<<"/result.g2o";
	main_functions.save_g2o_path = g2o_save.str();
	
	std::stringstream pcd_save;
	pcd_save<<srcDirPath.toStdString()<<"/map.pcd";
	main_functions.save_pcd_path = pcd_save.str();
	
	//开始建图
	main_functions.GetFileNames(main_functions.filepath,"pcd");//读取pcd文件
	main_functions.point2planeICP();//普通点面icp
	
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
//标定外参
void MainWindow::on_pushButton_3_clicked()
{
	//获取bag 的路径
	QString curPath=QDir::currentPath();//获取系统当前目录
	//获取应用程序的路径
	QString dlgTitle="Choose LiDAR pose file"; //对话框标题
	QString filter="g2o(*.g2o*)"; //文件过滤器
	QString aFileName=QFileDialog::getOpenFileName(this,dlgTitle,curPath,filter);
	
	if (!aFileName.isEmpty())
		ui->plainTextEdit_2->appendPlainText(aFileName);
	main_functions.g2o_path = aFileName.toStdString();
	
	//获取pcd的路径
	dlgTitle="Choose GPS PCD file";
	filter="pcd(*.pcd*)";
	QString srcDirPath = QFileDialog::getOpenFileName(this,dlgTitle,curPath,filter);
	
 
	ui->plainTextEdit_4->appendPlainText(srcDirPath);
	//显示到下面的提示框中
	QString message;
	message.append("Save converted bag to Dir: ");
	message.append( srcDirPath);
	ui-> textBrowser->append(message);
	main_functions.LiDARGNSScalibration(aFileName.toStdString(),srcDirPath.toStdString());
}

//bag 转换格式
void MainWindow::on_pushButton_4_clicked()
{
	//获取bag 的路径
	QString curPath=QDir::currentPath();//获取系统当前目录
	//获取应用程序的路径
	QString dlgTitle="choose ros bag"; //对话框标题
	QString filter="bag(*.bag*)"; //文件过滤器
	QString aFileName=QFileDialog::getOpenFileName(this,dlgTitle,curPath,filter);
	if (!aFileName.isEmpty())
		ui->plainTextEdit_2->appendPlainText(aFileName);
	
	main_functions.g2o_path = aFileName.toStdString();
	
	//获取pcd的路径
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
	
	ReadBag a;
	a.readVLP16(aFileName.toStdString(),srcDirPath.toStdString());
}
