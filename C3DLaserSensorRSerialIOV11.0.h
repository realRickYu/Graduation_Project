/*! 
 *	\section About 关于摆动式线激光视觉传感器的IO类
 *
 *  \section Announce 说明该类主要针对摆动式线激光视觉传感器的输入输出编写的接口
 *  
 *  \section Log 日志

 *\n 20150907 
     1.数据的读入类型全部为double类型，数据的写入全部为float类型;
	 2.数据的计算全部采用double类型;
	 3.

 * \n 20150701 V11.0
    1. 所有的函数输入Mat类型参数均转换成float类型，返回均为float类型(仅限于标定参数);
	由于不同类之间会出现输出日志共用的现象，故在标定线程中的标定类中所需要的读写函数重新，主要是里面的日志会共用，
	标定日志单独写在一个文件中;
	2. 添加WriteLogCalib函数,将在标定线程中需要输出的日志，单独写在一个文件中;
	3. 添加标定所需的读取文件函数  ReadInitialParametersCalib、 WriteInitialParametersCalib、
	4. 添加EyeToBase()函数

 * \n 20150630 V10.0 增加系统标定所需的读取参数操作(读写标定参数)
      1.将所需的读入和写入的测量参数的路径在子函数中写死;
      2.在每个利用相对路径进行读写的文件头 添加了改变相对路径的操作
 * \n 20150624 V9.0 修改TeachParameters中的参数，将m_step去掉，增加负偏差,修改 WriteOnePointStandardTxt函数和WriteTwoPointStandardTxt函数
 *  \n 20150623 V 8.0 增加标准格式输出的方法实现
 *  \n 20150617 V7.0  读取错误时增加了错误日志输出
 *  \n 201505013 V 7.0 修改 写入参数的几个笔误，重载函数(Mat Point3f)
 *  \n 20150509 V6.0 为了做成安装程序的需要,将示教路径做成相对路径
 *  \n 20150507 V5.0 添加上位机所需的txt格式(未完成)
 *  \n 20150427 V4.0 添加Class TeachParameters类和读取示教参数的程序,示教参数的输入为车型编码
 *  \n 20150424 V0.3 添加写日志的功能 WriteLog()成员函数
 *  \n 20150424 V0.2 修改和增加了 Append和Truncate 两个输出成员变量
 *  \n 20150424 v0.1 定义了C3DLaserSensorRSerialIO类，并完成6个成员函数的编写
 */
#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <tchar.h>
#include <io.h>
#include <atlconv.h>
#include <wchar.h>
#include <direct.h>
#include <windows.h>
#include <stdio.h>
#include <math.h>

#include "opencv/cv.h"

using namespace std;
using namespace cv;

class TeachParameters
{
public:
	TeachParameters(void)
	{
		//////// 先对参数进行初始化操作
		m_type=1;
		m_exposure=3000;
		m_gain=300;
		m_threshold=150;
		m_upLimit=20;
		m_downLimit=100;
		m_leftLimit=300;
		m_rightLimit=1000;
		m_pointseril1="(08-03L)CPF";
		m_pointseril2="(08-03R)CPF";
		m_CMMX1=0;
		Pm_CMMX1Tol=1.5;
		Nm_CMMX1Tol=-1.5;
		m_CMMY1=0;
		Pm_CMMY1Tol=1.5;
		Nm_CMMY1Tol=-1.5;
		m_CMMZ1=0;
		Pm_CMMZ1Tol=1.5;
        Nm_CMMZ1Tol=-1.5;
		m_CMMX2=0;
		Pm_CMMX2Tol=1.5;
		Nm_CMMX2Tol=-1.5;
		m_CMMY2=0;
		Pm_CMMY2Tol=1.5;
        Nm_CMMY2Tol=-1.5;
		m_CMMZ2=0;
		Pm_CMMZ2Tol=1.5;
		Nm_CMMZ2Tol=-1.5; 
	}
	public: /////// 示教共需要28个参数

	int m_type;
	int m_exposure;
	int m_gain;
	int m_threshold;
	int m_upLimit;
	int m_downLimit;
	int m_leftLimit;
	int m_rightLimit;
	string m_pointseril1;/////// 折边输出两个特征
	string m_pointseril2;////// 这边输出两个特征
	float m_CMMX1;
	float m_CMMY1;
	float m_CMMZ1;
	float Pm_CMMX1Tol;
	float Pm_CMMY1Tol;
	float Pm_CMMZ1Tol;
	float Nm_CMMX1Tol;
	float Nm_CMMY1Tol;
	float Nm_CMMZ1Tol;
	float m_CMMX2;
	float m_CMMY2;
	float m_CMMZ2;
	float Pm_CMMX2Tol;
	float Pm_CMMY2Tol;
	float Pm_CMMZ2Tol;
	float Nm_CMMX2Tol;
	float Nm_CMMY2Tol;
	float Nm_CMMZ2Tol;
};


class C3DLaserSensorRSerialIO
{
public:
	C3DLaserSensorRSerialIO(void);
	~C3DLaserSensorRSerialIO(void);

#pragma region/////系统标定所需的读取参数操作

	//////其中 名义值、测量值、标定关节值、标定参数初始值均为double类型
	Mat ReadObservations();
    Mat ReadNominalData(int Num);
    Mat ReadInitialParametersCalib();/////标定专用	
	bool WriteInitialParametersCalib(Mat InitialParameters);/////按行优先写入
	Mat ReadBaseEyeData(int flag);
	Mat ReadMeasureTheta(int flag);
	Mat ReadMeasureThetaCalib(int flag);

	bool WriteBaseEyeData(int flag,Mat BaseEyeData);///// BaseEyeData为N*16的矩阵
	bool WriteBaseEyeDataCalib(int flag,Mat BaseEyeData);

	Mat DataTranslateOneRow(Mat HighRowsData);////// 4*4的BaseEyeData double ---1*16的BaseEyeData float
	Mat DataTranslateFourRow(Mat LowRowsData);////// 1*16的BaseEyeData----4*4的BaseEyeData
	Mat DataTranlateHom(Point3f SphereCenters);////// Point3f数据转换为 4*1的double类型
	Mat DoubleToFloat(Mat DoubleData);/////仅改变类型，不改变维数
	Mat FloatToDouble(Mat FloatData);/////仅改变类型，不改变维数
	bool IsCalibration(Mat MatErrorData,double Rate,double RateValue);////////
	Point3f EyeToBase(Mat BaseEye,Point3f Center);////////将相机坐标系下的坐标值转换到车身坐标系;
   
#pragma endregion
	//////// 读数据
	Mat ReadCameraCoors(string strCameraCoorPath);
	Mat ReadCameraMatrix(); ///////读取相机内参数矩阵，该矩阵为(3,3,CV_64FC1)

	Mat ReadDistCoeffs();  //////// 读取相机畸变系数矩阵，该矩阵为(1,5,CV_64FC1);

	Mat ReadLaserPlanePara(string LaserPlaneParaPath);/////// 读取光平面矩阵，该矩阵为(1,4,CV_64FC1) 法向量+平面上一点;

	Mat ReadLaserRotatingPlanePara(int Num_Of_Planes=23); //////读取摆动式线激光光平面方程，选择到文件夹

	///////// 写数据
	void CreFiles(string SavePath);/////一层一层创建文件夹的操作

	bool WriteRowFirstAPP(string SavePath,Mat CameraCoor);///////一行一行的输出数据,不删除原数据
	bool WriteRowFirstAPP(string SavePath,Point3f CameraCoor);

	bool WriteColFirstAPP(string SavePath,Mat CameraCoor);////////一列一列的输出数据,不删除原数据

    bool WriteRowFirstTrunc(string SavePath,Mat CameraCoor);///////一行一行的输出数据,删除原数据
    bool WriteRowFirstTrunc(string SavePath,Point3f CameraCoor);

	bool WriteColFirstTrunc(string SavePath,Mat CameraCoor);////////一列一列的输出数据,删除原数据

	
	/////////////// 输出上位机所需的标准格式
     bool WriteOnePointStandardTxt(string SavePath,Point3f CameraCoor,TeachParameters m_TeachParameters);

	 bool WriteTwoPointStandardTxt(string SavePath,vector<Point3f> CameraCoor, TeachParameters m_TeachParameters);	 

    ///////// 写日志的功能
    void WriteLog(string A_working_item); ////////为了避免读取过大文件夹造成的时间浪费，
    void WriteLogCalib(string Calib_working_item);//////为了避免日志文件共用，将标定日志单独输出
	// 故一小时的数据保存在一个log文件中,一天的log文件保存在以日期命名的文件夹中
	vector<TeachParameters>  ReadTeachParameter(int CarType);/////// 输入为一个3位数整型，
	////其中百位表示车型，十位表示车身分组号，个位表示测量标定柱编号；
	int TotalNum;
	char *DefaultWorkingPath;
	bool IsFileOpen;
	bool IsThetaFileOpen;	
private:
	TeachParameters m_TeachParameters;	
};
