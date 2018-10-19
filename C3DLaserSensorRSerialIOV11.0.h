/*! 
 *	\section About ���ڰڶ�ʽ�߼����Ӿ���������IO��
 *
 *  \section Announce ˵��������Ҫ��԰ڶ�ʽ�߼����Ӿ������������������д�Ľӿ�
 *  
 *  \section Log ��־

 *\n 20150907 
     1.���ݵĶ�������ȫ��Ϊdouble���ͣ����ݵ�д��ȫ��Ϊfloat����;
	 2.���ݵļ���ȫ������double����;
	 3.

 * \n 20150701 V11.0
    1. ���еĺ�������Mat���Ͳ�����ת����float���ͣ����ؾ�Ϊfloat����(�����ڱ궨����);
	���ڲ�ͬ��֮�����������־���õ����󣬹��ڱ궨�߳��еı궨��������Ҫ�Ķ�д�������£���Ҫ���������־�Ṳ�ã�
	�궨��־����д��һ���ļ���;
	2. ���WriteLogCalib����,���ڱ궨�߳�����Ҫ�������־������д��һ���ļ���;
	3. ��ӱ궨����Ķ�ȡ�ļ�����  ReadInitialParametersCalib�� WriteInitialParametersCalib��
	4. ���EyeToBase()����

 * \n 20150630 V10.0 ����ϵͳ�궨����Ķ�ȡ��������(��д�궨����)
      1.������Ķ����д��Ĳ���������·�����Ӻ�����д��;
      2.��ÿ���������·�����ж�д���ļ�ͷ ����˸ı����·���Ĳ���
 * \n 20150624 V9.0 �޸�TeachParameters�еĲ�������m_stepȥ�������Ӹ�ƫ��,�޸� WriteOnePointStandardTxt������WriteTwoPointStandardTxt����
 *  \n 20150623 V 8.0 ���ӱ�׼��ʽ����ķ���ʵ��
 *  \n 20150617 V7.0  ��ȡ����ʱ�����˴�����־���
 *  \n 201505013 V 7.0 �޸� д������ļ����������غ���(Mat Point3f)
 *  \n 20150509 V6.0 Ϊ�����ɰ�װ�������Ҫ,��ʾ��·���������·��
 *  \n 20150507 V5.0 �����λ�������txt��ʽ(δ���)
 *  \n 20150427 V4.0 ���Class TeachParameters��Ͷ�ȡʾ�̲����ĳ���,ʾ�̲���������Ϊ���ͱ���
 *  \n 20150424 V0.3 ���д��־�Ĺ��� WriteLog()��Ա����
 *  \n 20150424 V0.2 �޸ĺ������� Append��Truncate ���������Ա����
 *  \n 20150424 v0.1 ������C3DLaserSensorRSerialIO�࣬�����6����Ա�����ı�д
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
		//////// �ȶԲ������г�ʼ������
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
	public: /////// ʾ�̹���Ҫ28������

	int m_type;
	int m_exposure;
	int m_gain;
	int m_threshold;
	int m_upLimit;
	int m_downLimit;
	int m_leftLimit;
	int m_rightLimit;
	string m_pointseril1;/////// �۱������������
	string m_pointseril2;////// ��������������
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

#pragma region/////ϵͳ�궨����Ķ�ȡ��������

	//////���� ����ֵ������ֵ���궨�ؽ�ֵ���궨������ʼֵ��Ϊdouble����
	Mat ReadObservations();
    Mat ReadNominalData(int Num);
    Mat ReadInitialParametersCalib();/////�궨ר��	
	bool WriteInitialParametersCalib(Mat InitialParameters);/////��������д��
	Mat ReadBaseEyeData(int flag);
	Mat ReadMeasureTheta(int flag);
	Mat ReadMeasureThetaCalib(int flag);

	bool WriteBaseEyeData(int flag,Mat BaseEyeData);///// BaseEyeDataΪN*16�ľ���
	bool WriteBaseEyeDataCalib(int flag,Mat BaseEyeData);

	Mat DataTranslateOneRow(Mat HighRowsData);////// 4*4��BaseEyeData double ---1*16��BaseEyeData float
	Mat DataTranslateFourRow(Mat LowRowsData);////// 1*16��BaseEyeData----4*4��BaseEyeData
	Mat DataTranlateHom(Point3f SphereCenters);////// Point3f����ת��Ϊ 4*1��double����
	Mat DoubleToFloat(Mat DoubleData);/////���ı����ͣ����ı�ά��
	Mat FloatToDouble(Mat FloatData);/////���ı����ͣ����ı�ά��
	bool IsCalibration(Mat MatErrorData,double Rate,double RateValue);////////
	Point3f EyeToBase(Mat BaseEye,Point3f Center);////////���������ϵ�µ�����ֵת������������ϵ;
   
#pragma endregion
	//////// ������
	Mat ReadCameraCoors(string strCameraCoorPath);
	Mat ReadCameraMatrix(); ///////��ȡ����ڲ������󣬸þ���Ϊ(3,3,CV_64FC1)

	Mat ReadDistCoeffs();  //////// ��ȡ�������ϵ�����󣬸þ���Ϊ(1,5,CV_64FC1);

	Mat ReadLaserPlanePara(string LaserPlaneParaPath);/////// ��ȡ��ƽ����󣬸þ���Ϊ(1,4,CV_64FC1) ������+ƽ����һ��;

	Mat ReadLaserRotatingPlanePara(int Num_Of_Planes=23); //////��ȡ�ڶ�ʽ�߼����ƽ�淽�̣�ѡ���ļ���

	///////// д����
	void CreFiles(string SavePath);/////һ��һ�㴴���ļ��еĲ���

	bool WriteRowFirstAPP(string SavePath,Mat CameraCoor);///////һ��һ�е��������,��ɾ��ԭ����
	bool WriteRowFirstAPP(string SavePath,Point3f CameraCoor);

	bool WriteColFirstAPP(string SavePath,Mat CameraCoor);////////һ��һ�е��������,��ɾ��ԭ����

    bool WriteRowFirstTrunc(string SavePath,Mat CameraCoor);///////һ��һ�е��������,ɾ��ԭ����
    bool WriteRowFirstTrunc(string SavePath,Point3f CameraCoor);

	bool WriteColFirstTrunc(string SavePath,Mat CameraCoor);////////һ��һ�е��������,ɾ��ԭ����

	
	/////////////// �����λ������ı�׼��ʽ
     bool WriteOnePointStandardTxt(string SavePath,Point3f CameraCoor,TeachParameters m_TeachParameters);

	 bool WriteTwoPointStandardTxt(string SavePath,vector<Point3f> CameraCoor, TeachParameters m_TeachParameters);	 

    ///////// д��־�Ĺ���
    void WriteLog(string A_working_item); ////////Ϊ�˱����ȡ�����ļ�����ɵ�ʱ���˷ѣ�
    void WriteLogCalib(string Calib_working_item);//////Ϊ�˱�����־�ļ����ã����궨��־�������
	// ��һСʱ�����ݱ�����һ��log�ļ���,һ���log�ļ��������������������ļ�����
	vector<TeachParameters>  ReadTeachParameter(int CarType);/////// ����Ϊһ��3λ�����ͣ�
	////���а�λ��ʾ���ͣ�ʮλ��ʾ�������ţ���λ��ʾ�����궨����ţ�
	int TotalNum;
	char *DefaultWorkingPath;
	bool IsFileOpen;
	bool IsThetaFileOpen;	
private:
	TeachParameters m_TeachParameters;	
};
