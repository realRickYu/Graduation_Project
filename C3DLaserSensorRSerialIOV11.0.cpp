#include "C3DLaserSensorRSerialIOV11.0.h"

C3DLaserSensorRSerialIO::C3DLaserSensorRSerialIO(void)
{
	DefaultWorkingPath="E:\\";
	IsFileOpen=false;
	IsThetaFileOpen=false;
}

C3DLaserSensorRSerialIO::~C3DLaserSensorRSerialIO(void)
{

}


/////////读数据
Mat C3DLaserSensorRSerialIO::ReadCameraCoors(string strCameraCoorPath)
{
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	Mat ImageCoor;
	Mat SingleData(1,3,CV_64FC1,Scalar(0));
	Mat DefaultImageCoor(1,3,CV_64FC1,Scalar(0));
	USES_CONVERSION;   
	WCHAR*WFileName=A2W(strCameraCoorPath.c_str());
	ifstream RImagCoor;
	RImagCoor.open(WFileName);
	if (RImagCoor.is_open())
	{
		while (!RImagCoor.eof())
		{
			for (int i=0; i<3; i++)
			{
				RImagCoor>>SingleData.at<double>(0,i);
			}
			if (RImagCoor.fail())
			{
				break;
			} 
			else
			{
				ImageCoor.push_back(SingleData);
			}
		}
	}
	else
	{
		RImagCoor.close();
		return  DefaultImageCoor;     
	}
	RImagCoor.close();
	return ImageCoor;
}

Mat C3DLaserSensorRSerialIO::ReadCameraMatrix()//////选择到文件名
{
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	string CameraMatrixPath="Parameters\\MeasurementParas\\CameraMatrix.txt";
	USES_CONVERSION;   
	WCHAR*WFileName=A2W(CameraMatrixPath.c_str());
	ifstream RCameraMatrix;
	RCameraMatrix.open(WFileName);
	Mat CameraMatrix(3,3,CV_64FC1,Scalar(0));
	Mat defaultCameraMatrix(1,1,CV_64FC1,Scalar(0));
	if (RCameraMatrix.is_open())
	{
		for (int i=0;i<CameraMatrix.rows;i++)
		{
			for (int j=0;j<CameraMatrix.cols;j++)
			{
				RCameraMatrix>>CameraMatrix.at<double>(i,j);
			}

		} 
		RCameraMatrix.close();

	} 
	else
	{
		WriteLog("Error:ReadCameraMatrix_相机内参数读取失败");
		return defaultCameraMatrix;
	}
	return CameraMatrix;
}


Mat C3DLaserSensorRSerialIO::ReadDistCoeffs()//////选择到文件名
{
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	string DistCoeffsPath="Parameters\\MeasurementParas\\DistCoeffes.txt";
	USES_CONVERSION;   
	WCHAR*WFileName=A2W(DistCoeffsPath.c_str());
	ifstream RDistCoeffs;
	RDistCoeffs.open(WFileName);
	Mat DistCoeffs(1,5,CV_64FC1,Scalar(0));
	Mat defaultDistCoeffs(1,1,CV_64FC1,Scalar(0));
	if (RDistCoeffs.is_open())
	{
		for (int j=0;j<DistCoeffs.cols;j++)
		{
			RDistCoeffs>>DistCoeffs.at<double>(0,j);
		}
		RDistCoeffs.close();
	} 
	else
	{
		WriteLog("Error:ReadDistCoeffs_相机畸变参数读取失败");
		return defaultDistCoeffs;
	}
	return DistCoeffs;
}



Mat C3DLaserSensorRSerialIO::ReadLaserPlanePara(string LaserPlaneParaPath)//////选择到文件名
{
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	USES_CONVERSION;   
	WCHAR*WFileName=A2W(LaserPlaneParaPath.c_str());
	ifstream RLaserPlanePara;
	RLaserPlanePara.open(WFileName);
	Mat LaserPlanePara(1,4,CV_64FC1,Scalar(0));//////光平面参数保存格式为  法向量+平面上一点
	Mat defaultLaserPlanePara(1,1,CV_64FC1,Scalar(0));
	if (RLaserPlanePara.is_open())
	{
		for (int i=0;i<LaserPlanePara.rows;i++)
		{
			for (int j=0;j<LaserPlanePara.cols;j++)
			{
				RLaserPlanePara>>LaserPlanePara.at<double>(i,j);
			}
		}

		RLaserPlanePara.close();
	} 
	else
	{
		WriteLog("Error:ReadLaserPlanePara_光平面参数读取失败");
		return defaultLaserPlanePara;
	}

	return LaserPlanePara;
}


Mat C3DLaserSensorRSerialIO::ReadLaserRotatingPlanePara(int Num_Of_Planes/* =30 */)
{
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	string LaserRotaingPlaneParaPath="Parameters\\MeasurementParas\\PlaneFitPara\\";
	Mat LaserPlaneParas(Num_Of_Planes,4,CV_64FC1,Scalar(0));
	stringstream FileNum;
	string strFileNum="1";
	string StrFullPlanePath=" ";
	for (int i=1;i<Num_Of_Planes+1;i++)
	{
		FileNum<<i;
		FileNum>>strFileNum;
		FileNum.clear();
		StrFullPlanePath=LaserRotaingPlaneParaPath+strFileNum+".txt";
		LaserPlaneParas.row(i-1)=ReadLaserPlanePara(StrFullPlanePath)+0;/////左边为临时头文件，故需要进行数学运算，才能发生文件的拷贝
	}
	return LaserPlaneParas;
}




/////////写数据
void C3DLaserSensorRSerialIO::CreFiles(string SavePath)
{
	string Flag1="\\";
	string Flag2="/";
	string::size_type Position=0;
	if (SavePath.find(Flag1)!=string::npos)//// 是否存在所要查找的标示
	{
		while ((Position=SavePath.find_first_of(Flag1,Position))!=string::npos)
		{
			string SubPath=SavePath.substr(0,Position+1);
			if (-1==_chdir(SubPath.c_str()))
			{
				_mkdir(SubPath.c_str());
			} 
			Position++;

		}
	}  
	else
	{
		while ((Position=SavePath.find_first_of(Flag2,Position))!=string::npos)
		{
			string SubPath=SavePath.substr(0,Position+1);
			if (-1==_chdir(SubPath.c_str()))
			{
				_mkdir(SubPath.c_str());
			} 
			Position++;

		}
	}
}


bool C3DLaserSensorRSerialIO::WriteRowFirstAPP(string SavePath,Mat CameraCoor) //////// 输出float类型
{
	Mat FloatCameraCoor;
	if (CameraCoor.depth()==6)
	{
		FloatCameraCoor=DoubleToFloat(CameraCoor);
	} 
	else
	{
		FloatCameraCoor=CameraCoor;
	}

	CreFiles(SavePath);
	USES_CONVERSION;   
	WCHAR *WSavePath=A2W(SavePath.c_str());
	ofstream WCameraCoorData;
	WCameraCoorData.open(WSavePath,ios_base::out|ios_base::app);
	if (WCameraCoorData.is_open())
	{
		for (int i=0;i<FloatCameraCoor.rows;i++)
		{
			for (int j=0;j<FloatCameraCoor.cols;j++)
			{
				WCameraCoorData<<FloatCameraCoor.at<float>(i,j)<<' ';
			}
			WCameraCoorData<<'\n';
		}
		WCameraCoorData.close();
		return true;
	} 
	else
	{
		return false;
	}
}


bool C3DLaserSensorRSerialIO::WriteRowFirstAPP(string SavePath,Point3f CameraCoor)
{
	CreFiles(SavePath);
	USES_CONVERSION;   
	WCHAR *WSavePath=A2W(SavePath.c_str());
	ofstream WCameraCoorData;
	WCameraCoorData.open(WSavePath,ios_base::out|ios_base::app);
	if (WCameraCoorData.is_open())
	{

		WCameraCoorData<<CameraCoor.x<<' '<<CameraCoor.y<<' '<<CameraCoor.z;
		WCameraCoorData<<'\n';
		WCameraCoorData.close();
		return true;
	}
	else
	{
		return false;
	}
}



bool C3DLaserSensorRSerialIO::WriteRowFirstTrunc(string SavePath,Mat CameraCoor) //////// 删除原文件
{
	Mat FloatCameraCoor;

	if (CameraCoor.depth()==6)
	{
		FloatCameraCoor=DoubleToFloat(CameraCoor);
	} 
	else
	{
		FloatCameraCoor=CameraCoor;
	}

	CreFiles(SavePath);
	USES_CONVERSION;   
	WCHAR *WSavePath=A2W(SavePath.c_str());
	ofstream WCameraCoorData;
	WCameraCoorData.open(WSavePath,ios_base::out|ios_base::trunc);
	if (WCameraCoorData.is_open())
	{
		for (int i=0;i<FloatCameraCoor.rows;i++)
		{
			for (int j=0;j<FloatCameraCoor.cols;j++)
			{
				WCameraCoorData<<FloatCameraCoor.at<float>(i,j)<<' ';
			}
			WCameraCoorData<<'\n';
		}
		WCameraCoorData.close();
		return true;
	} 
	else
	{
		return false;
	}
}


bool C3DLaserSensorRSerialIO::WriteRowFirstTrunc(string SavePath, Point3f CameraCoor)
{

	CreFiles(SavePath);
	USES_CONVERSION;   
	WCHAR *WSavePath=A2W(SavePath.c_str());
	ofstream WCameraCoorData;
	WCameraCoorData.open(WSavePath,ios_base::out|ios_base::trunc);
	if (WCameraCoorData.is_open())
	{

		WCameraCoorData<<CameraCoor.x<<' '<<CameraCoor.y<<' '<<CameraCoor.z;
		WCameraCoorData<<'\n';
		WCameraCoorData.close();
		return true;
	}
	else
	{
		return false;
	}
}


bool C3DLaserSensorRSerialIO::WriteColFirstAPP(string SavePath,Mat CameraCoor)   //////// 不删除原文件
{
	Mat FloatCameraCoor;
	if (CameraCoor.depth()==6)
	{
		FloatCameraCoor=DoubleToFloat(CameraCoor);
	} 
	else
	{
		FloatCameraCoor=CameraCoor;
	}
	CreFiles(SavePath);
	USES_CONVERSION;   
	WCHAR *WSavePath=A2W(SavePath.c_str());
	ofstream WCameraCoorData;
	WCameraCoorData.open(WSavePath,ios_base::out|ios_base::app);
	if (WCameraCoorData.is_open())
	{
		for (int j=0;j<FloatCameraCoor.cols;j++)
		{
			for (int i=0;i<FloatCameraCoor.rows;i++)
			{
				WCameraCoorData<<FloatCameraCoor.at<float>(i,j)<<' ';
			}
			WCameraCoorData<<'\n';
		}
		WCameraCoorData.close();
		return true;
	} 
	else
	{
		return false;
	}

}


bool C3DLaserSensorRSerialIO::WriteColFirstTrunc(string SavePath,Mat CameraCoor)   //////// 删除原文件
{
	Mat FloatCameraCoor;
	if (CameraCoor.depth()==6)
	{
		FloatCameraCoor=DoubleToFloat(CameraCoor);
	} 
	else
	{
		FloatCameraCoor=CameraCoor;
	}
	CreFiles(SavePath);
	USES_CONVERSION;   
	WCHAR *WSavePath=A2W(SavePath.c_str());
	ofstream WCameraCoorData;
	WCameraCoorData.open(WSavePath,ios_base::out|ios_base::trunc);
	if (WCameraCoorData.is_open())
	{
		for (int j=0;j<FloatCameraCoor.cols;j++)
		{
			for (int i=0;i<FloatCameraCoor.rows;i++)
			{
				WCameraCoorData<<FloatCameraCoor.at<float>(i,j)<<' ';
			}
			WCameraCoorData<<'\n';
		}
		WCameraCoorData.close();
		return true;
	} 
	else
	{
		return false;
	}

}

void C3DLaserSensorRSerialIO::WriteLog(string A_working_item) /////// 
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	char log_file_path[80];
	memset(log_file_path,'\0',80);
	sprintf_s(log_file_path,"E:\\在线检测\\log\\%04d%02d%02d\\",sys.wYear,sys.wMonth,sys.wDay);
	string strlog_file_path(log_file_path);
	CreFiles(strlog_file_path);
	char log_item_hour[40];
	char log_item_mSencond[60];
	memset(log_item_hour,'\0',40);
	memset(log_item_mSencond,'\0',60);
	sprintf_s(log_item_hour,"%04d%02d%02d%02d",sys.wYear,sys.wMonth,sys.wDay,sys.wHour);
	sprintf_s(log_item_mSencond,"%04d%02d%02d-%02d:%02d-%02d:%02d",sys.wYear,sys.wMonth,sys.wDay,sys.wHour,
		sys.wMinute,sys.wSecond,sys.wMilliseconds);
	USES_CONVERSION;  
	string Full_Log_Path=strlog_file_path+log_item_hour+".log";
	WCHAR *Wlog_fileHour_path=A2W(Full_Log_Path.c_str());
	ofstream WWriteLog;
	WWriteLog.open(Wlog_fileHour_path,ios_base::out|ios_base::app);
	if (WWriteLog.is_open())
	{
		WWriteLog<<log_item_mSencond<<"	< ";
		WWriteLog<<A_working_item<<" >\r\n";
		WWriteLog.close();
	}
}

void C3DLaserSensorRSerialIO::WriteLogCalib(string Calib_working_item) /////// 
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	char log_file_path[80];
	memset(log_file_path,'\0',80);
	sprintf_s(log_file_path,"E:\\在线检测\\log\\%04d%02d%02d_Calib\\",sys.wYear,sys.wMonth,sys.wDay);
	string strlog_file_path(log_file_path);
	CreFiles(strlog_file_path);
	char log_item_hour[40];
	char log_item_mSencond[60];
	memset(log_item_hour,'\0',40);
	memset(log_item_mSencond,'\0',60);
	sprintf_s(log_item_hour,"%04d%02d%02d%02d",sys.wYear,sys.wMonth,sys.wDay,sys.wHour);
	sprintf_s(log_item_mSencond,"%04d%02d%02d-%02d:%02d-%02d:%02d",sys.wYear,sys.wMonth,sys.wDay,sys.wHour,
		sys.wMinute,sys.wSecond,sys.wMilliseconds);
	USES_CONVERSION;  
	string Full_Log_Path=strlog_file_path+log_item_hour+".log";
	WCHAR *Wlog_fileHour_path=A2W(Full_Log_Path.c_str());
	ofstream WWriteLog;
	WWriteLog.open(Wlog_fileHour_path,ios_base::out|ios_base::app);
	if (WWriteLog.is_open())
	{
		WWriteLog<<log_item_mSencond<<"	< ";
		WWriteLog<<Calib_working_item<<" >\r\n";
		WWriteLog.close();
	}
}


/////// 输入为一个3位数整型，
////其中百位表示车型，十位表示车身分组号，个位表示测量标定柱编号；
vector<TeachParameters> C3DLaserSensorRSerialIO::ReadTeachParameter(int CarType)
{
	////// 百位和十位确定测量示教参数,个位数确定测量标定柱参数
	vector<TeachParameters> TotalTeachParameters;
	TotalNum=1;
#pragma region ////// 利用百位和十位确定测量示教参数
	string CarFileName=" ";
	string CalibFileName=" ";
	int Carnumber=CarType/10;
	int Calibnumber=CarType%10;
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	switch(Carnumber) ///////// modified in 20150509，修改成相对路径的读取方式，程序发布时读取。
	{
	case 11:
		CarFileName="Parameters\\TeachParameters\\CarType1\\1.txt";
		break;
	case 12:
		CarFileName="Parameters\\TeachParameters\\CarType1\\2.txt";
		break;
	case 13:
		CarFileName="Parameters\\TeachParameters\\CarType1\\3.txt";
		break;
	case 14:
		CarFileName="Parameters\\TeachParameters\\CarType1\\4.txt";
		break;
	case 21:
		CarFileName="Parameters\\TeachParameters\\CarType2\\1.txt";
		break;
	case 22:
		CarFileName="Parameters\\TeachParameters\\CarType2\\2.txt";
		break;
	case 23:
		CarFileName="Parameters\\TeachParameters\\CarType2\\3.txt";
		break;
	case 24:
		CarFileName="Parameters\\TeachParameters\\CarType2\\4.txt";
		break;
	case 31:
		CarFileName="Parameters\\TeachParameters\\CarType3\\1.txt";
		break;
	case 32:
		CarFileName="Parameters\\TeachParameters\\CarType3\\2.txt";
		break;
	case 33:
		CarFileName="Parameters\\TeachParameters\\CarType3\\3.txt";
		break;
	case 34:
		CarFileName="Parameters\\TeachParameters\\CarType3\\4.txt";
		break;
	case 41:		
		CarFileName="Parameters\\TeachParameters\\CarType4\\1.txt";
		break;
	case 42:
		CarFileName="Parameters\\TeachParameters\\CarType4\\2.txt";
		break;
	case 43:
		CarFileName="Parameters\\TeachParameters\\CarType4\\3.txt";
		break;
	case 44:
		CarFileName="Parameters\\TeachParameters\\CarType4\\4.txt";
		break;
	case 51:
		CarFileName="Parameters\\TeachParameters\\CarType5\\1.txt";
		break;
	case 52:
		CarFileName="Parameters\\TeachParameters\\CarType5\\2.txt";
		break;
	case 53:
		CarFileName="Parameters\\TeachParameters\\CarType5\\3.txt";
		break;
	case 54:
		CarFileName="Parameters\\TeachParameters\\CarType5\\4.txt";
		break;
	case 61:
		CarFileName="Parameters\\TeachParameters\\CarType6\\1.txt";
		break;
	case 62:
		CarFileName="Parameters\\TeachParameters\\CarType6\\2.txt";
		break;
	case 63:
		CarFileName="Parameters\\TeachParameters\\CarType6\\3.txt";
		break;
	case 64:
		CarFileName="Parameters\\TeachParameters\\CarType6\\4.txt";
		break;
	case 71:
		CarFileName="Parameters\\TeachParameters\\CarType7\\1.txt";
		break;
	case 72:
		CarFileName="Parameters\\TeachParameters\\CarType7\\2.txt";
		break;
	case 73:
		CarFileName="Parameters\\TeachParameters\\CarType7\\3.txt";
		break;
	case 74:
		CarFileName="Parameters\\TeachParameters\\CarType7\\4.txt";
		break;
	case 81:
		CarFileName="Parameters\\TeachParameters\\CarType8\\1.txt";
		break;
	case 82:
		CarFileName="Parameters\\TeachParameters\\CarType8\\2.txt";
		break;
	case 83:
		CarFileName="Parameters\\TeachParameters\\CarType8\\3.txt";
		break;
	case 84:
		CarFileName="Parameters\\TeachParameters\\CarType8\\4.txt";
		break;
	default:
		TotalTeachParameters.push_back(m_TeachParameters);
		WriteLog("Error:ReadTeachParameter车型和分组号传输错误");
		return TotalTeachParameters;
	}
#pragma endregion 

	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
#pragma  region //////利用个位数确定测量标定柱参数
	switch(Calibnumber)
	{
	case 1:
		CalibFileName="Parameters\\TeachParameters\\CalibTarget\\1.txt";
		break;
	case 2:
		CalibFileName="Parameters\\TeachParameters\\CalibTarget\\2.txt";
		break;
	case 3:
		CalibFileName="Parameters\\TeachParameters\\CalibTarget\\3.txt";
		break;
	case 4:
		CalibFileName="Parameters\\TeachParameters\\CalibTarget\\4.txt";
		break;
	default:
		TotalTeachParameters.push_back(m_TeachParameters);
		WriteLog("Error:ReadTeachParameter标定柱编号传输错误");
		return TotalTeachParameters;
	}
#pragma endregion

	USES_CONVERSION;
	WCHAR *WCarFileName =A2W(CarFileName.c_str());
	WCHAR *WCalibFileName =A2W(CalibFileName.c_str());
	ifstream RWCarFileName;
	ifstream RWCalibFileName;
#pragma  region ////// 读取测量白车身示教参数
	RWCarFileName.open(WCarFileName);
	if (RWCarFileName.is_open())
	{
		RWCarFileName>>TotalNum;//////获取示教点的个数
		for(int i=0;i<TotalNum;i++)
		{
			RWCarFileName>>m_TeachParameters.m_type;
			if (1==m_TeachParameters.m_type) //////修边特征,读取两组特征点参数
			{
				RWCarFileName>>m_TeachParameters.m_exposure;
				RWCarFileName>>m_TeachParameters.m_gain;
				RWCarFileName>>m_TeachParameters.m_threshold;
				RWCarFileName>>m_TeachParameters.m_upLimit;
				RWCarFileName>>m_TeachParameters.m_downLimit;
				RWCarFileName>>m_TeachParameters.m_leftLimit;
				RWCarFileName>>m_TeachParameters.m_rightLimit;
				RWCarFileName>>m_TeachParameters.m_pointseril1;
				RWCarFileName>>m_TeachParameters.m_CMMX1;
				RWCarFileName>>m_TeachParameters.m_CMMY1;
				RWCarFileName>>m_TeachParameters.m_CMMZ1;
				RWCarFileName>>m_TeachParameters.Pm_CMMX1Tol;
				RWCarFileName>>m_TeachParameters.Pm_CMMY1Tol;
				RWCarFileName>>m_TeachParameters.Pm_CMMZ1Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMX1Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMY1Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMZ1Tol;
				RWCarFileName>>m_TeachParameters.m_pointseril2;
				RWCarFileName>>m_TeachParameters.m_CMMX2;
				RWCarFileName>>m_TeachParameters.m_CMMY2;
				RWCarFileName>>m_TeachParameters.m_CMMZ2;
				RWCarFileName>>m_TeachParameters.Pm_CMMX2Tol;
				RWCarFileName>>m_TeachParameters.Pm_CMMY2Tol;
				RWCarFileName>>m_TeachParameters.Pm_CMMZ2Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMX2Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMY2Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMZ2Tol;
			} 
			else
			{
				RWCarFileName>>m_TeachParameters.m_exposure;
				RWCarFileName>>m_TeachParameters.m_gain;
				RWCarFileName>>m_TeachParameters.m_threshold;
				RWCarFileName>>m_TeachParameters.m_upLimit;
				RWCarFileName>>m_TeachParameters.m_downLimit;
				RWCarFileName>>m_TeachParameters.m_leftLimit;
				RWCarFileName>>m_TeachParameters.m_rightLimit;
				RWCarFileName>>m_TeachParameters.m_pointseril1;
				RWCarFileName>>m_TeachParameters.m_CMMX1;
				RWCarFileName>>m_TeachParameters.m_CMMY1;
				RWCarFileName>>m_TeachParameters.m_CMMZ1;
				RWCarFileName>>m_TeachParameters.Pm_CMMX1Tol;
				RWCarFileName>>m_TeachParameters.Pm_CMMY1Tol;
				RWCarFileName>>m_TeachParameters.Pm_CMMZ1Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMX1Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMY1Tol;
				RWCarFileName>>m_TeachParameters.Nm_CMMZ1Tol;
			}
			TotalTeachParameters.push_back(m_TeachParameters);
		}
		RWCarFileName.close();		
	}
	else
	{
		WriteLog("Error:ReadTeachParameter车身示教文件路径错误");
		TotalTeachParameters.push_back(m_TeachParameters);
		return TotalTeachParameters;		
	}

#pragma endregion

#pragma region ////// 读取测量标定柱的示教参数
	RWCalibFileName.open(WCalibFileName);
	if (RWCalibFileName.is_open())
	{
		for (int j=0;j<6;j++)
		{ ////////// 里面测点名意义不大,m_CMM 为球心的理论坐标值，m_CMMX1Tol为球心容许的偏差，大于允许偏差则进行系统标定
			RWCalibFileName>>m_TeachParameters.m_type;
			RWCalibFileName>>m_TeachParameters.m_exposure;
			RWCalibFileName>>m_TeachParameters.m_gain;
			RWCalibFileName>>m_TeachParameters.m_threshold;
			RWCalibFileName>>m_TeachParameters.m_upLimit;
			RWCalibFileName>>m_TeachParameters.m_downLimit;
			RWCalibFileName>>m_TeachParameters.m_leftLimit;
			RWCalibFileName>>m_TeachParameters.m_rightLimit;
			RWCalibFileName>>m_TeachParameters.m_pointseril1; 
			RWCalibFileName>>m_TeachParameters.m_CMMX1;
			RWCalibFileName>>m_TeachParameters.m_CMMY1;
			RWCalibFileName>>m_TeachParameters.m_CMMZ1;
			RWCalibFileName>>m_TeachParameters.Pm_CMMX1Tol;
			RWCalibFileName>>m_TeachParameters.Pm_CMMY1Tol;
			RWCalibFileName>>m_TeachParameters.Pm_CMMZ1Tol;
			RWCalibFileName>>m_TeachParameters.Nm_CMMX1Tol;
			RWCalibFileName>>m_TeachParameters.Nm_CMMY1Tol;
			RWCalibFileName>>m_TeachParameters.Nm_CMMZ1Tol;
			TotalTeachParameters.push_back(m_TeachParameters);
		} 	
		RWCalibFileName.close();
	} 
	else
	{
		TotalTeachParameters.push_back(m_TeachParameters);
		WriteLog("Error:ReadTeachParameter标定柱示教文件路径错误");
		return TotalTeachParameters; 
	}

#pragma endregion

	return TotalTeachParameters;
}



bool C3DLaserSensorRSerialIO::WriteOnePointStandardTxt(string SavePath,Point3f CameraCoor,TeachParameters m_TeachParameters)
{
	/////// 将SavePath使用绝对路径表示 存放在E:在线测量文件夹中
	CreFiles(SavePath);
#pragma region ////将数值转换成string类型
	stringstream DTStrTranlate;
	string DevXValue="";
	string DevYValue="";
	string DevZValue="";
	DTStrTranlate<<CameraCoor.x-m_TeachParameters.m_CMMX1;
	DTStrTranlate>>DevXValue;
	DTStrTranlate.clear();
	DTStrTranlate<<CameraCoor.y-m_TeachParameters.m_CMMY1;
	DTStrTranlate>>DevYValue;
	DTStrTranlate.clear();
	DTStrTranlate<<CameraCoor.z-m_TeachParameters.m_CMMZ1;
	DTStrTranlate>>DevZValue;
	DTStrTranlate.clear();

	string strm_CMMX="";
	string strm_CMMY="";
	string strm_CMMZ="";
	string strPm_CMMXTol="";
	string strPm_CMMYTol="";
	string strPm_CMMZTol="";
	string strNm_CMMXTol="";
	string strNm_CMMYTol="";
	string strNm_CMMZTol="";
	DTStrTranlate<<m_TeachParameters.m_CMMX1;
	DTStrTranlate>>strm_CMMX;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.m_CMMY1;
	DTStrTranlate>>strm_CMMY;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.m_CMMZ1;
	DTStrTranlate>>strm_CMMZ;
	DTStrTranlate.clear();

	DTStrTranlate<<m_TeachParameters.Pm_CMMX1Tol;
	DTStrTranlate>>strPm_CMMXTol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Pm_CMMY1Tol;
	DTStrTranlate>>strPm_CMMYTol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Pm_CMMZ1Tol;
	DTStrTranlate>>strPm_CMMZTol;
	DTStrTranlate.clear();

	DTStrTranlate<<m_TeachParameters.Nm_CMMX1Tol;
	DTStrTranlate>>strNm_CMMXTol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Nm_CMMY1Tol;
	DTStrTranlate>>strNm_CMMYTol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Nm_CMMZ1Tol;
	DTStrTranlate>>strNm_CMMZTol;
	DTStrTranlate.clear();
#pragma endregion

	string strOutputStream = "DIM CCM(";
	strOutputStream = strOutputStream+m_TeachParameters.m_pointseril1+"),";
	strOutputStream=strOutputStream+strm_CMMX+","+strm_CMMY+","+strm_CMMZ+'\r'+'\n';
	strOutputStream=strOutputStream+"AX        "+"NOMINAL        "+"+TOL        "+"-TOL        "+"DEV"+'\r'+'\n';
	strOutputStream=strOutputStream+"X         "+strm_CMMX+"        "+strPm_CMMXTol+"        "+strNm_CMMXTol
		+"        "+DevXValue+'\r'+'\n';
	strOutputStream=strOutputStream+"Y         "+strm_CMMY+"        "+strPm_CMMYTol+"        "+strNm_CMMYTol
		+"        "+DevYValue+'\r'+'\n';
	strOutputStream=strOutputStream+"Z         "+strm_CMMZ+"        "+strPm_CMMZTol+"        "+strNm_CMMZTol
		+"        "+DevZValue+'\r'+'\n';
	strOutputStream=strOutputStream+"END DIM"+'\r'+'\n';
	/////// 先写
	USES_CONVERSION;
	WCHAR *WSavePath=A2W(SavePath.c_str());
	WCHAR *WOutputStream=A2W(strOutputStream.c_str());
	ofstream WFile;
	if (_access(SavePath.c_str(),02)!=-1)////// 文件夹存在则不写入
	{
		WFile.open(WSavePath,ios::app|ios::out|ios::binary);
		if (WFile.is_open())
		{
			WFile.write((char *)WOutputStream,strOutputStream.length()*2);
			WFile.close();
			return true;
		}
	} 
	else
	{
		WFile.open(WSavePath,ios::app|ios::out|ios::binary);
		if (WFile.is_open())
		{
			WFile.write("\xFF\xFE",2); 
			WFile.write((char *)WOutputStream,strOutputStream.length()*2);
			WFile.close();
			return true;
		}
	}
	return false;
}




bool C3DLaserSensorRSerialIO::WriteTwoPointStandardTxt(string SavePath,vector<Point3f> CameraCoor, TeachParameters m_TeachParameters)
{
	/////// 将SavePath使用绝对路径表示 存放在E:在线测量文件夹中
	CreFiles(SavePath);
#pragma region ////将数值转换成string类型
	stringstream DTStrTranlate;
	string DevX1Value="";
	string DevY1Value="";
	string DevZ1Value="";
	DTStrTranlate<<CameraCoor.at(0).x-m_TeachParameters.m_CMMX1;
	DTStrTranlate>>DevX1Value;
	DTStrTranlate.clear();
	DTStrTranlate<<CameraCoor.at(0).y-m_TeachParameters.m_CMMY1;
	DTStrTranlate>>DevY1Value;
	DTStrTranlate.clear();
	DTStrTranlate<<CameraCoor.at(0).z-m_TeachParameters.m_CMMZ1;
	DTStrTranlate>>DevZ1Value;
	DTStrTranlate.clear();

	string strm_CMMX1="";
	string strm_CMMY1="";
	string strm_CMMZ1="";
	string strPm_CMMX1Tol="";
	string strPm_CMMY1Tol="";
	string strPm_CMMZ1Tol="";
	string strNm_CMMX1Tol="";
	string strNm_CMMY1Tol="";
	string strNm_CMMZ1Tol="";
	DTStrTranlate<<m_TeachParameters.m_CMMX1;
	DTStrTranlate>>strm_CMMX1;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.m_CMMY1;
	DTStrTranlate>>strm_CMMY1;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.m_CMMZ1;
	DTStrTranlate>>strm_CMMZ1;
	DTStrTranlate.clear();

	DTStrTranlate<<m_TeachParameters.Pm_CMMX1Tol;
	DTStrTranlate>>strPm_CMMX1Tol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Pm_CMMY1Tol;
	DTStrTranlate>>strPm_CMMY1Tol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Pm_CMMZ1Tol;
	DTStrTranlate>>strPm_CMMZ1Tol;
	DTStrTranlate.clear();

	DTStrTranlate<<m_TeachParameters.Nm_CMMX1Tol;
	DTStrTranlate>>strNm_CMMX1Tol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Nm_CMMY1Tol;
	DTStrTranlate>>strNm_CMMY1Tol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Nm_CMMZ1Tol;
	DTStrTranlate>>strNm_CMMZ1Tol;
	DTStrTranlate.clear();


	string DevX2Value="";
	string DevY2Value="";
	string DevZ2Value="";
	DTStrTranlate<<CameraCoor.at(1).x-m_TeachParameters.m_CMMX2;
	DTStrTranlate>>DevX2Value;
	DTStrTranlate.clear();
	DTStrTranlate<<CameraCoor.at(1).y-m_TeachParameters.m_CMMY2;
	DTStrTranlate>>DevY2Value;
	DTStrTranlate.clear();
	DTStrTranlate<<CameraCoor.at(1).z-m_TeachParameters.m_CMMZ2;
	DTStrTranlate>>DevZ2Value;
	DTStrTranlate.clear();

	string strm_CMMX2="";
	string strm_CMMY2="";
	string strm_CMMZ2="";
	string strPm_CMMX2Tol="";
	string strPm_CMMY2Tol="";
	string strPm_CMMZ2Tol="";
	string strNm_CMMX2Tol="";
	string strNm_CMMY2Tol="";
	string strNm_CMMZ2Tol="";
	DTStrTranlate<<m_TeachParameters.m_CMMX2;
	DTStrTranlate>>strm_CMMX2;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.m_CMMY2;
	DTStrTranlate>>strm_CMMY2;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.m_CMMZ2;
	DTStrTranlate>>strm_CMMZ2;
	DTStrTranlate.clear();

	DTStrTranlate<<m_TeachParameters.Pm_CMMX2Tol;
	DTStrTranlate>>strPm_CMMX2Tol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Pm_CMMY2Tol;
	DTStrTranlate>>strPm_CMMY2Tol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Pm_CMMZ2Tol;
	DTStrTranlate>>strPm_CMMZ2Tol;
	DTStrTranlate.clear();

	DTStrTranlate<<m_TeachParameters.Nm_CMMX2Tol;
	DTStrTranlate>>strNm_CMMX2Tol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Nm_CMMY2Tol;
	DTStrTranlate>>strNm_CMMY2Tol;
	DTStrTranlate.clear();
	DTStrTranlate<<m_TeachParameters.Nm_CMMZ2Tol;
	DTStrTranlate>>strNm_CMMZ2Tol;
	DTStrTranlate.clear();
#pragma endregion

	string strOutputStream1 = "DIM CCM(";
	strOutputStream1 = strOutputStream1+m_TeachParameters.m_pointseril1+"),";
	strOutputStream1=strOutputStream1+strm_CMMX1+","+strm_CMMY1+","+strm_CMMZ1+'\r'+'\n';
	strOutputStream1=strOutputStream1+"AX        "+"NOMINAL        "+"+TOL        "+"-TOL        "+"DEV"+'\r'+'\n';
	strOutputStream1=strOutputStream1+"X         "+strm_CMMX1+"        "+strPm_CMMX1Tol+"        "+strNm_CMMX1Tol
		+"        "+DevX1Value+'\r'+'\n';
	strOutputStream1=strOutputStream1+"Y         "+strm_CMMY1+"        "+strPm_CMMY1Tol+"        "+strNm_CMMY1Tol
		+"        "+DevY1Value+'\r'+'\n';
	strOutputStream1=strOutputStream1+"Z         "+strm_CMMZ1+"        "+strPm_CMMZ1Tol+"        "+strNm_CMMZ1Tol
		+"        "+DevZ1Value+'\r'+'\n';
	strOutputStream1=strOutputStream1+"END DIM"+'\r'+'\n';

	string strOutputStream2 = "DIM CCM(";
	strOutputStream2 = strOutputStream2+m_TeachParameters.m_pointseril2+"),";
	strOutputStream2=strOutputStream2+strm_CMMX2+","+strm_CMMY1+","+strm_CMMZ1+'\r'+'\n';
	strOutputStream2=strOutputStream2+"AX        "+"NOMINAL        "+"+TOL        "+"-TOL        "+"DEV"+'\r'+'\n';
	strOutputStream2=strOutputStream2+"X         "+strm_CMMX2+"        "+strPm_CMMX2Tol+"        "+strNm_CMMX2Tol
		+"        "+DevX2Value+'\r'+'\n';
	strOutputStream2=strOutputStream2+"Y         "+strm_CMMY2+"        "+strPm_CMMY2Tol+"        "+strNm_CMMY2Tol
		+"        "+DevY2Value+'\r'+'\n';
	strOutputStream2=strOutputStream2+"Z         "+strm_CMMZ2+"        "+strPm_CMMZ2Tol+"        "+strNm_CMMZ2Tol
		+"        "+DevZ2Value+'\r'+'\n';
	strOutputStream2=strOutputStream2+"END DIM"+'\r'+'\n';

	/////// 先写
	USES_CONVERSION;
	WCHAR *WSavePath=A2W(SavePath.c_str());
	WCHAR *WOutputStream1=A2W(strOutputStream1.c_str());
	WCHAR *WOutputStream2=A2W(strOutputStream2.c_str());
	ofstream WFile;
	if (_access(SavePath.c_str(),02)!=-1)////// 文件夹存在则不写入
	{
		WFile.open(WSavePath,ios::app|ios::out|ios::binary);
		if (WFile.is_open())
		{
			WFile.write((char *)WOutputStream1,strOutputStream1.length()*2);
			WFile.write((char *)WOutputStream2,strOutputStream2.length()*2);
			WFile.close();
			return true;
		}
	} 
	else
	{
		WFile.open(WSavePath,ios::app|ios::out|ios::binary);
		if (WFile.is_open())
		{
			WFile.write("\xFF\xFE",2); 
			WFile.write((char *)WOutputStream1,strOutputStream1.length()*2);
			WFile.write((char *)WOutputStream2,strOutputStream2.length()*2);
			WFile.close();
			return true;
		}
	}
	return false;
}




#pragma region////系统标定所需的函数

Mat C3DLaserSensorRSerialIO::ReadInitialParametersCalib()
{
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	string strInitialParameters="Parameters\\CalibrationDatas\\CalibDatas\\InitialParameters.txt";
	USES_CONVERSION;   
	WCHAR*WFileName=A2W(strInitialParameters.c_str());
	ifstream RInitialParameters;
	RInitialParameters.open(WFileName);
	Mat InitialParameters(1,30,CV_64FC1,Scalar(0));
	Mat defaultInitialParameters(1,1,CV_64FC1,Scalar(0));
	if (RInitialParameters.is_open())
	{
		for (int i=0;i<InitialParameters.rows;i++)
		{
			for (int j=0;j<InitialParameters.cols;j++)
			{
				RInitialParameters>>InitialParameters.at<double>(i,j);
			}

		} 
		RInitialParameters.close();
	} 
	else
	{
		WriteLogCalib("Error:ReadInitialParametersCalib标定初始参数读取失败");
		return defaultInitialParameters;
	}
	return InitialParameters;
}

Mat C3DLaserSensorRSerialIO::ReadObservations()
{
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	Mat Observations;
	Mat DefaultBaseEyeData(1,1,CV_64FC1,Scalar(1));
	Mat SingleObservations(1,4,CV_64FC1,Scalar(1));
	string strObservations="Parameters\\CalibrationDatas\\CalibDatas\\Observations.txt";    
	USES_CONVERSION;
	WCHAR *WstrObservations =A2W(strObservations.c_str());
	ifstream RWstrObservations;
	RWstrObservations.open(WstrObservations);
	if (RWstrObservations.is_open())
	{
		while(!RWstrObservations.eof())
		{
			for (int j=0; j<3; j++)
			{
				RWstrObservations>>SingleObservations.at<double>(0,j);
			}

			if (RWstrObservations.fail())
			{
				break;
			}
			else
			{
				Observations.push_back(SingleObservations);
			}
		}		
		RWstrObservations.close();		
	}
	else
	{
		WriteLog("Error:ReadObservations标定车型和分组号路径错误");	
		return DefaultBaseEyeData;		
	}
	return Observations;
}


Mat C3DLaserSensorRSerialIO::ReadNominalData(int Num)
{
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	string strNominalData="Parameters\\CalibrationDatas\\CalibDatas\\NominalData.txt";
	USES_CONVERSION;   
	WCHAR*WFileName=A2W(strNominalData.c_str());
	ifstream RNominalData;
	RNominalData.open(WFileName);
	Mat NominalData(4,4,CV_64FC1,Scalar(1));
	Mat defaultNominalData(1,1,CV_64FC1,Scalar(0));
	if (RNominalData.is_open())
	{
		for (int i=0;i<NominalData.cols;i++)
		{
			for (int j=0;j<3;j++)
			{
				RNominalData>>NominalData.at<double>(j,i);
			}
		}
		RNominalData.close();
	} 
	else
	{
		WriteLog("Error:ReadNominalData标定名义值读取失败");
		return defaultNominalData;
	}

	switch(Num)
	{
	case 1:
		return NominalData.col(0).t();
	case 2:
		return NominalData.col(1).t();
	case 3:
		return NominalData.col(2).t();
	case 4:
		return NominalData.col(3).t();
	default:
		return defaultNominalData;
	}
}


Mat C3DLaserSensorRSerialIO::ReadBaseEyeData(int flag)
{
	//////如果文件被占用则进行等待操作
	Mat DefaultBaseEyeData(1,1,CV_64FC1,Scalar(0));
	int Ncount=0;
	while(1)
	{
		if (IsFileOpen)
		{
			Sleep(10);
			Ncount++;
			if (Ncount>200)/////等待2s钟
			{
				WriteLog("Error:ReadBaseEyeData文件被占用超过2秒，无法读取文件");
				Ncount=0;
				return DefaultBaseEyeData;
			}
			continue;
		}
		break;
	}

	IsFileOpen=true;
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	Mat BaseEyeData;
	Mat SingleBaseEyeData(1,16,CV_64FC1,Scalar(0));
	string strBaseEyePath=" ";

#pragma region//////BaseEye参数
	switch(flag)
	{
	case 1:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\1.txt";
		break;
	case 2:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\2.txt";
		break;
	case 3:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\3.txt";
		break;
	case 4:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\4.txt";
		break;
	case 11:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\1.txt";
		break;
	case 12:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\2.txt";
		break;
	case 13:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\3.txt";
		break;
	case 14:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\4.txt";
		break;
	case 21:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\1.txt";
		break;
	case 22:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\2.txt";
		break;
	case 23:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\3.txt";
		break;
	case 24:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\4.txt";
		break;
	case 31:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\1.txt";
		break;
	case 32:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\2.txt";
		break;
	case 33:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\3.txt";
		break;
	case 34:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\4.txt";
		break;
	case 41:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\1.txt";
		break;
	case 42:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\2.txt";
		break;
	case 43:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\3.txt";
		break;
	case 44:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\4.txt";
		break;
	case 51:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\1.txt";
		break;
	case 52:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\2.txt";
		break;
	case 53:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\3.txt";
		break;
	case 54:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\4.txt";
		break;
	case 61:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\1.txt";
		break;
	case 62:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\2.txt";
		break;
	case 63:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\3.txt";
		break;
	case 64:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\4.txt";
		break;
	case 71:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\1.txt";
		break;
	case 72:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\2.txt";
		break;
	case 73:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\3.txt";
		break;
	case 74:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\4.txt";
		break;
	case 81:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\1.txt";
		break;
	case 82:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\2.txt";
		break;
	case 83:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\3.txt";
		break;
	case 84:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\4.txt";
		break;
	default:
		WriteLog("Error:ReadBaseEyeData车型和分组号传输错误");
		return DefaultBaseEyeData;
	}
#pragma endregion

	USES_CONVERSION;
	WCHAR *WstrBaseEyePath =A2W(strBaseEyePath.c_str());
	ifstream RWstrBaseEyePath;
	RWstrBaseEyePath.open(WstrBaseEyePath);
	if (RWstrBaseEyePath.is_open())
	{
		while(!RWstrBaseEyePath.eof())
		{
			for (int j=0; j<SingleBaseEyeData.cols; j++)
			{
				RWstrBaseEyePath>>SingleBaseEyeData.at<double>(0,j);
			}

			if (RWstrBaseEyePath.fail())
			{
				break;
			}
			else
			{
				BaseEyeData.push_back(SingleBaseEyeData);
			}
		}		
		RWstrBaseEyePath.close();		
	}
	else
	{
		WriteLog("Error:ReadBaseEyeData标定车型和分组号路径错误");	
		IsFileOpen=false;
		return DefaultBaseEyeData;		
	}
	IsFileOpen=false;
	return BaseEyeData;
}

Mat C3DLaserSensorRSerialIO::ReadMeasureThetaCalib(int flag)
{
	//////如果文件被占用则进行等待操作
	Mat DefaultMeasureThetaData(1,1,CV_64FC1,Scalar(0));
	int Ncount=0;
	while(1)
	{
		if (IsThetaFileOpen)
		{
			Sleep(10);
			Ncount++;
			if (Ncount>200)/////等待2s钟
			{
				WriteLogCalib("Error:ReadMeasureThetaCalib读取关节角文件被占用超过2秒，无法读取文件");
				Ncount=0;
				return DefaultMeasureThetaData;
			}
			continue;
		}
		break;
	}

	IsThetaFileOpen=true;
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	Mat MeasureThetaData;
	Mat SingleMeasureThetaData(1,6,CV_64FC1,Scalar(0));
	string strMeasureThetaPath=" ";

#pragma region///// 关节角参数
	switch(flag)
	{
	case 1:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CalibTarget\\1.txt";
		break;
	case 2:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CalibTarget\\2.txt";
		break;
	case 3:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CalibTarget\\3.txt";
		break;
	case 4:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CalibTarget\\4.txt";
		break;
	case 11:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType1\\1.txt";
		break;
	case 12:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType1\\2.txt";
		break;
	case 13:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType1\\3.txt";
		break;
	case 14:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType1\\4.txt";
		break;
	case 21:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType2\\1.txt";
		break;
	case 22:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType2\\2.txt";
		break;
	case 23:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType2\\3.txt";
		break;
	case 24:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType2\\4.txt";
		break;
	case 31:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType3\\1.txt";
		break;
	case 32:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType3\\2.txt";
		break;
	case 33:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType3\\3.txt";
		break;
	case 34:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType3\\4.txt";
		break;
	case 41:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType4\\1.txt";
		break;
	case 42:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType4\\2.txt";
		break;
	case 43:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType4\\3.txt";
		break;
	case 44:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType4\\4.txt";
		break;
	case 51:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType5\\1.txt";
		break;
	case 52:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType5\\2.txt";
		break;
	case 53:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType5\\3.txt";
		break;
	case 54:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType5\\4.txt";
		break;
	case 61:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType6\\1.txt";
		break;
	case 62:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType6\\2.txt";
		break;
	case 63:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType6\\3.txt";
		break;
	case 64:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType6\\4.txt";
		break;
	case 71:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType7\\1.txt";
		break;
	case 72:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType7\\2.txt";
		break;
	case 73:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType7\\3.txt";
		break;
	case 74:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType7\\4.txt";
		break;
	case 81:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType8\\1.txt";
		break;
	case 82:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType8\\2.txt";
		break;
	case 83:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType8\\3.txt";
		break;
	case 84:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType8\\4.txt";
		break;
	default:
		WriteLogCalib("Error:ReadMeasureThetaCalib车型和分组号传输错误");
		IsThetaFileOpen=false;
		return DefaultMeasureThetaData;
	}
#pragma endregion

	USES_CONVERSION;
	WCHAR *WstrMeasureThetaPath =A2W(strMeasureThetaPath.c_str());
	ifstream RWstrMeasureThetaPath;
	RWstrMeasureThetaPath.open(WstrMeasureThetaPath);
	if (RWstrMeasureThetaPath.is_open())
	{
		while(!RWstrMeasureThetaPath.eof())
		{
			for (int j=0; j<SingleMeasureThetaData.cols; j++)
			{
				RWstrMeasureThetaPath>>SingleMeasureThetaData.at<double>(0,j);
			}

			if (RWstrMeasureThetaPath.fail())
			{
				break;
			}
			else
			{
				MeasureThetaData.push_back(SingleMeasureThetaData);
			}
		}


		RWstrMeasureThetaPath.close();		
	}
	else
	{
		WriteLogCalib("Error:ReadMeasureThetaCalib标定车型和分组号关节角路径错误");	
		IsThetaFileOpen=false;
		return DefaultMeasureThetaData;		
	}
	IsThetaFileOpen=false;
	return MeasureThetaData;
}

Mat C3DLaserSensorRSerialIO::ReadMeasureTheta(int flag)
{
	//////如果文件被占用则进行等待操作
	Mat DefaultMeasureThetaData(1,1,CV_64FC1,Scalar(0));
	int Ncount=0;
	while(1)
	{
		if (IsThetaFileOpen)
		{
			Sleep(10);
			Ncount++;
			if (Ncount>200)/////等待2s钟
			{
				WriteLog("Error:ReadMeasureTheta读取关节角文件被占用超过2秒，无法读取文件");
				Ncount=0;
				return DefaultMeasureThetaData;
			}
			continue;
		}
		break;
	}

	IsThetaFileOpen=true;
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	Mat MeasureThetaData;
	Mat SingleMeasureThetaData(1,6,CV_64FC1,Scalar(0));
	string strMeasureThetaPath=" ";

#pragma region///// 关节角参数
	switch(flag)
	{
	case 1:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CalibTarget\\1.txt";
		break;
	case 2:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CalibTarget\\2.txt";
		break;
	case 3:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CalibTarget\\3.txt";
		break;
	case 4:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CalibTarget\\4.txt";
		break;
	case 11:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType1\\1.txt";
		break;
	case 12:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType1\\2.txt";
		break;
	case 13:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType1\\3.txt";
		break;
	case 14:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType1\\4.txt";
		break;
	case 21:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType2\\1.txt";
		break;
	case 22:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType2\\2.txt";
		break;
	case 23:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType2\\3.txt";
		break;
	case 24:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType2\\4.txt";
		break;
	case 31:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType3\\1.txt";
		break;
	case 32:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType3\\2.txt";
		break;
	case 33:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType3\\3.txt";
		break;
	case 34:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType3\\4.txt";
		break;
	case 41:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType4\\1.txt";
		break;
	case 42:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType4\\2.txt";
		break;
	case 43:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType4\\3.txt";
		break;
	case 44:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType4\\4.txt";
		break;
	case 51:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType5\\1.txt";
		break;
	case 52:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType5\\2.txt";
		break;
	case 53:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType5\\3.txt";
		break;
	case 54:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType5\\4.txt";
		break;
	case 61:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType6\\1.txt";
		break;
	case 62:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType6\\2.txt";
		break;
	case 63:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType6\\3.txt";
		break;
	case 64:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType6\\4.txt";
		break;
	case 71:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType7\\1.txt";
		break;
	case 72:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType7\\2.txt";
		break;
	case 73:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType7\\3.txt";
		break;
	case 74:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType7\\4.txt";
		break;
	case 81:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType8\\1.txt";
		break;
	case 82:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType8\\2.txt";
		break;
	case 83:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType8\\3.txt";
		break;
	case 84:
		strMeasureThetaPath="Parameters\\CalibrationDatas\\MeasureTheta\\CarType8\\4.txt";
		break;
	default:
		WriteLog("Error:ReadMeasureTheta车型和分组号传输错误");
		IsThetaFileOpen=false;
		return DefaultMeasureThetaData;
	}
#pragma endregion

	USES_CONVERSION;
	WCHAR *WstrMeasureThetaPath =A2W(strMeasureThetaPath.c_str());
	ifstream RWstrMeasureThetaPath;
	RWstrMeasureThetaPath.open(WstrMeasureThetaPath);
	if (RWstrMeasureThetaPath.is_open())
	{
		while(!RWstrMeasureThetaPath.eof())
		{
			for (int j=0; j<SingleMeasureThetaData.cols; j++)
			{
				RWstrMeasureThetaPath>>SingleMeasureThetaData.at<double>(0,j);
			}

			if (RWstrMeasureThetaPath.fail())
			{
				break;
			}
			else
			{
				MeasureThetaData.push_back(SingleMeasureThetaData);
			}
		}


		RWstrMeasureThetaPath.close();		
	}
	else
	{
		WriteLog("Error:ReadMeasureTheta标定车型和分组号关节角路径错误");	
		IsThetaFileOpen=false;
		return DefaultMeasureThetaData;		
	}
	IsThetaFileOpen=false;
	return MeasureThetaData;
}

bool C3DLaserSensorRSerialIO::WriteInitialParametersCalib(Mat InitialParameters)
{
	Mat FloatInitialParameters;
	//////先进行类型转换
	if (InitialParameters.depth()==6)
	{
		FloatInitialParameters=DoubleToFloat(InitialParameters);
	}
	else
	{
		FloatInitialParameters=InitialParameters;
	}
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	string strInitialParameters="Parameters\\CalibrationDatas\\CalibDatas\\InitialParameters.txt";
	CreFiles(strInitialParameters);
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	USES_CONVERSION;   
	WCHAR *WSavePath=A2W(strInitialParameters.c_str());
	ofstream WInitialParameters;
	WInitialParameters.open(WSavePath,ios_base::out|ios_base::trunc);
	if (WInitialParameters.is_open())
	{
		for (int i=0;i<FloatInitialParameters.rows;i++)
		{
			for (int j=0;j<FloatInitialParameters.cols;j++)
			{
				WInitialParameters<<FloatInitialParameters.at<float>(i,j)<<' ';
			}
			WInitialParameters<<'\n';
		}
		WInitialParameters.close();
		return true;
	} 
	else
	{
		WriteLogCalib("Error:WriteInitialParametersCalib存储标定后系统参数失败");
		return false;
	}
}

bool C3DLaserSensorRSerialIO::WriteBaseEyeData(int flag,Mat BaseEyeData)
{
	///////
	int Ncount=0;
	while(1)
	{
		if (IsFileOpen)
		{
			Sleep(10);
			Ncount++;
			if (Ncount>200)/////等待2s钟
			{
				WriteLog("Error:WriteBaseEyeData文件被占用超过2秒，无法写入文件");
				Ncount=0;
				return false;
			}
			continue;
		}
		break;
	}
	IsFileOpen=true;
	////////类型转换
	Mat FloatBaseEyeData;
	if (BaseEyeData.depth()==6)
	{
		FloatBaseEyeData=DoubleToFloat(BaseEyeData);
	} 
	else
	{
		FloatBaseEyeData=BaseEyeData;
	}
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	string strBaseEyePath=" ";

#pragma region////// 写BaseEye
	switch(flag)
	{
	case 1:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\1.txt";
		break;
	case 2:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\2.txt";
		break;
	case 3:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\3.txt";
		break;
	case 4:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\4.txt";
		break;
	case 11:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\1.txt";
		break;
	case 12:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\2.txt";
		break;
	case 13:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\3.txt";
		break;
	case 14:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\4.txt";
		break;
	case 21:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\1.txt";
		break;
	case 22:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\2.txt";
		break;
	case 23:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\3.txt";
		break;
	case 24:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\4.txt";
		break;
	case 31:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\1.txt";
		break;
	case 32:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\2.txt";
		break;
	case 33:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\3.txt";
		break;
	case 34:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\4.txt";
		break;
	case 41:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\1.txt";
		break;
	case 42:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\2.txt";
		break;
	case 43:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\3.txt";
		break;
	case 44:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\4.txt";
		break;
	case 51:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\1.txt";
		break;
	case 52:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\2.txt";
		break;
	case 53:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\3.txt";
		break;
	case 54:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\4.txt";
		break;
	case 61:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\1.txt";
		break;
	case 62:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\2.txt";
		break;
	case 63:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\3.txt";
		break;
	case 64:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\4.txt";
		break;
	case 71:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\1.txt";
		break;
	case 72:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\2.txt";
		break;
	case 73:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\3.txt";
		break;
	case 74:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\4.txt";
		break;
	case 81:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\1.txt";
		break;
	case 82:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\2.txt";
		break;
	case 83:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\3.txt";
		break;
	case 84:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\4.txt";
		break;
	default:
		WriteLog("Error:WriteBaseEyeData车型和分组号传输错误");
		IsFileOpen=false;
		return false;
	}
#pragma endregion

	CreFiles(strBaseEyePath);
	_chdir(DefaultWorkingPath);
	USES_CONVERSION;
	WCHAR *WstrBaseEyePath =A2W(strBaseEyePath.c_str());
	ofstream RWstrBaseEyePath;
	RWstrBaseEyePath.open(WstrBaseEyePath,ios_base::out|ios_base::trunc);
	if (RWstrBaseEyePath.is_open())
	{
		for (int i=0; i<FloatBaseEyeData.rows; i++)
		{
			for (int j=0; j<FloatBaseEyeData.cols; j++)
			{
				RWstrBaseEyePath<<FloatBaseEyeData.at<float>(i,j)<<' ';
			}
			RWstrBaseEyePath<<'\n';
		}

		RWstrBaseEyePath.close();
		IsFileOpen=false;
		return true;
	}
	else
	{
		WriteLog("Error:WriteBaseEyeData标定车型和分组号路径错误");	
		IsFileOpen=false;
		return false;		
	}
}


bool C3DLaserSensorRSerialIO::WriteBaseEyeDataCalib(int flag,Mat BaseEyeData) ////
{
	///////
	int Ncount=0;
	while(1)
	{
		if (IsFileOpen)
		{
			Sleep(10);
			Ncount++;
			if (Ncount>200)/////等待2s钟
			{
				WriteLogCalib("Error:WriteBaseEyeData文件被占用超过2秒，无法写入文件");
				Ncount=0;
				return false;
			}
			continue;
		}
		break;
	}
	IsFileOpen=true;
	////////类型转换
	Mat FloatBaseEyeData;
	if (BaseEyeData.depth()==6)
	{
		FloatBaseEyeData=DoubleToFloat(BaseEyeData);
	} 
	else
	{
		FloatBaseEyeData=BaseEyeData;
	}
	_chdir(DefaultWorkingPath);/////将保存的路径设置为当前路径
	string strBaseEyePath=" ";

#pragma region////// 写BaseEye
	switch(flag)
	{
	case 1:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\1.txt";
		break;
	case 2:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\2.txt";
		break;
	case 3:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\3.txt";
		break;
	case 4:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CalibTarget\\4.txt";
		break;
	case 11:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\1.txt";
		break;
	case 12:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\2.txt";
		break;
	case 13:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\3.txt";
		break;
	case 14:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType1\\4.txt";
		break;
	case 21:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\1.txt";
		break;
	case 22:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\2.txt";
		break;
	case 23:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\3.txt";
		break;
	case 24:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType2\\4.txt";
		break;
	case 31:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\1.txt";
		break;
	case 32:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\2.txt";
		break;
	case 33:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\3.txt";
		break;
	case 34:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType3\\4.txt";
		break;
	case 41:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\1.txt";
		break;
	case 42:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\2.txt";
		break;
	case 43:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\3.txt";
		break;
	case 44:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType4\\4.txt";
		break;
	case 51:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\1.txt";
		break;
	case 52:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\2.txt";
		break;
	case 53:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\3.txt";
		break;
	case 54:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType5\\4.txt";
		break;
	case 61:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\1.txt";
		break;
	case 62:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\2.txt";
		break;
	case 63:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\3.txt";
		break;
	case 64:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType6\\4.txt";
		break;
	case 71:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\1.txt";
		break;
	case 72:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\2.txt";
		break;
	case 73:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\3.txt";
		break;
	case 74:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType7\\4.txt";
		break;
	case 81:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\1.txt";
		break;
	case 82:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\2.txt";
		break;
	case 83:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\3.txt";
		break;
	case 84:
		strBaseEyePath="Parameters\\CalibrationDatas\\BaseEye\\CarType8\\4.txt";
		break;
	default:
		WriteLogCalib("Error:WriteBaseEyeData车型和分组号传输错误");
		IsFileOpen=false;
		return false;
	}
#pragma endregion

	CreFiles(strBaseEyePath);
	_chdir(DefaultWorkingPath);
	USES_CONVERSION;
	WCHAR *WstrBaseEyePath =A2W(strBaseEyePath.c_str());
	ofstream RWstrBaseEyePath;
	RWstrBaseEyePath.open(WstrBaseEyePath,ios_base::out|ios_base::trunc);
	if (RWstrBaseEyePath.is_open())
	{
		for (int i=0; i<FloatBaseEyeData.rows; i++)
		{
			for (int j=0; j<FloatBaseEyeData.cols; j++)
			{
				RWstrBaseEyePath<<FloatBaseEyeData.at<float>(i,j)<<' ';
			}
			RWstrBaseEyePath<<'\n';
		}

		RWstrBaseEyePath.close();
		IsFileOpen=false;
		return true;
	}
	else
	{
		WriteLogCalib("Error:WriteBaseEyeData标定车型和分组号路径错误");	
		IsFileOpen=false;
		return false;		
	}
}


Mat C3DLaserSensorRSerialIO::DataTranslateOneRow(Mat HighRowsData)
{
	Mat OneRowData(1,16,CV_64FC1,Scalar(0));
	Mat DoubleHighRowsData;
	if (HighRowsData.depth()==5)
	{
		DoubleHighRowsData=FloatToDouble(HighRowsData);
	} 
	else
	{
		DoubleHighRowsData=HighRowsData;
	}
	for (int i=0; i<DoubleHighRowsData.rows; i++)
	{
		for (int j=0; j<DoubleHighRowsData.cols; j++)
		{
			OneRowData.at<double>(0,i*DoubleHighRowsData.cols+j)=DoubleHighRowsData.at<double>(i,j);
		}
	}
	return OneRowData;
}

Mat C3DLaserSensorRSerialIO::DataTranslateFourRow(Mat LowRowsData)
{
	Mat FourRow(4,4,CV_64FC1,Scalar(0));
	Mat DoubleLowRowsData;
	if (LowRowsData.depth()==5)
	{
		DoubleLowRowsData=FloatToDouble(LowRowsData);
	} 
	else
	{
		DoubleLowRowsData=LowRowsData;
	}

	for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			FourRow.at<double>(i,j)=DoubleLowRowsData.at<double>(0,i*4+j);
		}
	}
	return FourRow;
}

Mat C3DLaserSensorRSerialIO::DataTranlateHom(Point3f SphereCenters)
{
	Mat HomSphereCenters(1,4,CV_64FC1,Scalar(1));
	HomSphereCenters.at<double>(0,0)=(double)SphereCenters.x;
	HomSphereCenters.at<double>(0,1)=(double)SphereCenters.y;
	HomSphereCenters.at<double>(0,2)=(double)SphereCenters.z;
	HomSphereCenters.at<double>(0,3)=1.0;
	return HomSphereCenters;
}

Point3f C3DLaserSensorRSerialIO::EyeToBase(Mat BaseEye,Point3f Center)///////计算类型为float类型
{
	Mat DoubleBaseEye;
	if (BaseEye.rows!=4 || BaseEye.cols!=4)
	{
		return Center;
	}

    if (BaseEye.depth()==5)
    {
		DoubleBaseEye=FloatToDouble(BaseEye);
    } 
    else
    {
		DoubleBaseEye=BaseEye;
    }

	Mat matCenter=DoubleBaseEye*DataTranlateHom(Center).t();
	Point3f ptCenter;
	ptCenter.x=(float)matCenter.at<double>(0,0);
	ptCenter.y=(float)matCenter.at<double>(1,0);
	ptCenter.z=(float)matCenter.at<double>(2,0);
	return ptCenter;
}

Mat C3DLaserSensorRSerialIO::DoubleToFloat(Mat DoubleData)
{
	Mat FloatData(DoubleData.rows,DoubleData.cols,CV_32FC1,Scalar(0));
	for (int i=0; i<DoubleData.rows; i++)
	{
		for (int j=0; j<DoubleData.cols; j++)
		{
			FloatData.at<float>(i,j) = (float)DoubleData.at<double>(i,j);
		}
	}
	return FloatData;
}

Mat C3DLaserSensorRSerialIO::FloatToDouble(Mat FloatData)
{
	Mat DoubleData(FloatData.rows,FloatData.cols,CV_64FC1,Scalar(0));
	for (int i=0; i<FloatData.rows; i++)
	{
		for (int j=0; j<FloatData.cols; j++)
		{
			DoubleData.at<double>(i,j) = (double)FloatData.at<float>(i,j);
		}
	}
	return DoubleData;
}


bool C3DLaserSensorRSerialIO::IsCalibration(Mat MatErrorData,double Rate,double RateValue)//////MatErrorData 为double类型
{
	Mat DoubleMatErrorData;

	if (MatErrorData.depth()==5)
	{
		DoubleMatErrorData=FloatToDouble(MatErrorData);
	} 
	else
	{
		DoubleMatErrorData=MatErrorData;
	}

	float nCount=0;
	for (int i=0; i<DoubleMatErrorData.rows; i++)
	{
		for (int j=0;j<3 && j<DoubleMatErrorData.cols; j++)
		{
			if (std::abs(DoubleMatErrorData.at<double>(i,j))>=RateValue)
			{
				nCount++;
			}
		}
	}

	if ((nCount/(DoubleMatErrorData.rows*3))>Rate)
	{
		WriteLog("IsCalibration系统需标定:");///// 日志在main函数中顺序执行
		return true;
	}
	WriteLog("IsCalibration系统无需标定:");
	return false;
}
#pragma endregion 