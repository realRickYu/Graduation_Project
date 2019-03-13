#include "C3DLaserSensorFeatureExtractionV20.0.h"
#include "circlefitsolver.h"

C3DLaserSensorFeatureExtraction::C3DLaserSensorFeatureExtraction(void)
{
	m_SphereCenter=Point3f(0,0,0);/////////初始化为0
	m_BaseEye=Mat::eye(4,4,CV_64FC1);/////初始化为4*4的单位矩阵
}

C3DLaserSensorFeatureExtraction::~C3DLaserSensorFeatureExtraction(void)
{
}


#pragma region  /////通用处理算法

#pragma  region ////// 光刀提取基础函数的实现
//////自适应光刀中心提取函数
///////将提取的轮廓按照周长由大到小排序
bool C3DLaserSensorFeatureExtraction::LengthDown(vector<Point> elem1,vector<Point> elem2)////std::sort的第三个参数
{
	return arcLength(elem1,true)>arcLength(elem2,true);/////按轮廓线的周长排序 modified in Version 4
	//arcLength calculates a contour perimeter or a curve length.
}

//////////////修正感兴趣区域
Rect C3DLaserSensorFeatureExtraction::ModifyROI(Mat ImgGray,Rect OriginalROI,int Threshold/* =150 */)
{
	////Case 1: 如果OriginalROI.x=1,可能与图片左端相交
	if (1==OriginalROI.x)
	{
		for (int i=0;i<ImgGray.rows;i++)
		{
			if ((float)ImgGray.at<uchar>(i,0)>Threshold)
			{
				OriginalROI.x=0;
				OriginalROI.width=OriginalROI.width+1;
				break;
			}

		}

	}
	/////// Case 2:连通域可能与图片上端相交
	if (1==OriginalROI.y)
	{
		for (int j=0;j<ImgGray.cols;j++)
		{
			if ((float)ImgGray.at<uchar>(0,j)>Threshold)
			{
				OriginalROI.y=0;
				OriginalROI.height=OriginalROI.height+1;
				break;
			}
		}
	}

	///////// Case 3: 连通域可能与图片下端相交 ///////可能出问题

	if ((ImgGray.rows-2)==(OriginalROI.y+OriginalROI.height-1))
	{
		for (int j=0;j<ImgGray.cols;j++)
		{
			if ((float)ImgGray.at<uchar>(ImgGray.rows-1,j)>Threshold)
			{
				OriginalROI.height=OriginalROI.height+1;
				break;
			}
		}                    
	}
	////////// Case 4: 连通域可能与图片右端相交
	if ((ImgGray.cols-2)==(OriginalROI.x+OriginalROI.width-1))
	{
		for (int i=0;i<ImgGray.rows;i++)
		{
			if ((float)ImgGray.at<uchar>(i,ImgGray.cols-1)>Threshold)
			{
				OriginalROI.width=OriginalROI.width+1;
				break;
			}

		}

	}
	return OriginalROI;
}


////////////扩展感兴趣区域,在不超过图片边界的情况下
/////////// 向上向下扩展一定的像素(上下分别扩展10个像素肯定够了)
Rect C3DLaserSensorFeatureExtraction::ExtendedROI(const Mat &ImgGray,const Rect &OutRect,int PixelNum/*=10*/)
{
	Rect ROIRect;
	ROIRect.y=(OutRect.y-10>0)?(OutRect.y-10):(0);
	ROIRect.x=OutRect.x;
	ROIRect.width=OutRect.width;
	ROIRect.height=(ROIRect.y+OutRect.height+20>ImgGray.rows)?(ImgGray.rows-ROIRect.y):(OutRect.height+20);
	return ROIRect;
}

//////////// 确定初始光刀线的中心位置
Mat C3DLaserSensorFeatureExtraction::OriginalCenter(const Mat &LaserStripSmoothed)
{
	double SumMax(0);
	double SumVaue(0);
	const int SumNum=5;
	Mat Index(1,LaserStripSmoothed.cols,CV_64FC1,Scalar::all(-1));////  Index存储光刀线的初始中心位置
	for (int ColNum=0;ColNum<LaserStripSmoothed.cols;ColNum++)
	{
		SumMax=0;
		for (int RowsNum=0;RowsNum<LaserStripSmoothed.rows-SumNum;RowsNum++)
		{
			SumVaue=LaserStripSmoothed.at<double>(RowsNum,ColNum)+LaserStripSmoothed.at<double>(RowsNum+1,ColNum)+LaserStripSmoothed.at<double>(RowsNum+2,ColNum)
				+LaserStripSmoothed.at<double>(RowsNum+3,ColNum)+LaserStripSmoothed.at<double>(RowsNum+4,ColNum);

			if (SumMax<SumVaue)
			{
				SumMax=SumVaue;
				Index.at<double>(0,ColNum)=(double)(RowsNum+3);
			}
		}
		if (Index.at<double>(0, ColNum) > (LaserStripSmoothed.rows - 1) || int(Index.at<double>(0, ColNum)) == -1)
		{
			Index.at<double>(0, ColNum) = LaserStripSmoothed.rows - 1;
		}
	}

	return Index;
}

/////////// 确定光刀线的边界灰度阈值，确定光刀线的宽度
Mat C3DLaserSensorFeatureExtraction::DetermineEdgeValue(const Mat &LaserStripSmoothed,const Mat &Index)
{
	/// 计算光刀线的边界阈值，确定光刀线的宽度(modified in version 4 2014年6月11日)
	Mat I(1,1,CV_64FC1,0);
	Mat AverageValue(1,LaserStripSmoothed.cols,CV_64FC1,Scalar::all(0));
	Mat ThresholdValue(1,LaserStripSmoothed.cols,CV_64FC1,Scalar::all(0));
	double SumBackG(0);
	int Point1Count(0);////第一次求平均值使用的点数
	int Point2Count(0);////第二次求平均值使用的点数			
	for(int CNum=0;CNum<LaserStripSmoothed.cols;CNum++)
	{
		Point1Count=0;////每次计算均进行初始化
		//////向下搜索15个像素值(包含初始中心一共16个数据点)
		for (int RNum=(int)Index.at<double>(0,CNum);(RNum<(int)Index.at<double>(0,CNum)+16)&&(RNum<LaserStripSmoothed.rows);RNum++)
		{		
			AverageValue.at<double>(0,CNum)+=LaserStripSmoothed.at<double>(RNum,CNum);
			Point1Count++;

		}
		//////向上搜索15个像素值
		for (int RNum=(int)Index.at<double>(0,CNum)-1;(RNum>(int)Index.at<double>(0,CNum)-16)&&(RNum>-1);RNum--)///// 此处为-1，主要是为了包含RNum=0的情况
		{		
			AverageValue.at<double>(0,CNum)+=LaserStripSmoothed.at<double>(RNum,CNum);
			Point1Count++;
		}
		if (Point1Count != 0)
		{
			AverageValue.at<double>(0,CNum)=AverageValue.at<double>(0,CNum)/Point1Count;
		} 
		else
		{
			return I;
			///////写错误日志
		}

	}

	/////// 第二次求均值
	for (int ColNum=0;ColNum<LaserStripSmoothed.cols;ColNum++)
	{
		SumBackG=0;
		Point2Count=0; 
		//////向下15个像素值(包含初始中心一共16个数据点)范围内二次求均值
		for (int RowsNum=(int)Index.at<double>(0,ColNum);(RowsNum<(int)Index.at<double>(0,ColNum)+16)&&(RowsNum<LaserStripSmoothed.rows);RowsNum++)
		{
			if (LaserStripSmoothed.at<double>(RowsNum,ColNum)<=AverageValue.at<double>(0,ColNum))
			{
				SumBackG+=LaserStripSmoothed.at<double>(RowsNum,ColNum);
				Point2Count++;
			} 
		}
		///////上15个像素值(不包含初始中心一共15个数据点)范围内二次求平均值
		for (int RowsNum=(int)Index.at<double>(0,ColNum)-1;(RowsNum>(int)Index.at<double>(0,ColNum)-16)&&(RowsNum>-1);RowsNum--)
		{
			if (LaserStripSmoothed.at<double>(RowsNum,ColNum)<=AverageValue.at<double>(0,ColNum))
			{
				SumBackG+=LaserStripSmoothed.at<double>(RowsNum,ColNum);
				Point2Count++;
			} 
		}  
		if (0!=Point2Count)///// 求边界阈值
		{
			ThresholdValue.at<double>(0,ColNum)=SumBackG/Point2Count;
		} 
		else
		{
			return I;/////写错误日志
		}
	}
	return ThresholdValue;
}

////////////
Mat C3DLaserSensorFeatureExtraction::DetermineCenters(const Mat &LaserStripSmoothed, const Mat &Index, const Mat &ThresholdValue, const Rect &ROIRect)
{
	//// 利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
	Mat  Center(1,2,CV_64FC1,Scalar::all(0));
	Mat LaserStripeCenters; 
	double SumWeightValue(0);
	double SumGrayValue(0);
	for (int ColNum=0;ColNum<LaserStripSmoothed.cols;ColNum++)
	{
		SumGrayValue=0;
		SumWeightValue=0;
		//////向下寻找光刀线的边界
		for (int RowsNum=(int)Index.at<double>(0,ColNum);(RowsNum<LaserStripSmoothed.rows)&&(RowsNum<(int)Index.at<double>(0,ColNum)+16);RowsNum++)
		{
			if (LaserStripSmoothed.at<double>(RowsNum,ColNum)>ThresholdValue.at<double>(0,ColNum))
			{
				SumGrayValue+=LaserStripSmoothed.at<double>(RowsNum,ColNum);
				SumWeightValue+=(ROIRect.y+RowsNum)*LaserStripSmoothed.at<double>(RowsNum,ColNum);
			} 
			else
			{
				break;
			}
		}
		//////向上寻找光刀线的边界
		for (int RowsNum=(int)Index.at<double>(0,ColNum)-1;(RowsNum>-1)&&(RowsNum>(Index.at<double>(0,ColNum)-16));RowsNum--)
		{
			if (LaserStripSmoothed.at<double>(RowsNum,ColNum)>ThresholdValue.at<double>(0,ColNum))
			{
				SumGrayValue+=LaserStripSmoothed.at<double>(RowsNum,ColNum);
				SumWeightValue+=(ROIRect.y+RowsNum)*LaserStripSmoothed.at<double>(RowsNum,ColNum);

			} 
			else
			{
				break;
			}
		}
		///// 灰度重心法计算光刀线的中心坐标值
		Center.at<double>(0,0)=SumWeightValue/SumGrayValue;
		Center.at<double>(0,1)=(double)(ROIRect.x+ColNum);
		LaserStripeCenters.push_back(Center);

	}
	return LaserStripeCenters;
}

//////// modified in 20150505 
Mat C3DLaserSensorFeatureExtraction::CameraCoorDatas(const Mat &ImageDatas, const Mat &CameraMatrix, const Mat &DistCoeffs, const Mat &LaserPlaneParas)
{
	/////////首先筛选不能反映被测特征的ImageDatas
	Mat DefaultDatas(1,3,CV_64FC1,Scalar(10000));
	Mat CameraCDatas;
	Mat DistortedPoints(1,1,CV_64FC2,cv::Scalar(0,0));
	Mat UndistortPoints;
	Mat ImgeCoor(4,1,CV_64FC1,Scalar(0));///// 图像坐标系
	Mat CameraCoor(4,1,CV_64FC1,Scalar(0));	
	Mat TranlateDatas(3,1,CV_64FC1,Scalar(0));
	Mat TranMatrix(4,4,CV_64FC1,Scalar(0));
	///////转换矩阵赋值
	CameraMatrix.copyTo(TranMatrix(Range(0,3),Range(0,3)));
	LaserPlaneParas.copyTo(TranMatrix(Range(3,4),Range(0,4)));////////考虑参数中将d计算出来
	for (int RowNum=0;RowNum<ImageDatas.rows;RowNum++)
	{
		///// 畸变校正
		DistortedPoints.at<Vec2d>(0,0)[0]=ImageDatas.at<double>(RowNum,1);
		DistortedPoints.at<Vec2d>(0,0)[1]=ImageDatas.at<double>(RowNum,0);
		////////
		undistortPoints(DistortedPoints,UndistortPoints,CameraMatrix,DistCoeffs,noArray(),CameraMatrix);
		////////////// 多通道转换成图像坐标系下的齐次坐标
		ImgeCoor.at<double>(0,0)=UndistortPoints.at<Vec2d>(0)[0];
		ImgeCoor.at<double>(1,0)=UndistortPoints.at<Vec2d>(0)[1];
		ImgeCoor.at<double>(2,0)=1;
		ImgeCoor.at<double>(3,0)=0;
		//////////////相机坐标系下的坐标值
		CameraCoor=TranMatrix.inv(DECOMP_SVD)*ImgeCoor;
		////////相机坐标下的齐次变换
		CameraCoor.row(0)= CameraCoor.row(0).mul(1/ CameraCoor.row(3));
		CameraCoor.row(1)= CameraCoor.row(1).mul(1/ CameraCoor.row(3));
		CameraCoor.row(2)= CameraCoor.row(2).mul(1/ CameraCoor.row(3));
		CameraCoor.row(3)= CameraCoor.row(3).mul(1/ CameraCoor.row(3));
		//////
		TranlateDatas=CameraCoor(Range(0,3),Range::all()).t();
		CameraCDatas.push_back(TranlateDatas);		
	}

	////////判断是否有数据生成
	if (CameraCDatas.empty())
	{

		return DefaultDatas;
	} 
	else
	{
		return CameraCDatas;

	}
}

void C3DLaserSensorFeatureExtraction::deleteRow(Mat& sample, int rowNum)
{
	if (rowNum < 0 || rowNum >= sample.rows)
	{
		cerr << "输入的行数越界" << endl;
		return;
	}

	Mat tempMat = sample.rowRange(rowNum + 1, sample.rows).clone();
	sample.pop_back(sample.rows - rowNum);
	sample.push_back(tempMat);
	~ tempMat;
	return;
}
#pragma endregion

#pragma region /////// 特征拟合所需的基本类的实现

bool C3DLaserSensorFeatureExtraction::PointValueAsc(Point2d pt1,Point2d pt2) //////升序排列
{
	return pt1.x<pt2.x;
}

////标准Hough圆检测去除异常点
Mat C3DLaserSensorFeatureExtraction::HoughCircle(Mat Camera3DData, Mat XYRoted, double min_r, double max_r, double step_r/* = 0.1 */, double step_theta/* = 0.01*/, double CircleMeanErrorThreshold/* =0.2 */,double CircleGrossErrorCoe/* =0.1 */,double CircleNormalErrorCoe/* =3 */)
{
	Mat MinMaxData(XYRoted.rows, XYRoted.cols, CV_64FC1, Scalar(1));
    cv::sort(XYRoted, MinMaxData, CV_SORT_EVERY_COLUMN|CV_SORT_ASCENDING);  //各列升序独立排列
	double max_X = MinMaxData.at<double>(MinMaxData.rows - 1, 0);
	double min_X = MinMaxData.at<double>(0, 0);
	double max_Y = MinMaxData.at<double>(MinMaxData.rows - 1, 1);
	double min_Y = MinMaxData.at<double>(0, 1);

	double length_X = max_X - min_X;
    double length_Y = max_Y - min_Y;

	//四分之一筛选法，原理不知 180516
	Mat XYRoted2;
	for (int i = 0; i != XYRoted.rows; i++)
	{
		if ((XYRoted.at<double>(i, 0) < (min_X + length_X / 4)) || (XYRoted.at<double>(i, 0) > (max_X - length_X / 4)) ||
			(XYRoted.at<double>(i, 1) < (min_Y + length_Y / 4)) || (XYRoted.at<double>(i, 1) > (max_Y - length_Y / 4)))
		{
			XYRoted2.push_back(XYRoted.row(i));
		}
	}

	int m = (cvCeil(max_X) - cvFloor(min_X) + 2 * cvCeil(max_r)) * 10;  //给hough圆检测最大半径预留空间
	int n = (cvCeil(max_Y) - cvFloor(min_Y) + 2 * cvCeil(max_r)) * 10;
    //平移至原点，并且增加一个最大检测半径
	XYRoted2.col(0) = XYRoted2.col(0) - min_X + max_r;
	XYRoted2.col(1) = XYRoted2.col(1) - min_Y + max_r;

	int size_theta = cvRound(2 * CV_PI / step_theta);
	int size_r = cvRound((max_r - min_r) / step_r);
	int size_r10 = cvRound(max_r * 10) + 1;
	//申请一个3维数组m*n*size_r10,并进行初始化操作
	int ***Hough_space;
	Hough_space = new int **[m];
	for (int i = 0; i < m; i++)
	{
		Hough_space[i] = new int *[n];
		for (int j = 0; j < n; j++)
		{
			Hough_space[i][j] = new int[size_r10];
			memset(Hough_space[i][j], 0, sizeof(int)*size_r10);
		}
	}

	for (int i = 0; i != XYRoted2.rows; i++)
	{
	    for (int j = 0; j != size_r + 1; j++)
	    {
			double r1 = min_r + j * step_r;
			for (int k = 0; k != size_theta; k++)
			{
				int a = cvRound((XYRoted2.at<double>(i, 0) + r1 * cos(k * step_theta)) * 10);
				int b = cvRound((XYRoted2.at<double>(i, 1) + r1 * sin(k * step_theta)) * 10);
				int r10 = cvRound(r1 * 10);
				if (a >= 0 && a <= m - 1 && b >= 0 && b <= n - 1 && r10 >= 0 && r10 <= size_r10 - 1)
					Hough_space[a][b][r10] = Hough_space[a][b][r10] + 1;
			}
	    }
	}

	//遍历Hough_space寻找最大值及其索引
	int X_index = 0;
	int Y_index = 0;
	int r_index = 0;
	int maxValue = 1;
	for (int i = 0; i < m; i++)
	{
	    for (int j = 0; j < n; j++)
	    {
			for (int k = 0; k < size_r10; k++)
			{
				if (maxValue < Hough_space[i][j][k])//180516 < change to <=
				{
					maxValue = Hough_space[i][j][k];
					X_index = i;
					Y_index = j;
					r_index = k;
				}
			}
	    }
	}

	//释放动态生成的内存
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			delete[]Hough_space[i][j];
		}
		delete[]Hough_space[i];
	}
	delete[]Hough_space;

	double x = double(X_index) / 10 + min_X - max_r;
	double y = double(Y_index) / 10 + min_Y - max_r;
	double r = double(r_index) / 10;

	Mat tempcircleresult(1, 4, CV_64FC1);
	tempcircleresult.at<double>(0, 0) = x;
	tempcircleresult.at<double>(0, 1) = y;
	tempcircleresult.at<double>(0, 2) = r;
	tempcircleresult.at<double>(0, 3) = maxValue;
	string houghresult = "houghcircle.txt";
	LaserSensorIO.WriteRowFirstAPP(houghresult, tempcircleresult);

	//根据Hough圆变换求出的初始中心，筛选出需要的点
	Mat error;
	Mat XDistance(XYRoted.rows, 1, CV_64FC1, cv::Scalar(1));
	Mat YDistance(XYRoted.rows, 1, CV_64FC1, cv::Scalar(1));
	XDistance = (XYRoted.col(0) - x).mul(XYRoted.col(0) - x);
	YDistance = (XYRoted.col(1) - y).mul(XYRoted.col(1) - y);
	cv::sqrt(XDistance + YDistance, error);
	error = error - r;

	Mat pickedXY;
	double MeanResidal = EliminateGrossError(error, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
	//去除异常点
	for (int cnt = 0; cnt < error.rows; cnt++)
	{
		if (std::abs(error.at<double>(cnt, 0)) < step_r)
		{
			pickedXY.push_back(Camera3DData.row(cnt));
		}
	}
	
	return pickedXY;
}

Mat C3DLaserSensorFeatureExtraction::HoughCircle(vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r/* = 0.1 */, double step_theta/* = 0.01*/, double CircleMeanErrorThreshold/* =0.2 */,double CircleGrossErrorCoe/* =0.1 */,double CircleNormalErrorCoe/* =3 */)
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));

	Mat MinMaxData(XYRoted.rows, XYRoted.cols, CV_64FC1, Scalar(1));
    cv::sort(XYRoted, MinMaxData, CV_SORT_EVERY_COLUMN|CV_SORT_ASCENDING);  //各列升序独立排列
	double max_X = MinMaxData.at<double>(MinMaxData.rows - 1, 0);
	double min_X = MinMaxData.at<double>(0, 0);
	double max_Y = MinMaxData.at<double>(MinMaxData.rows - 1, 1);
	double min_Y = MinMaxData.at<double>(0, 1);

	double length_X = max_X - min_X;
    double length_Y = max_Y - min_Y;
	Mat XYRoted2;
	for (int i = 0; i != XYRoted.rows; i++)
	{
		if ((XYRoted.at<double>(i, 0) < (min_X + length_X / 4)) || (XYRoted.at<double>(i, 0) > (max_X - length_X / 4)) ||
			(XYRoted.at<double>(i, 1) < (min_Y + length_Y / 4)) || (XYRoted.at<double>(i, 1) > (max_Y - length_Y / 4)))
		{
			XYRoted2.push_back(XYRoted.row(i));
		}
	}

	int m = (cvCeil(max_X) - cvFloor(min_X) + 2 * cvCeil(max_r)) * 10;  //给hough圆检测最大半径预留空间
	int n = (cvCeil(max_Y) - cvFloor(min_Y) + 2 * cvCeil(max_r)) * 10;
    //平移至原点，并且增加一个最大检测半径
	XYRoted2.col(0) = XYRoted2.col(0) - min_X + max_r;
	XYRoted2.col(1) = XYRoted2.col(1) - min_Y + max_r;

	int size_theta = cvRound(2 * CV_PI / step_theta);
	int size_r = cvRound((max_r - min_r) / step_r);
	int size_r100 = cvRound(max_r * 100) + 1;
	//申请一个3维数组m*n*size_r10,并进行初始化操作
	int ***Hough_space;
	Hough_space = new int **[m];
	for (int i = 0; i < m; i++)
	{
		Hough_space[i] = new int *[n];
		for (int j = 0; j < n; j++)
		{
			Hough_space[i][j] = new int[size_r100];
			memset(Hough_space[i][j], 0, sizeof(int)*size_r100);
		}
	}

	for (int i = 0; i != XYRoted2.rows; i++)
	{
	    for (int j = 0; j != size_r + 1; j++)
	    {
			double r1 = min_r + j * step_r;
			for (int k = 0; k != size_theta; k++)
			{
				int a = cvRound((XYRoted2.at<double>(i, 0) + r1 * cos(k * step_theta)) * 10);
				int b = cvRound((XYRoted2.at<double>(i, 1) + r1 * sin(k * step_theta)) * 10);
				int r100 = cvRound(r1 * 100);
				if (a >= 0 && a <= m - 1 && b >= 0 && b <= n - 1 && r100 >= 0 && r100 <= size_r100 - 1)
					Hough_space[a][b][r100] = Hough_space[a][b][r100] + 1;
			}
	    }
	}

	//遍历Hough_space寻找最大值及其索引
	int X_index = 0;
	int Y_index = 0;
	int r_index = 0;
	int maxValue = 1;
	for (int i = 0; i < m; i++)
	{
	    for (int j = 0; j < n; j++)
	    {
			for (int k = 0; k < size_r100; k++)
			{
				if (maxValue < Hough_space[i][j][k])
				{
					maxValue = Hough_space[i][j][k];
					X_index = i;
					Y_index = j;
					r_index = k;
				}
			}
	    }
	}

	//释放动态生成的内存
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			delete[]Hough_space[i][j];
		}
		delete[]Hough_space[i];
	}
	delete[]Hough_space;

	double x = double(X_index) / 10 + min_X - max_r;
	double y = double(Y_index) / 10 + min_Y - max_r;
	double r = double(r_index) / 100;

	CircleFit3DInfo studInfo;

	double radius_error=max_r;

	//根据Hough圆变换求出的初始中心，筛选出需要的点
	Mat finalPickedData;
	int max = 0;
	for (int i = 0; i != vecXYRoted.size(); i++)
	{
		if (vecXYRoted[i].empty())
			continue;

		int num = 0;
		Mat error;
		Mat XDistance = (vecXYRoted[i].col(0) - x).mul(vecXYRoted[i].col(0) - x);
		Mat YDistance = (vecXYRoted[i].col(1) - y).mul(vecXYRoted[i].col(1) - y);
		cv::sqrt(XDistance + YDistance, error);
		error = error - r;
		//去除异常点
		Mat pickedData;
		for (int cnt = 0; cnt < error.rows; cnt++)
		{
			if (std::abs(error.at<double>(cnt, 0)) < step_r)
			{
				pickedData.push_back(Camera3DData[i].row(cnt));
				num++;
			}
		}
		//if (num > max)
		//{
			//finalPickedData = Camera3DData[i];
			finalPickedData = pickedData;
			if (finalPickedData.rows > 10)
			{
				CircleFit3DInfo studInfoTemp = CircleFit3D(finalPickedData);;
				if ((abs(studInfoTemp.Radius - r) < (max_r - min_r)) && (abs(studInfoTemp.Radius - r) < radius_error))
				{
					max = num;
					radius_error = abs(studInfoTemp.Radius - r);
					studInfo = studInfoTemp;
				}
			}
		//}
	}

	if (max == 0)
		return defaultStudCenter;

	//while (1)
	//{
	//	//根据Hough圆变换求出的初始中心，筛选出需要的点
	//    Mat finalPickedData;
	//    int max = 0;
	//    for (int i = 0; i != vecXYRoted.size(); i++)
	//    {
	//	    if (vecXYRoted[i].empty())
	//		    continue;

	//	    int num = 0;
	//	    Mat error;
	//		Mat XDistance = (vecXYRoted[i].col(0) - x).mul(vecXYRoted[i].col(0) - x);
	//		Mat YDistance = (vecXYRoted[i].col(1) - y).mul(vecXYRoted[i].col(1) - y);
	//		cv::sqrt(XDistance + YDistance, error);
	//		error = error - r;
	//		//去除异常点
	//		Mat pickedData;
	//		for (int cnt = 0; cnt < error.rows; cnt++)
	//		{
	//			if (std::abs(error.at<double>(cnt, 0)) < step_r)
	//			{
	//				pickedData.push_back(Camera3DData[i].row(cnt));
	//				num ++;
	//			}
	//		}
	//		if (num > max)
	//		{
	//			max = num;
	//			finalPickedData = Camera3DData[i];
	//			Mat neg(1, vecXYRoted[i].cols, CV_64FC1, Scalar(0));
	//			vecXYRoted[i] = neg;
	//		}
	//	}

	//	if (finalPickedData.rows < 10)
	//		return defaultStudCenter;

	//	studInfo = CircleFit3D(finalPickedData);

	//	if (abs(studInfo.Radius - r) < (max_r - min_r))
	//		break;

	//}
	return studInfo.Center.t();
}

Mat C3DLaserSensorFeatureExtraction::HoughCircle(Mat axesMat, int num, vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r/* = 0.1 */, double step_theta/* = 0.01*/, double CircleMeanErrorThreshold/* =0.2 */, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));

	Mat MinMaxData(XYRoted.rows, XYRoted.cols, CV_64FC1, Scalar(1));
	cv::sort(XYRoted, MinMaxData, CV_SORT_EVERY_COLUMN | CV_SORT_ASCENDING);  //各列升序独立排列
	double max_X = MinMaxData.at<double>(MinMaxData.rows - 1, 0);
	double min_X = MinMaxData.at<double>(0, 0);
	double max_Y = MinMaxData.at<double>(MinMaxData.rows - 1, 1);
	double min_Y = MinMaxData.at<double>(0, 1);

	double length_X = max_X - min_X;
	double length_Y = max_Y - min_Y;
	Mat XYRoted2;
	for (int i = 0; i != XYRoted.rows; i++)
	{
		if ((XYRoted.at<double>(i, 0) < (min_X + length_X / 4)) || (XYRoted.at<double>(i, 0) > (max_X - length_X / 4)) ||
			(XYRoted.at<double>(i, 1) < (min_Y + length_Y / 4)) || (XYRoted.at<double>(i, 1) > (max_Y - length_Y / 4)))
		{
			XYRoted2.push_back(XYRoted.row(i));
		}
	}

	int m = (cvCeil(max_X) - cvFloor(min_X) + 2 * cvCeil(max_r)) * 10;  //给hough圆检测最大半径预留空间
	int n = (cvCeil(max_Y) - cvFloor(min_Y) + 2 * cvCeil(max_r)) * 10;
	//平移至原点，并且增加一个最大检测半径
	XYRoted2.col(0) = XYRoted2.col(0) - min_X + max_r;
	XYRoted2.col(1) = XYRoted2.col(1) - min_Y + max_r;

	int size_theta = cvRound(2 * CV_PI / step_theta);
	int size_r = cvRound((max_r - min_r) / step_r);
	int size_r100 = cvRound(max_r * 100) + 1;
	//申请一个3维数组m*n*size_r10,并进行初始化操作
	int ***Hough_space;
	Hough_space = new int **[m];
	for (int i = 0; i < m; i++)
	{
		Hough_space[i] = new int *[n];
		for (int j = 0; j < n; j++)
		{
			Hough_space[i][j] = new int[size_r100];
			memset(Hough_space[i][j], 0, sizeof(int)*size_r100);
		}
	}

	for (int i = 0; i != XYRoted2.rows; i++)
	{
		for (int j = 0; j != size_r + 1; j++)
		{
			double r1 = min_r + j * step_r;
			for (int k = 0; k != size_theta; k++)
			{
				int a = cvRound((XYRoted2.at<double>(i, 0) + r1 * cos(k * step_theta)) * 10);
				int b = cvRound((XYRoted2.at<double>(i, 1) + r1 * sin(k * step_theta)) * 10);
				int r100 = cvRound(r1 * 100);
				if (a >= 0 && a <= m - 1 && b >= 0 && b <= n - 1 && r100 >= 0 && r100 <= size_r100 - 1)
					Hough_space[a][b][r100] = Hough_space[a][b][r100] + 1;
			}
		}
	}

	//遍历Hough_space寻找最大值及其索引
	int X_index = 0;
	int Y_index = 0;
	int r_index = 0;
	int maxValue = 1;
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			for (int k = 0; k < size_r100; k++)
			{
				if (maxValue < Hough_space[i][j][k])
				{
					maxValue = Hough_space[i][j][k];
					X_index = i;
					Y_index = j;
					r_index = k;
				}
			}
		}
	}

	//释放动态生成的内存
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			delete[]Hough_space[i][j];
		}
		delete[]Hough_space[i];
	}
	delete[]Hough_space;

	double x = double(X_index) / 10 + min_X - max_r;
	double y = double(Y_index) / 10 + min_Y - max_r;
	double r = double(r_index) / 100;

	CircleFit3DInfo studInfo;
	while (1)
	{
		//根据Hough圆变换求出的初始中心，筛选出需要的点
		Mat finalPickedData;
		int max = 0;
		for (int i = 0; i != vecXYRoted.size(); i++)
		{
			if (vecXYRoted[i].empty())
				continue;

			int num = 0;
			Mat error;
			Mat XDistance = (vecXYRoted[i].col(0) - x).mul(vecXYRoted[i].col(0) - x);
			Mat YDistance = (vecXYRoted[i].col(1) - y).mul(vecXYRoted[i].col(1) - y);
			cv::sqrt(XDistance + YDistance, error);
			error = error - r;
			//去除异常点
			Mat pickedData;
			for (int cnt = 0; cnt < error.rows; cnt++)
			{
				if (std::abs(error.at<double>(cnt, 0)) < step_r)
				{
					pickedData.push_back(Camera3DData[i].row(cnt));
					num++;
				}
			}
			if (num > max)
			{
				max = num;
				finalPickedData = Camera3DData[i];
				Mat neg(1, vecXYRoted[i].cols, CV_64FC1, Scalar(0));
				vecXYRoted[i] = neg;
			}
		}

		if (finalPickedData.rows < 10)
			return defaultStudCenter;

		//180525
		RotToZInfo rotInfo = RotToZ(finalPickedData, axesMat);
		string strResultPath = "circlePointsFinal.txt";
		LaserSensorIO.WriteRowFirstAPP(strResultPath, rotInfo.XYRoted);
		Mat picture = Mat::zeros(1000, 1000, CV_8UC3);
		Mat picture2 = Mat::zeros(1000, 1000, CV_8UC3);
		for (int i = 0; i < rotInfo.XYRoted.rows; ++i)
		{
			Point2d Pt;
			Pt.x = rotInfo.XYRoted.at<double>(i, 0) * 100;
			Pt.y = (105 + rotInfo.XYRoted.at<double>(i, 1)) * 100;
			circle(picture, Pt, 1, Scalar(0, 0, 255), -1);
		}
		stringstream a;
		string strNum;
		a << num;
		a >> strNum;
		string strImageSavePath = "CircleFinal\\"+strNum+".tiff";
		imwrite(strImageSavePath, picture);

		studInfo = CircleFit3D(finalPickedData);

		if (abs(studInfo.Radius - r) < (max_r - min_r))
			break;

	}
	return studInfo.Center.t();
}

//180525 取票数排在前N名的点，而非取最高票数
Mat C3DLaserSensorFeatureExtraction::HoughCircle2(Mat &XYRoted, double min_r, double max_r, double step_r/* = 0.1 */, double step_theta/* = 0.01*/, double CircleMeanErrorThreshold/* =0.2 */, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)
{
	#define NOMINATE 3
	
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));

	Mat MinMaxData(XYRoted.rows, XYRoted.cols, CV_64FC1, Scalar(1));
	cv::sort(XYRoted, MinMaxData, CV_SORT_EVERY_COLUMN | CV_SORT_ASCENDING);  //各列升序独立排列
	double max_X = MinMaxData.at<double>(MinMaxData.rows - 1, 0);
	double min_X = MinMaxData.at<double>(0, 0);
	double max_Y = MinMaxData.at<double>(MinMaxData.rows - 1, 1);
	double min_Y = MinMaxData.at<double>(0, 1);

	double length_X = max_X - min_X;
	double length_Y = max_Y - min_Y;
	Mat XYRoted2;
	for (int i = 0; i != XYRoted.rows; i++)
	{
		if ((XYRoted.at<double>(i, 0) < (min_X + length_X / 4)) || (XYRoted.at<double>(i, 0) > (max_X - length_X / 4)) ||
			(XYRoted.at<double>(i, 1) < (min_Y + length_Y / 4)) || (XYRoted.at<double>(i, 1) > (max_Y - length_Y / 4)))
		{
			XYRoted2.push_back(XYRoted.row(i));
		}
	}

	int m = (cvCeil(max_X) - cvFloor(min_X) + 2 * cvCeil(max_r)) * 10;  //给hough圆检测最大半径预留空间
	int n = (cvCeil(max_Y) - cvFloor(min_Y) + 2 * cvCeil(max_r)) * 10;
	//平移至原点，并且增加一个最大检测半径
	XYRoted2.col(0) = XYRoted2.col(0) - min_X + max_r;
	XYRoted2.col(1) = XYRoted2.col(1) - min_Y + max_r;

	int size_theta = cvRound(2 * CV_PI / step_theta);
	int size_r = cvRound((max_r - min_r) / step_r);
	int size_r100 = cvRound(max_r * 100) + 1;
	//申请一个3维数组m*n*size_r10,并进行初始化操作
	int ***Hough_space;
	Hough_space = new int **[m];
	for (int i = 0; i < m; i++)
	{
		Hough_space[i] = new int *[n];
		for (int j = 0; j < n; j++)
		{
			Hough_space[i][j] = new int[size_r100];
			memset(Hough_space[i][j], 0, sizeof(int)*size_r100);
		}
	}

	for (int i = 0; i != XYRoted2.rows; i++)
	{
		for (int j = 0; j != size_r + 1; j++)
		{
			double r1 = min_r + j * step_r;
			for (int k = 0; k != size_theta; k++)
			{
				int a = cvRound((XYRoted2.at<double>(i, 0) + r1 * cos(k * step_theta)) * 10);
				int b = cvRound((XYRoted2.at<double>(i, 1) + r1 * sin(k * step_theta)) * 10);
				int r100 = cvRound(r1 * 100);
				if (a >= 0 && a <= m - 1 && b >= 0 && b <= n - 1 && r100 >= 0 && r100 <= size_r100 - 1)
					Hough_space[a][b][r100] = Hough_space[a][b][r100] + 1;
			}
		}
	}

	//遍历Hough_space寻找最大值及其索引
	int X_index[NOMINATE];
	int Y_index[NOMINATE];
	int r_index[NOMINATE];
	int maxValue[NOMINATE];
	for (int i = 0; i != NOMINATE; ++i)
	{
		maxValue[i] = -1;
	}

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			for (int k = 0; k < size_r100; k++)
			{
				/*if (maxValue < Hough_space[i][j][k])
				{
					maxValue = Hough_space[i][j][k];
					X_index = i;
					Y_index = j;
					r_index = k;
				}*/
				for (int nomt = 0; nomt != NOMINATE; ++nomt)
				{
					if (maxValue[nomt] < Hough_space[i][j][k])
					{
						for (int changenomt = NOMINATE - 1; changenomt > nomt; --changenomt)
						{
							maxValue[changenomt] = maxValue[changenomt - 1];
							X_index[changenomt] = X_index[changenomt - 1];
							Y_index[changenomt] = Y_index[changenomt - 1];
							r_index[changenomt] = r_index[changenomt - 1];
						}
						maxValue[nomt] = Hough_space[i][j][k];
						X_index[nomt] = i;
						Y_index[nomt] = j;
						r_index[nomt] = k;
						break;
					}
				}
			}
		}
	}

	//释放动态生成的内存
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			delete[]Hough_space[i][j];
		}
		delete[]Hough_space[i];
	}
	delete[]Hough_space;

	double x = 0;
	double y = 0;
	double r = 0;
	for (int i = 0; i != NOMINATE; ++i)
	{
		x += double(X_index[i]) / 10 + min_X - max_r;
		y += double(Y_index[i]) / 10 + min_Y - max_r;
		r += double(r_index[i]) / 100;
	}
	x /= NOMINATE;
	y /= NOMINATE;
	r /= NOMINATE;

	Mat studCenter(1, 3, CV_64FC1, Scalar(-1));
	studCenter.at<double>(0, 0) = x;
	studCenter.at<double>(0, 1) = y;
	studCenter.at<double>(0, 2) = XYRoted.at<double>(0, 2);
	return studCenter;

	//double x = double(X_index) / 10 + min_X - max_r;
	//double y = double(Y_index) / 10 + min_Y - max_r;
	//double r = double(r_index) / 100;

	//CircleFit3DInfo studInfo;

	//double radius_error = max_r;

	////根据Hough圆变换求出的初始中心，筛选出需要的点
	//Mat finalPickedData;
	//int max = 0;
	//for (int i = 0; i != vecXYRoted.size(); i++)
	//{
	//	if (vecXYRoted[i].empty())
	//		continue;

	//	int num = 0;
	//	Mat error;
	//	Mat XDistance = (vecXYRoted[i].col(0) - x).mul(vecXYRoted[i].col(0) - x);
	//	Mat YDistance = (vecXYRoted[i].col(1) - y).mul(vecXYRoted[i].col(1) - y);
	//	cv::sqrt(XDistance + YDistance, error);
	//	error = error - r;
	//	//去除异常点
	//	Mat pickedData;
	//	for (int cnt = 0; cnt < error.rows; cnt++)
	//	{
	//		if (std::abs(error.at<double>(cnt, 0)) < step_r)
	//		{
	//			pickedData.push_back(Camera3DData[i].row(cnt));
	//			num++;
	//		}
	//	}
	//	//if (num > max)
	//	//{
	//	//finalPickedData = Camera3DData[i];
	//	finalPickedData = pickedData;
	//	if (finalPickedData.rows > 10)
	//	{
	//		CircleFit3DInfo studInfoTemp = CircleFit3D(finalPickedData);;
	//		if ((abs(studInfoTemp.Radius - r) < (max_r - min_r)) && (abs(studInfoTemp.Radius - r) < radius_error))
	//		{
	//			max = num;
	//			radius_error = abs(studInfoTemp.Radius - r);
	//			studInfo = studInfoTemp;
	//		}
	//	}
	//	//}
	//}

	//if (max == 0)
	//	return defaultStudCenter;

	//while (1)
	//{
	//	//根据Hough圆变换求出的初始中心，筛选出需要的点
	//	Mat finalPickedData;
	//	int max = 0;
	//	for (int i = 0; i != vecXYRoted.size(); i++)
	//	{
	//		if (vecXYRoted[i].empty())
	//			continue;

	//		int num = 0;
	//		Mat error;
	//		Mat XDistance = (vecXYRoted[i].col(0) - x).mul(vecXYRoted[i].col(0) - x);
	//		Mat YDistance = (vecXYRoted[i].col(1) - y).mul(vecXYRoted[i].col(1) - y);
	//		cv::sqrt(XDistance + YDistance, error);
	//		error = error - r;
	//		//去除异常点
	//		Mat pickedData;
	//		for (int cnt = 0; cnt < error.rows; cnt++)
	//		{
	//			if (std::abs(error.at<double>(cnt, 0)) < step_r)
	//			{
	//				pickedData.push_back(Camera3DData[i].row(cnt));
	//				num++;
	//			}
	//		}
	//		if (num > max)
	//		{
	//			max = num;
	//			finalPickedData = Camera3DData[i];
	//			Mat neg(1, vecXYRoted[i].cols, CV_64FC1, Scalar(0));
	//			vecXYRoted[i] = neg;
	//		}
	//	}

	//	if (finalPickedData.rows < 10)
	//		return defaultStudCenter;

	//	studInfo = CircleFit3D(finalPickedData);

	//	if (abs(studInfo.Radius - r) < (max_r - min_r))
	//		break;
	//}
	//return studInfo.Center.t();
}

//180525 取票数排在前N名的点，而非取最高票数
Mat C3DLaserSensorFeatureExtraction::HoughCircle2(Mat axesMat, int num, vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r/* = 0.1 */, double step_theta/* = 0.01*/, double CircleMeanErrorThreshold/* =0.2 */, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)
{
#define NOMINATE 3

	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));

	Mat MinMaxData(XYRoted.rows, XYRoted.cols, CV_64FC1, Scalar(1));
	cv::sort(XYRoted, MinMaxData, CV_SORT_EVERY_COLUMN | CV_SORT_ASCENDING);  //各列升序独立排列
	double max_X = MinMaxData.at<double>(MinMaxData.rows - 1, 0);
	double min_X = MinMaxData.at<double>(0, 0);
	double max_Y = MinMaxData.at<double>(MinMaxData.rows - 1, 1);
	double min_Y = MinMaxData.at<double>(0, 1);

	double length_X = max_X - min_X;
	double length_Y = max_Y - min_Y;
	Mat XYRoted2;
	for (int i = 0; i != XYRoted.rows; i++)
	{
		if ((XYRoted.at<double>(i, 0) < (min_X + length_X / 4)) || (XYRoted.at<double>(i, 0) > (max_X - length_X / 4)) ||
			(XYRoted.at<double>(i, 1) < (min_Y + length_Y / 4)) || (XYRoted.at<double>(i, 1) > (max_Y - length_Y / 4)))
		{
			XYRoted2.push_back(XYRoted.row(i));
		}
	}

	int m = (cvCeil(max_X) - cvFloor(min_X) + 2 * cvCeil(max_r)) * 10;  //给hough圆检测最大半径预留空间
	int n = (cvCeil(max_Y) - cvFloor(min_Y) + 2 * cvCeil(max_r)) * 10;
	//平移至原点，并且增加一个最大检测半径
	XYRoted2.col(0) = XYRoted2.col(0) - min_X + max_r;
	XYRoted2.col(1) = XYRoted2.col(1) - min_Y + max_r;

	int size_theta = cvRound(2 * CV_PI / step_theta);
	int size_r = cvRound((max_r - min_r) / step_r);
	int size_r100 = cvRound(max_r * 100) + 1;
	//申请一个3维数组m*n*size_r10,并进行初始化操作
	int ***Hough_space;
	Hough_space = new int **[m];
	for (int i = 0; i < m; i++)
	{
		Hough_space[i] = new int *[n];
		for (int j = 0; j < n; j++)
		{
			Hough_space[i][j] = new int[size_r100];
			memset(Hough_space[i][j], 0, sizeof(int)*size_r100);
		}
	}

	for (int i = 0; i != XYRoted2.rows; i++)
	{
		for (int j = 0; j != size_r + 1; j++)
		{
			double r1 = min_r + j * step_r;
			for (int k = 0; k != size_theta; k++)
			{
				int a = cvRound((XYRoted2.at<double>(i, 0) + r1 * cos(k * step_theta)) * 10);
				int b = cvRound((XYRoted2.at<double>(i, 1) + r1 * sin(k * step_theta)) * 10);
				int r100 = cvRound(r1 * 100);
				if (a >= 0 && a <= m - 1 && b >= 0 && b <= n - 1 && r100 >= 0 && r100 <= size_r100 - 1)
					Hough_space[a][b][r100] = Hough_space[a][b][r100] + 1;
			}
		}
	}

	//遍历Hough_space寻找最大值及其索引
	int X_index[NOMINATE];
	int Y_index[NOMINATE];
	int r_index[NOMINATE];
	int maxValue[NOMINATE];
	for (int i = 0; i != NOMINATE; ++i)
	{
		maxValue[i] = -1;
	}

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			for (int k = 0; k < size_r100; k++)
			{
				/*if (maxValue < Hough_space[i][j][k])
				{
				maxValue = Hough_space[i][j][k];
				X_index = i;
				Y_index = j;
				r_index = k;
				}*/
				for (int nomt = 0; nomt != NOMINATE; ++nomt)
				{
					if (maxValue[nomt] < Hough_space[i][j][k])
					{
						for (int changenomt = NOMINATE - 1; changenomt > nomt; --changenomt)
						{
							maxValue[changenomt] = maxValue[changenomt - 1];
							X_index[changenomt] = X_index[changenomt - 1];
							Y_index[changenomt] = Y_index[changenomt - 1];
							r_index[changenomt] = r_index[changenomt - 1];
						}
						maxValue[nomt] = Hough_space[i][j][k];
						X_index[nomt] = i;
						Y_index[nomt] = j;
						r_index[nomt] = k;
						break;
					}
				}
			}
		}
	}

	//释放动态生成的内存
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			delete[]Hough_space[i][j];
		}
		delete[]Hough_space[i];
	}
	delete[]Hough_space;

	//180602
	/*vector<Mat> tickets;
	string strResultPath1 = "Houghtickets1.txt";
	string strResultPath2 = "Houghtickets2.txt";
	string strResultPath3 = "Houghtickets3.txt";
	for (int i = 0; i != NOMINATE; ++i)
	{
		Mat temp(1, 4, CV_64FC1);
		temp.at<double>(0, 0) = X_index[i];
		temp.at<double>(0, 1) = Y_index[i];
		temp.at<double>(0, 2) = r_index[i];
		temp.at<double>(0, 3) = maxValue[i];
		tickets.push_back(temp);
	}
	LaserSensorIO.WriteRowFirstAPP(strResultPath1, tickets[0]);
	LaserSensorIO.WriteRowFirstAPP(strResultPath2, tickets[1]);
	LaserSensorIO.WriteRowFirstAPP(strResultPath3, tickets[2]);*/

	double x = 0;
	double y = 0;
	double r = 0;
	for (int i = 0; i != NOMINATE; ++i)
	{
		x += double(X_index[i]) / 10 + min_X - max_r;
		y += double(Y_index[i]) / 10 + min_Y - max_r;
		r += double(r_index[i]) / 100;
	}
	x /= NOMINATE;
	y /= NOMINATE;
	r /= NOMINATE;

	//double x = double(X_index) / 10 + min_X - max_r;
	//double y = double(Y_index) / 10 + min_Y - max_r;
	//double r = double(r_index) / 100;

	CircleFit3DInfo studInfo;
	while (1)
	{
		//根据Hough圆变换求出的初始中心，筛选出需要的点
		Mat finalPickedData;
		int max = 0;
		for (int i = 0; i != vecXYRoted.size(); i++)
		{
			if (vecXYRoted[i].empty())
				continue;

			int num = 0;
			Mat error;
			Mat XDistance = (vecXYRoted[i].col(0) - x).mul(vecXYRoted[i].col(0) - x);
			Mat YDistance = (vecXYRoted[i].col(1) - y).mul(vecXYRoted[i].col(1) - y);
			cv::sqrt(XDistance + YDistance, error);
			error = error - r;
			//去除异常点
			Mat pickedData;
			for (int cnt = 0; cnt < error.rows; cnt++)
			{
				if (std::abs(error.at<double>(cnt, 0)) < step_r)
				{
					pickedData.push_back(Camera3DData[i].row(cnt));
					num++;
				}
			}
			if (num > max)
			{
				max = num;
				finalPickedData = Camera3DData[i];
				Mat neg(1, vecXYRoted[i].cols, CV_64FC1, Scalar(0));
				vecXYRoted[i] = neg;
			}
		}

		if (finalPickedData.rows < 10)
			return defaultStudCenter;

		//180525
		//RotToZInfo rotInfo = RotToZ(finalPickedData, axesMat);
		//string strResultPath = "circlePointsFinal.txt";
		//LaserSensorIO.WriteRowFirstAPP(strResultPath, rotInfo.XYRoted);
		//Mat picture = Mat::zeros(1000, 1000, CV_8UC3);
		//Mat picture2 = Mat::zeros(1000, 1000, CV_8UC3);
		//for (int i = 0; i < rotInfo.XYRoted.rows; ++i)
		//{
		//	Point2d Pt;
		//	Pt.x = rotInfo.XYRoted.at<double>(i, 0) * 100;
		//	Pt.y = (105 + rotInfo.XYRoted.at<double>(i, 1)) * 100;
		//	circle(picture, Pt, 1, Scalar(0, 0, 255), -1);
		//}
		//stringstream a;
		//string strNum;
		//a << num;
		//a >> strNum;
		//string strImageSavePath = "CircleFinal\\"+strNum+".tiff";
		//imwrite(strImageSavePath, picture);

		studInfo = CircleFit3D(finalPickedData);

		if (abs(studInfo.Radius - r) < (max_r - min_r))
			break;
	}
	return studInfo.Center.t();
}

Point3d C3DLaserSensorFeatureExtraction::calcPCA(const Mat &Camera3DData)
{
	Mat meanPoint(1, Camera3DData.cols, CV_64FC1, cvScalar(0));
	//求取平均值
	for (int i = 0; i != Camera3DData.cols; i++)
	{
		meanPoint.at<double>(0, i) = cv::mean(Camera3DData.col(i)).val[0];
	}

    Mat dataAdjust(Camera3DData.rows, Camera3DData.cols, CV_64FC1);
	for (int cnt = 0; cnt != Camera3DData.rows; cnt++)
		dataAdjust.row(cnt) = Camera3DData.row(cnt) - meanPoint;

    //求协方差矩阵
	Mat covMat(dataAdjust.cols, dataAdjust.cols, CV_64FC1);
    for (int cnt = 0; cnt != dataAdjust.cols; cnt++)
	{
		covMat.at<double>(cnt, cnt) = dataAdjust.col(cnt).dot(dataAdjust.col(cnt)) / (dataAdjust.rows - 1);
	}
	covMat.at<double>(0, 1) = covMat.at<double>(1, 0) = dataAdjust.col(0).dot(dataAdjust.col(1)) / (dataAdjust.rows - 1);
	covMat.at<double>(0, 2) = covMat.at<double>(2, 0) = dataAdjust.col(0).dot(dataAdjust.col(2)) / (dataAdjust.rows - 1);
	covMat.at<double>(1, 2) = covMat.at<double>(2, 1) = dataAdjust.col(1).dot(dataAdjust.col(2)) / (dataAdjust.rows - 1);

	//求协方差矩阵的特征值和特征向量，eigen函数已经排序
	Mat eigVals(3, 1, CV_64FC1, cvScalar(0));
	Mat eigVecs(3, 3, CV_64FC1, cvScalar(0));
	cv::eigen(covMat, eigVals, eigVecs);

	//轴线方向即为协方差矩阵的最大特征值对应的特征向量
	Point3d axes(eigVecs.row(0));

	return axes;
}

double C3DLaserSensorFeatureExtraction::EliminateGrossError(Mat Error,double MeanErrorThreshold/* =0.2 */,double GrossErrorCoe/* =0.5 */,double NormalErrorCoe/* =3 */)
{
	double MeanError=0;
	double StdThreshold=0;
	vector<int> Index;
	MeanError=cv::mean(abs(Error)).val[0];
	if (MeanError>MeanErrorThreshold)
	{
		StdThreshold = GrossErrorCoe*std::sqrt(Error.dot(Error)/(Error.rows-1));
		if(StdThreshold<0.1)
		{
			StdThreshold = 0.1;
		}
	} 
	else
	{
		if (NormalErrorCoe>3)/////// 图像坐标系下直线避免去除异常点造成 modified in 20150623
		{
			double A=NormalErrorCoe*std::sqrt(Error.dot(Error)/(Error.rows-1));
			double B=3*std::sqrt(Error.dot(Error)/(Error.rows-1))+MeanError;
			StdThreshold=(A>B)?A:B;
		} 
		else
		{
			StdThreshold=NormalErrorCoe*std::sqrt(Error.dot(Error)/(Error.rows-1));
		}
	}
	return StdThreshold;  
}

PlaneFit3DInfo C3DLaserSensorFeatureExtraction::PlaneFit3D(const Mat &X,int flag/* =0 */)
{
	//// X(:,1)=x X(:,2)=y X(:,3)=z
	PlaneFit3DInfo PlaneFitInfo;
	Mat U(3,3,CV_64FC1,Scalar(1));
	Mat S;
	Mat VT;
	Mat Points(3,1,CV_64FC1,Scalar(0));
	Mat Residual(X.rows,1,CV_64FC1,Scalar(1));
	Mat XMeaned(X.rows,X.cols,CV_64FC1,Scalar(0));
	double MeanX=0;
	double MeanY=0;
	double MeanZ=0;
	for (int i=0;i<X.rows;i++)
	{
		MeanX+=X.at<double>(i,0);
		MeanY+=X.at<double>(i,1);
		MeanZ+=X.at<double>(i,2);
	}
	//// 计算平面所过的点
	Points.at<double>(0,0)=MeanX/X.rows;
	Points.at<double>(1,0)=MeanY/X.rows;
	Points.at<double>(2,0)=MeanZ/X.rows;

	Points.copyTo(PlaneFitInfo.Points);
	XMeaned.col(0)=X.col(0)-PlaneFitInfo.Points.at<double>(0,0);
	XMeaned.col(1)=X.col(1)-PlaneFitInfo.Points.at<double>(1,0);
	XMeaned.col(2)=X.col(2)-PlaneFitInfo.Points.at<double>(2,0);
	SVD::compute(XMeaned.t(),S,U,VT);
	/////// 平面的法向量
	PlaneFitInfo.NormVector=U.col(2);
	if (1==flag)
	{ 
		/// 计算拟合误差
		/////点到平面的距离误差
		for(int j=0;j<XMeaned.rows;j++)
		{  
			Residual.at<double>(j,0)=XMeaned.row(j).dot(PlaneFitInfo.NormVector.t());
		}
		Residual.copyTo(PlaneFitInfo.PlaneFitError);
	} 
	else if(0==flag)
	{
		Mat ErrorInfo(1,1,CV_64FC1,Scalar(0));
		PlaneFitInfo.PlaneFitError=ErrorInfo;
	}
	return PlaneFitInfo;
}

LSCircleFitInfo C3DLaserSensorFeatureExtraction::LSCircleFit2D(Mat X)
{
	LSCircleFitInfo LSCircle;
	Mat b(X.rows, 1, CV_64FC1, cv::Scalar(0));
	Mat A(X.rows, 3, CV_64FC1, cv::Scalar(0));
	Mat AA(3, 3, CV_64FC1, cv::Scalar(0));
	Mat Center1(3, 1, CV_64FC1, cv::Scalar(0));///// 包含圆心与半径信息
	double Radius;
	A.col(0) = X.col(0) * 1;
	A.col(1) = X.col(1) * 1;
	A.col(2).setTo(cv::Scalar(1));
	AA = A.t()*A;
	b = X.col(0).mul(X.col(0)) + X.col(1).mul(X.col(1));

	Center1 = AA.inv(DECOMP_SVD)*(A.t()*b);////// modified in 20150621
	LSCircle.Par = Center1(Range(0, 2), Range().all()) / 2;
	Radius = std::sqrt(Center1.at<double>(2, 0) + LSCircle.Par.dot(LSCircle.Par));
	LSCircle.Par.push_back(Radius);
	return LSCircle;
}

CircleFitGradientInfo C3DLaserSensorFeatureExtraction::CircleCurrentIteration(Mat Par, Mat X)
{
	CircleFitGradientInfo Info;
	Mat DX(X.rows, 1, CV_64FC1);
	Mat DY(X.rows, 1, CV_64FC1);
	Mat D(X.rows, 1, CV_64FC1);
	Mat J(X.rows, 3, CV_64FC1);
	DX = X.col(0) - Par.at<double>(0, 0);
	DY = X.col(1) - Par.at<double>(1, 0);
	cv::sqrt((DX.col(0).mul(DX.col(0)) + DY.col(0).mul(DY.col(0))), D);
	J.col(0) = -DX.col(0).mul(1 / D);
	J.col(1) = -DY.col(0).mul(1 / D);
	J.col(2) = -1;
	Info.J = J;
	Info.g = D - Par.at<double>(2, 0);
	Info.F = cv::norm(Info.g, NORM_L2);
	return Info;
}

LMCircleFitInfo C3DLaserSensorFeatureExtraction::LMCircleFit2D(Mat XY,int MaxIter/* =100 */,double LambdaIni/* =1 */)
{

	Mat X(XY.rows,2,CV_64FC1,Scalar(0));
	XY(Range().all(),Range(0,2)).copyTo(X);	 
	const double eps=0.000001;
	double lambda_sqrt=std::sqrt(LambdaIni);
	Mat DelPar(3,1,CV_64FC1,cv::Scalar(1));
	double Progress;
	Mat ParTemp(3,1,CV_64FC1);
	Mat GJ(X.rows+3,3,CV_64FC1);
	Mat Gg(X.rows+3,1,CV_64FC1);
	Mat I=Mat::eye(3,3,CV_64FC1);
	Mat B(3,1,CV_64FC1,cv::Scalar(0));
	LMCircleFitInfo LMInfo;
	CircleFitGradientInfo GInfo;
	CircleFitGradientInfo GInfoTemp;
	LSCircleFitInfo LSInfo;
	LSInfo=LSCircleFit2D(X);
	GInfo=CircleCurrentIteration(LSInfo.Par,X);
	int Iter=0;
	for (int i=1;i<MaxIter+1;i++)
	{
		Iter++;
		GJ(Range(0,X.rows),Range().all())=GInfo.J*1;
		Gg(Range(0,X.rows),Range().all())=GInfo.g*1;
		Gg(Range(X.rows,Gg.rows),Range().all())=B*1;
#pragma  region ///adjusting Lamda 
		while (1)
		{
			GJ(Range(X.rows,GJ.rows),Range().all())=lambda_sqrt*I;
			DelPar=GJ.inv(DECOMP_SVD)*Gg;
			Progress = cv::norm(DelPar, NORM_L2) / (cv::norm(LSInfo.Par) + eps);
			if (Progress<eps)
			{
				break;
			}
			ParTemp=LSInfo.Par-DelPar;
			GInfoTemp=CircleCurrentIteration(ParTemp,X);
			if ((GInfoTemp.F<GInfo.F))
			{
				lambda_sqrt=lambda_sqrt/2;
				break;
			} 
			else
			{
				lambda_sqrt=lambda_sqrt*2;
				continue;
			}
		}
#pragma  endregion
		if (Progress<eps)
		{
			break;
		}   
		LSInfo.Par=ParTemp;
		GInfo.J=GInfoTemp.J;
		GInfo.g=GInfoTemp.g;
		GInfo.F=GInfoTemp.F;
	}
	LMInfo.Par=LSInfo.Par;
	LMInfo.Iter=Iter;
	return LMInfo;
}

PProjectToPlaneInfo C3DLaserSensorFeatureExtraction::PProjectToPlane(Mat XY)
{
	////先进行平面拟合，再将三维数据点投影到平面上
	PProjectToPlaneInfo PProjectedInfo;
	PlaneFit3DInfo PlaneInfo;
	Mat MeanXY(XY.rows,3,CV_64FC1,Scalar(0));
	Mat Pdistance(XY.rows,1,CV_64FC1,Scalar(0));
	Mat ProjectedXY(XY.rows,3,CV_64FC1,Scalar(0));
	PlaneInfo=PlaneFit3D(XY);
	MeanXY.col(0)=XY.col(0)-PlaneInfo.Points.at<double>(0,0);
	MeanXY.col(1)=XY.col(1)-PlaneInfo.Points.at<double>(1,0);
	MeanXY.col(2)=XY.col(2)-PlaneInfo.Points.at<double>(2,0);
	Pdistance=MeanXY*PlaneInfo.NormVector;
	ProjectedXY=XY-Pdistance*PlaneInfo.NormVector.t();	 
	PProjectedInfo.ProjectedXY=ProjectedXY;
	PProjectedInfo.VT=PlaneInfo.NormVector;
	return PProjectedInfo;
}

Mat C3DLaserSensorFeatureExtraction::Angvec2r(double theta,Mat k)
{
	Mat R(3,3,CV_64FC1,Scalar(0));
	double ux=k.at<double>(0,0);
	double uy=k.at<double>(1,0);
	double uz=k.at<double>(2,0);
	double cth=std::cos(theta);  
	double sth=std::sin(theta);
	double vth=1-cth;
	R.at<double>(0,0)=cth+ux*ux*vth;
	R.at<double>(0,1)=ux*uy*vth-uz*sth;
	R.at<double>(0,2)=ux*uz*vth+uy*sth;
	R.at<double>(1,0)=ux*uy*vth+uz*sth;
	R.at<double>(1,1)=cth+uy*uy*vth;
	R.at<double>(1,2)=uz*uy*vth-ux*sth;
	R.at<double>(2,0)=ux*uz*vth-uy*sth;
	R.at<double>(2,1)=uy*uz*vth+ux*sth;
	R.at<double>(2,2)=uz*uz*vth+cth;
	return R;
}


RotToZInfo C3DLaserSensorFeatureExtraction::RotToZ(const Mat &XY,const Mat &VT)
{
	RotToZInfo RotInfo;
	Mat Z(3,1,CV_64FC1,Scalar(0));
	Z.at<double>(2,0)=1;
	if (VT.at<double>(0,0)==Z.at<double>(0,0)
		&&VT.at<double>(1,0)==Z.at<double>(1,0)
		&&VT.at<double>(2,0)==Z.at<double>(2,0))////// modified in 20150615
	{
		RotInfo.R=Mat::eye(3,3,CV_64FC1);
		RotInfo.XYRoted=XY;
		return RotInfo;
	}
	Mat k(3,1,CV_64FC1,Scalar(1));
	Mat R(3,3,CV_64FC1,Scalar(1));
	Mat XYRoted(XY.cols,XY.rows,CV_64FC1,Scalar(1));
	k=VT.cross(Z);
	k=k/cv::norm(k);
	double theta=std::acos(VT.dot(Z));
	R=Angvec2r(theta,k);
	XYRoted=R*XY.t();
	RotInfo.R=R;
	RotInfo.XYRoted=XYRoted.t();
	return RotInfo;
}

CircleFit3DInfo C3DLaserSensorFeatureExtraction::CircleFit3D(Mat X,double CircleMeanErrorThreshold/* =0.2 */,double CircleGrossErrorCoe/* =0.5 */,double CircleNormalErrorCoe/* =3 */)
{
	PProjectToPlaneInfo ProjectInfo;
	RotToZInfo RotInfo;
	LMCircleFitInfo LMFitInfo;
	CircleFit3DInfo CircleFitInfo;
	double sum(0);
	ProjectInfo=PProjectToPlane(X);
	RotInfo=RotToZ(ProjectInfo.ProjectedXY,ProjectInfo.VT);

#pragma region////////////////////////// 迭代进行空间圆拟合
	vector<int> ErrorIndex;
	double MeanResidal=0;
	do 
	{ ///////考虑设置一个控制变量flag
		LMFitInfo=LMCircleFit2D(RotInfo.XYRoted);
		Mat TraslateData;
		ErrorIndex.clear();
		Mat XDistance(RotInfo.XYRoted.rows,1,CV_64FC1,Scalar(1));
		Mat YDistance(RotInfo.XYRoted.rows,1,CV_64FC1,Scalar(1));
		XDistance=(RotInfo.XYRoted.col(0)-LMFitInfo.Par.at<double>(0,0)).mul(RotInfo.XYRoted.col(0)-LMFitInfo.Par.at<double>(0,0));
		YDistance=(RotInfo.XYRoted.col(1)-LMFitInfo.Par.at<double>(1,0)).mul(RotInfo.XYRoted.col(1)-LMFitInfo.Par.at<double>(1,0));
		cv::sqrt(XDistance+YDistance,CircleFitInfo.CircleFit3DError);
		CircleFitInfo.CircleFit3DError=CircleFitInfo.CircleFit3DError-LMFitInfo.Par.at<double>(2,0);

		////////////////使用新编写的去除粗大误差的方法删除粗大误差点 moidified in 20150601
		MeanResidal=EliminateGrossError(CircleFitInfo.CircleFit3DError,CircleMeanErrorThreshold,CircleGrossErrorCoe,CircleNormalErrorCoe);
		for (int RRNum=0; RRNum<CircleFitInfo.CircleFit3DError.rows; RRNum++)
		{
			if (std::abs(CircleFitInfo.CircleFit3DError.at<double>(RRNum,0))>MeanResidal)
			{
				ErrorIndex.push_back(RRNum);
			}
			else
			{
				TraslateData.push_back(RotInfo.XYRoted.row(RRNum));
			}

		}
		/////////////////
		if (ErrorIndex.size()==RotInfo.XYRoted.rows)
		{
			break;
		}
		RotInfo.XYRoted.pop_back(ErrorIndex.size());
		TraslateData.copyTo(RotInfo.XYRoted);

	} while (!ErrorIndex.empty() && (RotInfo.XYRoted.rows>3));
	/////////////////////////
#pragma endregion

	//string strResultPath = "radius.txt";//180516
	//LaserSensorIO.WriteRowFirstAPP(strResultPath, LMFitInfo.Par.t());//180516

	CircleFitInfo.Center=LMFitInfo.Par(Range(0,2),Range().all());
	///// 计算Z的坐标值
	for(int i=0;i<RotInfo.XYRoted.rows;i++)
	{
		sum+=RotInfo.XYRoted.at<double>(i,2);
	}
	CircleFitInfo.Center.push_back(sum/RotInfo.XYRoted.rows);
	CircleFitInfo.Radius=LMFitInfo.Par.at<double>(2,0);
	CircleFitInfo.Center=RotInfo.R.t()*CircleFitInfo.Center;	//////// 笔误，已修改 20150513
	CircleFitInfo.V=ProjectInfo.VT;	
	return CircleFitInfo;
}
////////////////////////////

/////// 利用平面度对拟合数据进行筛选 modified in 20150603
Mat C3DLaserSensorFeatureExtraction::PlaneThreshod(Mat &PlaneFitData, double PlaneMeanErrorThreshold/* =0.2 */,double PlaneGrossErrorCoe/* =0.5 */,double PlaneNormalErrorCoe/* =3 */)
{
	/////// 先去掉返回默认值的数据
	Mat DefaultValue(1,3,CV_64FC1,Scalar(10000));

#pragma  region //////平面迭代拟合 moidified in 20150519
	vector<int> LaserCalibErrorIndex; 
	double ResidalThresh=0;
	PlaneFit3DInfo mPlaneFit3DInfo;
	do 
	{
		LaserCalibErrorIndex.clear();
		Mat TraslateData;
		mPlaneFit3DInfo=PlaneFit3D(PlaneFitData,1);
		////// 使用新编写的去除粗大误差的方法删除粗大误差点 moidified in 20150601
		ResidalThresh=EliminateGrossError(mPlaneFit3DInfo.PlaneFitError,PlaneMeanErrorThreshold,PlaneGrossErrorCoe,PlaneNormalErrorCoe);
		for (int RRNum=0;RRNum<mPlaneFit3DInfo.PlaneFitError.rows;RRNum++)
		{
			if (std::abs(mPlaneFit3DInfo.PlaneFitError.at<double>(RRNum,0))>ResidalThresh)
			{
				LaserCalibErrorIndex.push_back(RRNum);
			}
			else
			{
				TraslateData.push_back(PlaneFitData.row(RRNum));
			}
		}
		//////////////
		PlaneFitData.pop_back(LaserCalibErrorIndex.size());
		TraslateData.copyTo(PlaneFitData);
	} while (!LaserCalibErrorIndex.empty()&&(PlaneFitData.rows>3));
#pragma endregion

	if(PlaneFitData.rows < 3)
	{
		return DefaultValue;
	}
	else
	{
		return PlaneFitData;
	}

}
#pragma endregion

#pragma  region /////被测特征提取操作

#pragma region/////// 全部点云提取算法(特征代号 0) modified in 20150511
Mat C3DLaserSensorFeatureExtraction::LaserSensorPointCloudsExtraction(Mat Img, int Threshold/* =150 */, int ArcLengthValue/* =50 */, Size2i KernelSize/* =Size */)
{  
	return LaserStripeExtraction(Img,Threshold,ArcLengthValue,KernelSize);
}

Mat C3DLaserSensorFeatureExtraction::LaserStripeExtraction(Mat Img,int Threshold/* =150 */,int ArcLengthValue/* =50 */,Size2i KernelSize/* =Size */)
{

	Mat ImgGray;
	Mat LaserStripeWholeCenters;
	Mat DefaultStripeWholeCenters(1,2,CV_64FC1,Scalar(-1));
	if (Img.rows<1)
	{
		return DefaultStripeWholeCenters;
	}
	if(Img.channels()==3)
	{
		cvtColor(Img,ImgGray,CV_BGR2GRAY);
	}
	else
	{
		ImgGray=Img;
	}
	Mat ImgBinary;
	vector<vector<Point>> Contours;
	Rect ROIOutRect;/////轮廓的外接矩形	
	Rect ExtendRect;///////扩展后的感兴趣区域
	Rect FinalRect;//////// 
	//// 可以将阈值设置的高一些,可以减少初始提取感兴趣区域的面积
	/////设置过大的阈值,导致感兴趣区域不能包含所有光刀区域，
	threshold(ImgGray,ImgBinary,Threshold,255,THRESH_TOZERO);/////主要为了提取光刀线的轮廓
	////// 寻找轮廓findcontours
	findContours(ImgBinary,Contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
	//////自定义函数 按轮廓周长大小排序
	std::sort(Contours.begin(),Contours.end(),LengthDown);///// 按轮廓周长大小排序
	for(vector<vector<Point>>::size_type ContoursNum=0; ContoursNum<Contours.size(); ContoursNum++)/////// 
		///// 最多选择最长的两个轮廓
	{
		if (arcLength(Contours.at(ContoursNum),true)>ArcLengthValue)
		{

			///// mininal up-right bounding rectangle
			ROIOutRect=boundingRect(Contours.at(ContoursNum));
			/////修正感兴趣区域
			Rect ExtendRect=ModifyROI(ImgGray,ROIOutRect,Threshold);
			///////在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
			Rect FinalRect=ExtendedROI(ImgGray,ExtendRect);
			///////复制避免边界效应
			Mat Temp(ImgGray,FinalRect);
			Mat LaserStripeROI;
			Mat LaserStripSmoothedUchar,LaserStripSmoothed;
			Temp.copyTo(LaserStripeROI);
			GaussianBlur(LaserStripeROI,LaserStripSmoothedUchar,KernelSize,0,0);
			LaserStripSmoothedUchar.convertTo(LaserStripSmoothed,CV_64FC1);///// 类型转换
			////确定光刀线的初始中心位置
			Mat Index=OriginalCenter(LaserStripSmoothed);
			if (Index.cols!=LaserStripSmoothed.cols)
			{
				LaserSensorIO.WriteLog("Error(00):Index与LaserStripeSmoothed列数不等!");
			}
			/// 计算光刀线的边界阈值，确定光刀线的宽度
			Mat EdgeThresholdValue=DetermineEdgeValue(LaserStripSmoothed,Index); 
			if (EdgeThresholdValue.cols!=LaserStripSmoothed.cols)
			{
				LaserSensorIO.WriteLog("Error(00):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
			}
			////////// 利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
			Mat LaserROIStripeCenters=DetermineCenters(LaserStripSmoothed,Index,EdgeThresholdValue,FinalRect);
			///////将感兴趣区域的光刀中心存储
			LaserStripeWholeCenters.push_back(LaserROIStripeCenters);        
		} 
		else
		{
			break;  
		}  
	}

	if (LaserStripeWholeCenters.empty())
	{
		return DefaultStripeWholeCenters;
	} 
	else
	{
		return LaserStripeWholeCenters;
	}
}
#pragma endregion

#pragma region  //圆柱特征提取（特征代号11） modified in 20151125
vector<Mat> C3DLaserSensorFeatureExtraction::LaserSensorCylinderExtraction(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, int Threshold/* =150 */, int ArcLengthValue/* =50 */, Size2i KernelSize/* =Size */)  //返回柱面拟合所需的点云数据
{
    Mat DefaultCylinderPoints(1, 2, CV_64FC1, cvScalar(-1));
    Mat DefaultPCAPoints(1, 2, CV_64FC1, cvScalar(-1));
	vector<Mat> points(2);
	points[0] = DefaultCylinderPoints;
	points[1] = DefaultPCAPoints;

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return points;
	}
	if(Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat CylinderPoints;
	vector<Mat> PCAPoints(10);
	int cnt = 0;

	//对其先进行闭运算 modified in 20150515
	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray,FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement=getStructuringElement(MORPH_ELLIPSE,CKernalSize);
	morphologyEx(ImgFirstROIOpen,ImgFirstROI,MORPH_CLOSE,StructurElement); 

	Mat ImgBinary;
	vector<vector<Point>> Contours;
	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;
	Rect ExtendRect;  //扩展后的感兴趣区域
	//可以将阈值设置的高一些,可以减少初始提取感兴趣区域的面积
	//设置过大的阈值,导致感兴趣区域不能包含所有光刀区域
	threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
	//寻找轮廓findContours
	findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	//自定义函数 按轮廓周长大小排序
	std::sort(Contours.begin(), Contours.end(), LengthDown);
	for (vector<vector<Point>>::size_type ContoursNum = 0; ContoursNum < Contours.size(); ContoursNum++)  //遍历找出所有大于弧长阈值的轮廓线
	{
		if (arcLength(Contours.at(ContoursNum), true) > ArcLengthValue)
		{
			//mininal up-right bounding rectangle
			ROIOutRect = boundingRect(Contours.at(ContoursNum));
			//将感兴趣区域的矩形框在整张图片中表达出来
			PicROI.x = FirstRect.x + ROIOutRect.x;
			PicROI.y = FirstRect.y + ROIOutRect.y;
			PicROI.width = ROIOutRect.width;
			PicROI.height = ROIOutRect.height;
			//修正感兴趣区域
			//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
			Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
			//不用进行感兴趣区域的复制
			Mat LaserStripeROI(ImgGray, ExtendRect);
			Mat LaserStripSmoothedUchar, LaserStripSmoothed;
			GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
			LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换
			//确定光刀线的初始中心位置
			Mat Index = OriginalCenter(LaserStripSmoothed);

			if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
			{
				LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
				return points;
			}

			//计算光刀线的边界阈值，确定光刀线的宽度
			Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index); 

			if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
			{
				LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
				return points;
			}

			//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
			Mat LaserROIStripeCenters = DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

			for (int i = 0; i != LaserROIStripeCenters.rows;)
			{
			    int num1 = 0, num2 = 0;
				double avg1 = 0, avg2 = 0;
				//向上最多取5个点比较一次
				for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--)
				{
                    avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
					num1 ++;
				}
				avg1 = avg1 / num1;
				if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10)
				{
					deleteRow(LaserROIStripeCenters, i);
					continue;
				}

				//向下最多取5个点再比较一次
				for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++)
				{
					avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
					num2 ++;
				}
				avg2 = avg2 / num2;
				if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10)
				{
					deleteRow(LaserROIStripeCenters, i);
					continue;
				}

				i++;
			}

			if (LaserROIStripeCenters.rows < 50)
				continue;
			//将感兴趣区域的光刀中心存储
			CylinderPoints.push_back(LaserROIStripeCenters); 
			PCAPoints[cnt].push_back(LaserROIStripeCenters);
			cnt++;
			if (cnt == 10)
				break;
		} 
		else
		{
			break;  
		}  
	}

	if (CylinderPoints.rows > 10)
	{
		points[0] = CylinderPoints;
	}

	if (cnt == 0)
	{
		return points;
	}
	else if (cnt <= 2)
	{
		points[1].pop_back();
		for (int i = 0; i != cnt; i++)
		{
			points[1].push_back(PCAPoints[i]);
		}
		return points;
	}
	else
	{
		if ((PCAPoints[0].at<double>(0,1) - PCAPoints[1].at<double>(0,1)) * (PCAPoints[0].at<double>(0,1) - PCAPoints[2].at<double>(0,1)) < 0)
		{
			points[1] = PCAPoints[0];
			return points;
		}
		else if ((PCAPoints[1].at<double>(0,1) - PCAPoints[0].at<double>(0,1)) * (PCAPoints[1].at<double>(0,1) - PCAPoints[2].at<double>(0,1)) < 0)
		{
			points[1] = PCAPoints[1];
			return points;
		}
		else
		{
			points[1] = PCAPoints[2];
			return points;
		}
	}
}

Mat C3DLaserSensorFeatureExtraction::LaserSensorStudExtraction3(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, int Threshold/* =150 */, int ArcLengthValue/* =50 */, Size2i KernelSize/* =Size2i */)  //返回柱面拟合所需的点云数据，针对螺牙的优化，效果一般
{
	Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultAllPoints;
	}
	if (Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat allPoints;

	//对其先进行闭运算 modified in 20150515
	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray, FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement = getStructuringElement(MORPH_ELLIPSE, CKernalSize);
	morphologyEx(ImgFirstROIOpen, ImgFirstROI, MORPH_CLOSE, StructurElement);

	Mat ImgBinary;
	vector<vector<Point>> Contours;
	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;
	Rect ExtendRect;  //扩展后的感兴趣区域
	//可以将阈值设置的高一些,可以减少初始提取感兴趣区域的面积
	//设置过大的阈值,导致感兴趣区域不能包含所有光刀区域
	threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
	//寻找轮廓findContours
	findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	//findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);//180601似乎无区别
	//自定义函数 按轮廓周长大小排序
	std::sort(Contours.begin(), Contours.end(), LengthDown);
	for (vector<vector<Point>>::size_type ContoursNum = 0; ContoursNum < Contours.size(); ContoursNum++)  //遍历找出所有大于弧长阈值的轮廓线
	{
		if (arcLength(Contours.at(ContoursNum), true) > ArcLengthValue)
		{
			//mininal up-right bounding rectangle
			ROIOutRect = boundingRect(Contours.at(ContoursNum));
			//将感兴趣区域的矩形框在整张图片中表达出来
			PicROI.x = FirstRect.x + ROIOutRect.x;
			PicROI.y = FirstRect.y + ROIOutRect.y;
			PicROI.width = ROIOutRect.width;
			PicROI.height = ROIOutRect.height;
			//修正感兴趣区域
			//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
			Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
			//不用进行感兴趣区域的复制
			Mat LaserStripeROI(ImgGray, ExtendRect);
			Mat LaserStripSmoothedUchar, LaserStripSmoothed;
			GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
			LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换

			/*180517*/
			//针对螺牙的优化，剔除弧线中不连续的线条
			for (int ColNum = 2; ColNum != LaserStripSmoothed.cols; ++ColNum)
			{
				for (int RowsNum = 0; RowsNum != LaserStripSmoothed.rows; ++RowsNum)
					if (LaserStripSmoothed.at<double>(RowsNum, ColNum) > 4)
					{
						bool flag = false;
						for (int i = RowsNum - 1; (i != RowsNum - 3) && (i > -1); --i)
							if (LaserStripSmoothed.at<double>(i, ColNum - 2) > 4)
							{
								flag = true;
								break;
							}
						if (!flag)
						{
							for (int i = RowsNum; (i != RowsNum + 3) && (i < LaserStripSmoothed.rows); ++i)
								if (LaserStripSmoothed.at<double>(i, ColNum - 2) > 4)
								{
									flag = true;
									break;
								}
						}
						if (!flag)
						{
							LaserStripSmoothed.at<double>(RowsNum, ColNum) = 0;
						}
					}
			}
			for (int ColNum = LaserStripSmoothed.cols-3; ColNum != -1; --ColNum)
			{
				for (int RowsNum = 0; RowsNum != LaserStripSmoothed.rows; ++RowsNum)
					if (LaserStripSmoothed.at<double>(RowsNum, ColNum) > 4)
					{
						bool flag = false;
						for (int i = RowsNum - 1; (i != RowsNum - 3) && (i > -1); --i)
							if (LaserStripSmoothed.at<double>(i, ColNum + 2) > 4)
							{
								flag = true;
								break;
							}
						if (!flag)
						{
							for (int i = RowsNum; (i != RowsNum + 3) && (i < LaserStripSmoothed.rows); ++i)
								if (LaserStripSmoothed.at<double>(i, ColNum + 2) > 4)
								{
									flag = true;
									break;
								}
						}
						if (!flag)
						{
							LaserStripSmoothed.at<double>(RowsNum, ColNum) = 0;
						}
					}
			}

			//确定光刀线的初始中心位置
			Mat Index = OriginalCenter(LaserStripSmoothed);

			if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
			{
				LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
				return defaultAllPoints;
			}

			//计算光刀线的边界阈值，确定光刀线的宽度
			Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index);

			if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
			{
				LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
				return defaultAllPoints;
			}

			//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
			Mat LaserROIStripeCenters = DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

			for (int i = 0; i != LaserROIStripeCenters.rows;)
			{
				int num1 = 0, num2 = 0;
				double avg1 = 0, avg2 = 0;
				//向上最多取5个点比较一次
				for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--) //180517 原版
				//for (int rowNum = i - 3; rowNum > i - 7 && rowNum >= 0; rowNum--)
				{
					avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
					num1++;
				}
				avg1 = avg1 / num1;
				//if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10) //180517 原版
				if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10)
				{
					deleteRow(LaserROIStripeCenters, i);
					continue;
				}

				//向下最多取5个点再比较一次
				for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++) //180517 原版
				//for (int rowNum = i + 3; rowNum < i + 7 && rowNum < LaserROIStripeCenters.rows; rowNum++)
				{
					avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
					num2++;
				}
				avg2 = avg2 / num2;
				//if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10) //180517 原版
				if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10)
				{
					deleteRow(LaserROIStripeCenters, i);
					continue;
				}

				i++;
			}

			if (LaserROIStripeCenters.rows < 50)
				continue;
			//将感兴趣区域的光刀中心存储 
			allPoints.push_back(LaserROIStripeCenters);
		}
		else
		{
			break;
		}
	}

	if (allPoints.empty())
		return defaultAllPoints;

	return allPoints;
}

Rect C3DLaserSensorFeatureExtraction::getRect(const Mat &Img, int uplimit, int downlimint, int leftlimit, int rightlimit, bool full)
{
	Rect defaultRect;
	defaultRect.x = 0;
	defaultRect.y = 0;
	defaultRect.width = 0;
	defaultRect.height = 0;

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultRect;
	}
	if (Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray, FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement = getStructuringElement(MORPH_ELLIPSE, CKernalSize);
	morphologyEx(ImgFirstROIOpen, ImgFirstROI, MORPH_CLOSE, StructurElement);

	Mat ImgBinary;
	vector<vector<Point>> Contours;
	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;
	Rect ExtendRect;  //扩展后的感兴趣区域
	int Threshold;

	const int channels[1] = { 0 };
	const int histSize[1] = { 256 };
	float hranges[2] = { 0, 255 };
	const float* ranges[1] = { hranges };
	MatND hist;
	calcHist(&ImgFirstROI, 1, channels, Mat(), hist, 1, histSize, ranges);

	Threshold = GetIterativeBestThreshold(hist);
	//Threshold = Get1DMaxEntropyThreshold(hist);

	if (Threshold <= 1)
	{
		return defaultRect;
	}
	if ((Threshold < 30) && (!full))
	{
		return defaultRect;
	}

	threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);
	findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	std::sort(Contours.begin(), Contours.end(), LengthDown);
	if (!Contours.empty())
	{
		ROIOutRect = boundingRect(Contours.at(0));
		//将感兴趣区域的矩形框在整张图片中表达出来
	}
	else
	{
		return defaultRect;
	}
	//Rect ROIOutRect2 = boundingRect(Contours.at(1));
	//Rect PicROI2;
	PicROI.x = FirstRect.x + ROIOutRect.x;
	PicROI.y = FirstRect.y + ROIOutRect.y;
	PicROI.width = ROIOutRect.width;
	PicROI.height = ROIOutRect.height;
	//PicROI2.x = FirstRect.x + ROIOutRect2.x;
	//PicROI2.y = FirstRect.y + ROIOutRect2.y;
	//PicROI2.width = ROIOutRect2.width;
	//PicROI2.height = ROIOutRect2.height;
	return PicROI;

}

Rect C3DLaserSensorFeatureExtraction::getRect(const int &Threshold, const Mat &Img, int uplimit, int downlimint, int leftlimit, int rightlimit, bool full)
{
	Rect defaultRect;
	defaultRect.x = 0;
	defaultRect.y = 0;
	defaultRect.width = 0;
	defaultRect.height = 0;

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultRect;
	}
	if (Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray, FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement = getStructuringElement(MORPH_ELLIPSE, CKernalSize);
	morphologyEx(ImgFirstROIOpen, ImgFirstROI, MORPH_CLOSE, StructurElement);

	Mat ImgBinary;
	vector<vector<Point>> Contours;
	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;
	Rect ExtendRect;  //扩展后的感兴趣区域

	if (Threshold <= 1)
	{
		return defaultRect;
	}
	if ((Threshold < 30) && (!full))
	{
		return defaultRect;
	}

	threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);
	findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	std::sort(Contours.begin(), Contours.end(), LengthDown);
	if (!Contours.empty())
	{
		ROIOutRect = boundingRect(Contours.at(0));
		//将感兴趣区域的矩形框在整张图片中表达出来
	}
	else
	{
		return defaultRect;
	}
	//Rect ROIOutRect2 = boundingRect(Contours.at(1));
	//Rect PicROI2;
	PicROI.x = FirstRect.x + ROIOutRect.x;
	PicROI.y = FirstRect.y + ROIOutRect.y;
	PicROI.width = ROIOutRect.width;
	PicROI.height = ROIOutRect.height;
	//PicROI2.x = FirstRect.x + ROIOutRect2.x;
	//PicROI2.y = FirstRect.y + ROIOutRect2.y;
	//PicROI2.width = ROIOutRect2.width;
	//PicROI2.height = ROIOutRect2.height;
	return PicROI;

}

Mat C3DLaserSensorFeatureExtraction::LaserSensorStudExtraction2(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, int Threshold/* =150 */, int ArcLengthValue/* =50 */, Size2i KernelSize/* =Size */)  //返回柱面拟合所需的点云数据
{
    Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultAllPoints;
	}
	if(Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat allPoints;

	//对其先进行闭运算 modified in 20150515
	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray,FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement=getStructuringElement(MORPH_ELLIPSE,CKernalSize);
	morphologyEx(ImgFirstROIOpen,ImgFirstROI,MORPH_CLOSE,StructurElement); 

	Mat ImgBinary;
	vector<vector<Point>> Contours;
	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;
	Rect ExtendRect;  //扩展后的感兴趣区域
	//可以将阈值设置的高一些,可以减少初始提取感兴趣区域的面积
	//设置过大的阈值,导致感兴趣区域不能包含所有光刀区域
	threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
	//寻找轮廓findContours
	findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	//findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);//180601似乎无区别
	//自定义函数 按轮廓周长大小排序
	std::sort(Contours.begin(), Contours.end(), LengthDown);
	for (vector<vector<Point>>::size_type ContoursNum = 0; ContoursNum < Contours.size(); ContoursNum++)  //遍历找出所有大于弧长阈值的轮廓线
	{
		if (arcLength(Contours.at(ContoursNum), true) > ArcLengthValue)
		{
			//mininal up-right bounding rectangle
			ROIOutRect = boundingRect(Contours.at(ContoursNum));
			//将感兴趣区域的矩形框在整张图片中表达出来
			PicROI.x = FirstRect.x + ROIOutRect.x;
			PicROI.y = FirstRect.y + ROIOutRect.y;
			PicROI.width = ROIOutRect.width;
			PicROI.height = ROIOutRect.height;
			//修正感兴趣区域
			//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
			Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
			//不用进行感兴趣区域的复制
			Mat LaserStripeROI(ImgGray, ExtendRect);
			Mat LaserStripSmoothedUchar, LaserStripSmoothed;
			GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
			LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换
			//确定光刀线的初始中心位置
			Mat Index = OriginalCenter(LaserStripSmoothed);

			if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
			{
				LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
				return defaultAllPoints;
			}

			//计算光刀线的边界阈值，确定光刀线的宽度
			Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index); 

			if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
			{
				LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
				return defaultAllPoints;
			}

			//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
			Mat LaserROIStripeCenters = DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

			for (int i = 0; i != LaserROIStripeCenters.rows;)
			{
			    int num1 = 0, num2 = 0;
				double avg1 = 0, avg2 = 0;
				//向上最多取5个点比较一次
				for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--) //180517 原版
				{
                    avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
					num1 ++;
				}
				avg1 = avg1 / num1;
				if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10) //180517 原版
				{
					deleteRow(LaserROIStripeCenters, i);
					continue;
				}

				//向下最多取5个点再比较一次
				for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++) //180517 原版
				{
					avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
					num2 ++;
				}
				avg2 = avg2 / num2;
				if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10) //180517 原版
				{
					deleteRow(LaserROIStripeCenters, i);
					continue;
				}

				i++;
			}

			if (LaserROIStripeCenters.rows < 50)
				continue;
			//将感兴趣区域的光刀中心存储 
			allPoints.push_back(LaserROIStripeCenters);
		} 
		else
		{
			break;  
		}  
	}

	if (allPoints.empty())
		return defaultAllPoints;

	return allPoints;
}

int C3DLaserSensorFeatureExtraction::Get1DMaxEntropyThreshold(const MatND &HistGram)
{
	int  X, Y, Amount = 0;
	double HistGramD[256];
	double SumIntegral, EntropyBack, EntropyFore, MaxEntropy;
	int MinValue = 255, MaxValue = 0;
	int Threshold = 0;

	for (MinValue = 0; MinValue < 256 && HistGram.at<float>(MinValue) == 0; MinValue++);
	for (MaxValue = 255; MaxValue > MinValue && HistGram.at<float>(MinValue) == 0; MaxValue--);
	if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色             
	if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

	for (Y = MinValue; Y <= MaxValue; Y++) Amount += HistGram.at<float>(Y);        //  像素总数

	for (Y = MinValue; Y <= MaxValue; Y++)   HistGramD[Y] = (double)HistGram.at<float>(Y) / Amount + 1e-17;

	MaxEntropy = (double)MinValue;
	for (Y = MinValue + 1; Y < MaxValue; Y++)
	{
		SumIntegral = 0;
		for (X = MinValue; X <= Y; X++) SumIntegral += HistGramD[X];
		EntropyBack = 0;
		for (X = MinValue; X <= Y; X++) EntropyBack += (-HistGramD[X] / SumIntegral * log(HistGramD[X] / SumIntegral));
		EntropyFore = 0;
		for (X = Y + 1; X <= MaxValue; X++) EntropyFore += (-HistGramD[X] / (1 - SumIntegral) * log(HistGramD[X] / (1 - SumIntegral)));
		if (MaxEntropy < EntropyBack + EntropyFore)
		{
			Threshold = Y;
			MaxEntropy = EntropyBack + EntropyFore;
		}
	}
	return Threshold;
}

int C3DLaserSensorFeatureExtraction::GetIterativeBestThreshold(const MatND &HistGram)
{
	int X, Iter = 0;
	int MeanValueOne, MeanValueTwo, SumOne, SumTwo, SumIntegralOne, SumIntegralTwo;
	int MinValue, MaxValue;
	int Threshold, NewThreshold;

	for (MinValue = 0; MinValue < 256 && HistGram.at<float>(MinValue) == 0; MinValue++);
	for (MaxValue = 255; MaxValue > MinValue && HistGram.at<float>(MinValue) == 0; MaxValue--);

	if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色             
	if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

	Threshold = MinValue;
	NewThreshold = (MaxValue + MinValue) >> 1;
	while (Threshold != NewThreshold)    // 当前后两次迭代的获得阈值相同时，结束迭代    
	{
		SumOne = 0; SumIntegralOne = 0;
		SumTwo = 0; SumIntegralTwo = 0;
		Threshold = NewThreshold;
		for (X = MinValue; X <= Threshold; X++)         //根据阈值将图像分割成目标和背景两部分，求出两部分的平均灰度值      
		{
			SumIntegralOne += HistGram.at<float>(X) * X;
			SumOne += HistGram.at<float>(X);
		}
		if (SumOne == 0)
		{
			MeanValueOne = 0;
		}
		else
		{
			MeanValueOne = SumIntegralOne / SumOne;
		}
		for (X = Threshold + 1; X <= MaxValue; X++)
		{
			SumIntegralTwo += HistGram.at<float>(X) * X;
			SumTwo += HistGram.at<float>(X);
		}
		if (SumTwo == 0)
		{
			MeanValueTwo = 0;
		}
		else
		{
			MeanValueTwo = SumIntegralTwo / SumTwo;
		}		
		NewThreshold = (MeanValueOne + MeanValueTwo) >> 1;       //求出新的阈值
		Iter++;
		if (Iter >= 1000) return -1;
	}
	return Threshold;
}

int C3DLaserSensorFeatureExtraction::GetOSTUThreshold(MatND HistGram)
{
	int X, Y, Amount = 0;
	int PixelBack = 0, PixelFore = 0, PixelIntegralBack = 0, PixelIntegralFore = 0, PixelIntegral = 0;
	double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma;              // 类间方差;
	int MinValue, MaxValue;
	int Threshold = 0;

	for (MinValue = 0; MinValue < 256 && HistGram.at<float>(MinValue) == 0; MinValue++);
	for (MaxValue = 255; MaxValue > MinValue && HistGram.at<float>(MinValue) == 0; MaxValue--);
	if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色             
	if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

	for (Y = MinValue; Y <= MaxValue; Y++) Amount += HistGram.at<float>(Y);        //  像素总数

	PixelIntegral = 0;
	for (Y = MinValue; Y <= MaxValue; Y++) PixelIntegral += HistGram.at<float>(Y) * Y;
	SigmaB = -1;
	for (Y = MinValue; Y < MaxValue; Y++)
	{
		PixelBack = PixelBack + HistGram.at<float>(Y);
		PixelFore = Amount - PixelBack;
		OmegaBack = (double)PixelBack / Amount;
		OmegaFore = (double)PixelFore / Amount;
		PixelIntegralBack += HistGram.at<float>(Y) * Y;
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;
		MicroBack = (double)PixelIntegralBack / PixelBack;
		MicroFore = (double)PixelIntegralFore / PixelFore;
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
		if (Sigma > SigmaB)
		{
			SigmaB = Sigma;
			Threshold = Y;
		}
	}
	return Threshold;
}


void VerticalProjection(const Mat &srcImage, int Threshold, Rect &ROIOutRect)//垂直积分投影
{
	Mat srcImageBin;
	int cnt=0;
	//threshold(srcImage, srcImageBin, 120, 255, CV_THRESH_BINARY_INV);
	threshold(srcImage, srcImageBin, Threshold, 255, CV_THRESH_BINARY);

	int *colswidth = new int[srcImage.cols];  //申请src.image.cols个int型的内存空间
	memset(colswidth, 0, srcImage.cols * 4);  //数组必须赋初值为零，否则出错。无法遍历数组。
	//  memset(colheight,0,src->width*4);  
	// CvScalar value; 
	int value;
	for (int i = 0; i < srcImage.cols; i++)
		for (int j = 0; j < srcImage.rows; j++)
		{
			//value=cvGet2D(src,j,i);
			value = srcImageBin.at<uchar>(j, i);
			if (value == 255)
			{
				colswidth[i]++; //统计每列的白色像素点  
			}
		}

	for (int i = 0; i < srcImage.cols; i++)
	{
		if (colswidth[i] > 0)
		{
			cnt++;
		}
		else
		{
			cnt = 0;
		}
		if (cnt > 4)
		{
			ROIOutRect.x = i - 4;
			cnt = 0;
			break;
		}
	}
	for (int i = srcImage.cols-1; i > -1; i--)
	{
		if (colswidth[i] > 0)
		{
			cnt++;
		}
		else
		{
			cnt = 0;
		}
		if (cnt > 4)
		{
			ROIOutRect.width = i + 4 - ROIOutRect.x + 1;
			cnt = 0;
			break;
		}
	}
	//if (studflag)
	//{
		for (int i = 0; i < ROIOutRect.width; ++i)
		{
			if (colswidth[i + ROIOutRect.x] > 0)
			{
				cnt = 0;
			}
			else
			{
				++cnt;
			}
			if (cnt > 10)
			{
				ROIOutRect.width = 0;
				cnt = 0;
				break;
			}
		}
	//}

	//Mat histogramImage(srcImage.rows, srcImage.cols, CV_8UC1);
	//for (int i = 0; i < srcImage.rows; i++)
	//	for (int j = 0; j < srcImage.cols; j++)
	//	{
	//		value = 255;  //背景设置为白色。 
	//		histogramImage.at<uchar>(i, j) = value;
	//	}
	//for (int i = 0; i < srcImage.cols; i++)
	//	for (int j = 0; j < colswidth[i]; j++)
	//	{
	//		value = 0;  //直方图设置为黑色
	//		histogramImage.at<uchar>(srcImage.rows - 1 - j, i) = value;
	//	}
	//imshow(" 垂直积分投影图", histogramImage);
	//waitKey(0);
	delete[] colswidth;//释放前面申请的空间
}

void HorizonProjection(const Mat &srcImage, int Threshold, Rect &ROIOutRect)//水平积分投影
{
	Mat srcImageBin;
	int cnt = 0;
	//threshold(srcImage, srcImageBin, 120, 255, CV_THRESH_BINARY_INV);
	threshold(srcImage, srcImageBin, Threshold, 255, CV_THRESH_BINARY);
	int *rowswidth = new int[srcImage.rows];  //申请src.image.rows个int型的内存空间
	memset(rowswidth, 0, srcImage.rows * 4);  //数组必须赋初值为零，否则出错。无法遍历数组。
	int value;
	for (int i = 0; i<srcImage.rows; i++)
		for (int j = 0; j<srcImage.cols; j++)
		{
			//value=cvGet2D(src,j,i);
			value = srcImageBin.at<uchar>(i, j);
			if (value == 255)
			{
				rowswidth[i]++; //统计每行的白色像素点  
			}
		}
	for (int i = 0; i < srcImage.rows; i++)
	{
		if (rowswidth[i] > 0)
		{
			cnt++;
		}
		else
		{
			cnt = 0;
		}
		//if ((cnt > 4) || (!studflag))
		if (cnt > 4)
		{
			ROIOutRect.y = i - 4;
			cnt = 0;
			break;
		}
	}
	for (int i = srcImage.rows - 1; i > -1; i--)
	{
		if (rowswidth[i] > 0)
		{
			cnt++;
		}
		else
		{
			cnt = 0;
		}
		//if ((cnt > 4) || (!studflag))
		if (cnt > 4)
		{
			ROIOutRect.height = i + 4 - ROIOutRect.y + 1;
			cnt = 0;
			break;
		}
	}
	//Mat histogramImage(srcImage.rows, srcImage.cols, CV_8UC1);
	//for (int i = 0; i<srcImage.rows; i++)
	//	for (int j = 0; j<srcImage.cols; j++)
	//	{
	//		value = 255;  //背景设置为白色。 
	//		histogramImage.at<uchar>(i, j) = value;
	//	}
	////imshow("d", histogramImage);
	//for (int i = 0; i<srcImage.rows; i++)
	//	for (int j = 0; j<rowswidth[i]; j++)
	//	{
	//		value = 0;  //直方图设置为黑色
	//		histogramImage.at<uchar>(i, j) = value;
	//	}
	//imshow("水平积分投影图", histogramImage);
	delete[] rowswidth;//释放前面申请的空间

}

Mat C3DLaserSensorFeatureExtraction::LaserSensorStudExtraction2(string strImagePath, Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, int ArcLengthValue/* =50 */, Size2i KernelSize/* =Size */)  //返回柱面拟合所需的点云数据
{
	Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultAllPoints;
	}
	if (Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat allPoints;

	//对其先进行闭运算 modified in 20150515
	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray, FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement = getStructuringElement(MORPH_ELLIPSE, CKernalSize);
	morphologyEx(ImgFirstROIOpen, ImgFirstROI, MORPH_CLOSE, StructurElement);

	//imwrite(strImagePath, ImgFirstROI);

	Mat ImgBinary;
	vector<vector<Point>> Contours;
	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;
	Rect ExtendRect;  //扩展后的感兴趣区域
	int Threshold;

	const int channels[1] = { 0 };
	const int histSize[1] = { 256 };
	float hranges[2] = { 0, 255 };
	const float* ranges[1] = { hranges };
	MatND hist;
	calcHist(&ImgFirstROI, 1, channels, Mat(), hist, 1, histSize, ranges);

	//for (int i = 0; i != 30; ++i)
	//{
	//	hist.at<float>(i) = 0;
	//}

	Threshold = GetIterativeBestThreshold(hist);
	//Threshold = GetOSTUThreshold(hist);
	//Threshold = 45;

	//double maxVal = 0;
	//double minVal = 0;
	////找到直方图中的最大值和最小值
	//minMaxLoc(hist, &minVal, &maxVal, 0, 0);
	//int size = hist.rows;
	//Mat histImg(size, size, CV_8U, Scalar(255));
	//// 设置最大峰值为图像高度的90%
	//int hpt = static_cast<int>(0.9*size);
	//for (int h = 0; h<size; h++)
	//{
	//	float binVal = hist.at<float>(h);
	//	int intensity = static_cast<int>(binVal*hpt / maxVal);
	//	line(histImg, Point(h, size), Point(h, size - intensity), Scalar::all(0));
	//}
	//imwrite(strImagePath, histImg);


	//可以将阈值设置的高一些,可以减少初始提取感兴趣区域的面积
	//设置过大的阈值,导致感兴趣区域不能包含所有光刀区域
	threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);//主要为了提取光刀线的轮廓

	//adaptiveThreshold(ImgFirstROI, ImgBinary, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 3, -1);

	//imwrite(strImagePath, ImgBinary);

	//Mat VP;
	//VP = VerticalProjection(ImgFirstROI, Threshold);
	//Mat HP;
	//HP = HorizonProjection(ImgFirstROI, Threshold);
	//waitKey(0);
	//VerticalProjection(ImgFirstROI, Threshold, ROIOutRect);
	//HorizonProjection(ImgFirstROI, Threshold, ROIOutRect);
	//waitKey(0);

	if (ROIOutRect.width > 140)
	{
		//mininal up-right bounding rectangle
		//ROIOutRect = boundingRect(Contours.at(ContoursNum));
		//将感兴趣区域的矩形框在整张图片中表达出来
		PicROI.x = FirstRect.x + ROIOutRect.x;
		PicROI.y = FirstRect.y + ROIOutRect.y;
		PicROI.width = ROIOutRect.width;
		PicROI.height = ROIOutRect.height;

		//修正感兴趣区域
		//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
		Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
		//不用进行感兴趣区域的复制
		Mat LaserStripeROI(ImgGray, ExtendRect);
		Mat LaserStripSmoothedUchar, LaserStripSmoothed;
		GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
		LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换

		//imwrite(strImagePath, LaserStripSmoothed);

		//确定光刀线的初始中心位置
		Mat Index = OriginalCenter(LaserStripSmoothed);

		if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
		{
			LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
			return defaultAllPoints;
		}

		//计算光刀线的边界阈值，确定光刀线的宽度
		Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index);

		if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
		{
			LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
			return defaultAllPoints;
		}

		//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
		Mat LaserROIStripeCenters = DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

		for (int i = 0; i != LaserROIStripeCenters.rows;)
		{
			int num1 = 0, num2 = 0;
			double avg1 = 0, avg2 = 0;
			//向上最多取5个点比较一次
			for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--) //180517 原版
			{
				avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
				num1++;
			}
			avg1 = avg1 / num1;
			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10) //180517 原版
			{
				deleteRow(LaserROIStripeCenters, i);
				continue;
			}

			//向下最多取5个点再比较一次
			for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++) //180517 原版
			{
				avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
				num2++;
			}
			avg2 = avg2 / num2;
			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10) //180517 原版
			{
				deleteRow(LaserROIStripeCenters, i);
				continue;
			}

			i++;
		}

		//将感兴趣区域的光刀中心存储 
		allPoints.push_back(LaserROIStripeCenters);
	}

	////寻找轮廓findContours
	//findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	////自定义函数 按轮廓周长大小排序
	//std::sort(Contours.begin(), Contours.end(), LengthDown);
	//for (vector<vector<Point>>::size_type ContoursNum = 0; ContoursNum < Contours.size(); ContoursNum++)  //遍历找出所有大于弧长阈值的轮廓线
	//{
	//	if (arcLength(Contours.at(ContoursNum), true) > ArcLengthValue)
	//	{
	//		//mininal up-right bounding rectangle
	//		//ROIOutRect = boundingRect(Contours.at(ContoursNum));
	//		//将感兴趣区域的矩形框在整张图片中表达出来
	//		PicROI.x = FirstRect.x + ROIOutRect.x;
	//		PicROI.y = FirstRect.y + ROIOutRect.y;
	//		PicROI.width = ROIOutRect.width;
	//		PicROI.height = ROIOutRect.height;

	//		//修正感兴趣区域
	//		//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
	//		Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
	//		//不用进行感兴趣区域的复制
	//		Mat LaserStripeROI(ImgGray, ExtendRect);
	//		Mat LaserStripSmoothedUchar, LaserStripSmoothed;
	//		GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
	//		LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换

	//		//imwrite(strImagePath, LaserStripSmoothed);

	//		//确定光刀线的初始中心位置
	//		Mat Index = OriginalCenter(LaserStripSmoothed);

	//		if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
	//		{
	//			LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
	//			return defaultAllPoints;
	//		}

	//		//计算光刀线的边界阈值，确定光刀线的宽度
	//		Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index);

	//		if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
	//		{
	//			LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
	//			return defaultAllPoints;
	//		}

	//		//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
	//		Mat LaserROIStripeCenters = DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

	//		for (int i = 0; i != LaserROIStripeCenters.rows;)
	//		{
	//			int num1 = 0, num2 = 0;
	//			double avg1 = 0, avg2 = 0;
	//			//向上最多取5个点比较一次
	//			for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--) //180517 原版
	//			{
	//				avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
	//				num1++;
	//			}
	//			avg1 = avg1 / num1;
	//			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10) //180517 原版
	//			{
	//				deleteRow(LaserROIStripeCenters, i);
	//				continue;
	//			}

	//			//向下最多取5个点再比较一次
	//			for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++) //180517 原版
	//			{
	//				avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
	//				num2++;
	//			}
	//			avg2 = avg2 / num2;
	//			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10) //180517 原版
	//			{
	//				deleteRow(LaserROIStripeCenters, i);
	//				continue;
	//			}

	//			i++;
	//		}

	//		if (LaserROIStripeCenters.rows < 50)
	//			continue;
	//		//将感兴趣区域的光刀中心存储 
	//		allPoints.push_back(LaserROIStripeCenters);
	//	}
	//	else
	//	{
	//		break;
	//	}
	//}

	if (allPoints.empty())
		return defaultAllPoints;

	return allPoints;
}

Mat C3DLaserSensorFeatureExtraction::LaserSensorStudExtraction(int num, const Mat &Img, int uplimit, int downlimint, int leftlimit, int rightlimit, bool studFlag, int widthValue, Size2i KernelSize/* =Size */)
{
	Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultAllPoints;
	}
	if (Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat allPoints;

	//对其先进行闭运算 modified in 20150515
	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray, FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement = getStructuringElement(MORPH_ELLIPSE, CKernalSize);
	morphologyEx(ImgFirstROIOpen, ImgFirstROI, MORPH_CLOSE, StructurElement);

	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;
	int Threshold;

	const int channels[1] = { 0 };
	const int histSize[1] = { 256 };
	float hranges[2] = { 0, 255 };
	const float* ranges[1] = { hranges };
	MatND hist;
	calcHist(&ImgFirstROI, 1, channels, Mat(), hist, 1, histSize, ranges);

	Threshold = GetIterativeBestThreshold(hist);
	//Threshold = Get1DMaxEntropyThreshold(hist);

	//string strHistPath = "Hist.txt";
	//LaserSensorIO.WriteRowFirstAPP(strHistPath, hist);

	Mat test(1, 6, CV_64FC1, cvScalar(-1));
	test.at<double>(0, 0) = num;
	test.at<double>(0, 1) = Threshold;
	test.at<double>(0, 2) = uplimit;
	test.at<double>(0, 3) = downlimint;
	test.at<double>(0, 4) = leftlimit;
	test.at<double>(0, 5) = rightlimit;
	stringstream a;
	string strNum;
	a << num;
	a >> strNum;
	string strThresholdPath = "Threshold.txt";
	LaserSensorIO.WriteRowFirstAPP(strThresholdPath, test);

	if (studFlag)
	{
		//if (Threshold < 30) return defaultAllPoints;
		VerticalProjection(ImgFirstROI, Threshold, ROIOutRect);
		HorizonProjection(ImgFirstROI, Threshold, ROIOutRect);
		if (ROIOutRect.width < widthValue * 1.0 / 2.0)
		{
			Mat ImgBinary;
			vector<vector<Point>> Contours;
			threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
			//寻找轮廓findContours
			findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
			//自定义函数 按轮廓周长大小排序
			std::sort(Contours.begin(), Contours.end(), LengthDown);
			if (!Contours.empty())
			{
				ROIOutRect = boundingRect(Contours.at(0));
				//将感兴趣区域的矩形框在整张图片中表达出来
			}
		}
	}
	else
	{
		Mat ImgBinary;
		vector<vector<Point>> Contours;
		threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
		//寻找轮廓findContours
		findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		//自定义函数 按轮廓周长大小排序
		std::sort(Contours.begin(), Contours.end(), LengthDown);
		ROIOutRect = boundingRect(Contours.at(0));
		//将感兴趣区域的矩形框在整张图片中表达出来
	}
	PicROI.x = FirstRect.x + ROIOutRect.x;
	PicROI.y = FirstRect.y + ROIOutRect.y;
	PicROI.width = ROIOutRect.width;
	PicROI.height = ROIOutRect.height;
	if (studFlag)
	{
		if (ROIOutRect.width < widthValue * 1.0 / 2.0)
		{
			return defaultAllPoints;
		}
	}
	else
	{
		if (PicROI.x < 400)
		{
			PicROI.width = PicROI.width - (400 - PicROI.x);
			PicROI.x = 400;
		}
		if ((PicROI.x + PicROI.width) > 900)
		{
			PicROI.width = 900 - PicROI.x;
		}
		if (PicROI.width < 1)
		{
			return defaultAllPoints;
		}
	}

	//修正感兴趣区域
	//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
	Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
	//不用进行感兴趣区域的复制
	Mat LaserStripeROI(ImgGray, ExtendRect);
	Mat LaserStripSmoothedUchar, LaserStripSmoothed;
	GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
	LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换

	//imwrite(strImagePath, LaserStripSmoothed);
	//imshow("", LaserStripSmoothedUchar);
	//waitKey(0);

	//确定光刀线的初始中心位置
	Mat Index = OriginalCenter(LaserStripSmoothed);

	if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
	{
		LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
		return defaultAllPoints;
	}

	//计算光刀线的边界阈值，确定光刀线的宽度
	Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index);

	if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
	{
		LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
		return defaultAllPoints;
	}

	//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
	Mat LaserROIStripeCenters= DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

	for (int i = 0; i != LaserROIStripeCenters.rows;)
	{
		int num1 = 0, num2 = 0;
		double avg1 = 0, avg2 = 0;
		//向上最多取5个点比较一次
		for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--) //180517 原版
		{
			avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
			num1++;
		}
		if (num1 != 0)
		{
			avg1 = avg1 / num1;
			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10) //180517 原版
			{
				deleteRow(LaserROIStripeCenters, i);
				continue;
			}
		}

		//向下最多取5个点再比较一次
		for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++) //180517 原版
		{
			avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
			num2++;
		}
		if (num2 != 0)
		{
			avg2 = avg2 / num2;
			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10) //180517 原版
			{
				deleteRow(LaserROIStripeCenters, i);
				continue;
			}
		}

		i++;
	}

	//将感兴趣区域的光刀中心存储 
	allPoints.push_back(LaserROIStripeCenters);

	if (allPoints.empty())
		return defaultAllPoints;

	return allPoints;
}

Mat C3DLaserSensorFeatureExtraction::LaserSensorStudExtraction(const int &Threshold, int num, const Mat &Img, int uplimit, int downlimint, int leftlimit, int rightlimit, bool studFlag, int widthValue, Size2i KernelSize/* =Size */)
{
	Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultAllPoints;
	}
	if (Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat allPoints;

	//对其先进行闭运算 modified in 20150515
	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray, FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement = getStructuringElement(MORPH_ELLIPSE, CKernalSize);
	morphologyEx(ImgFirstROIOpen, ImgFirstROI, MORPH_CLOSE, StructurElement);

	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;

	//string strHistPath = "Hist.txt";
	//LaserSensorIO.WriteRowFirstAPP(strHistPath, hist);

	if (studFlag)
	{
		//if (Threshold < 30) return defaultAllPoints;
		VerticalProjection(ImgFirstROI, Threshold, ROIOutRect);
		HorizonProjection(ImgFirstROI, Threshold, ROIOutRect);
		if (ROIOutRect.width < widthValue * 1.0 / 2.0)
		{
			Mat ImgBinary;
			vector<vector<Point>> Contours;
			threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
			//寻找轮廓findContours
			findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
			//自定义函数 按轮廓周长大小排序
			std::sort(Contours.begin(), Contours.end(), LengthDown);
			if (!Contours.empty())
			{
				ROIOutRect = boundingRect(Contours.at(0));
				//将感兴趣区域的矩形框在整张图片中表达出来
			}
		}
	}
	else
	{
		Mat ImgBinary;
		vector<vector<Point>> Contours;
		threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
		//寻找轮廓findContours
		findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		//自定义函数 按轮廓周长大小排序
		std::sort(Contours.begin(), Contours.end(), LengthDown);
		ROIOutRect = boundingRect(Contours.at(0));
		//将感兴趣区域的矩形框在整张图片中表达出来
	}
	PicROI.x = FirstRect.x + ROIOutRect.x;
	PicROI.y = FirstRect.y + ROIOutRect.y;
	PicROI.width = ROIOutRect.width;
	PicROI.height = ROIOutRect.height;
	if (studFlag)
	{
		if (ROIOutRect.width < widthValue * 1.0 / 2.0)
		{
			return defaultAllPoints;
		}
	}
	else
	{
		if (PicROI.x < 400)
		{
			PicROI.width = PicROI.width - (400 - PicROI.x);
			PicROI.x = 400;
		}
		if ((PicROI.x + PicROI.width) > 900)
		{
			PicROI.width = 900 - PicROI.x;
		}
		if (PicROI.width < 1)
		{
			return defaultAllPoints;
		}
	}

	//修正感兴趣区域
	//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
	Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
	//不用进行感兴趣区域的复制
	Mat LaserStripeROI(ImgGray, ExtendRect);
	Mat LaserStripSmoothedUchar, LaserStripSmoothed;
	GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
	LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换

	//imwrite(strImagePath, LaserStripSmoothed);
	//imshow("", LaserStripSmoothedUchar);
	//waitKey(0);

	//确定光刀线的初始中心位置
	Mat Index = OriginalCenter(LaserStripSmoothed);

	if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
	{
		LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
		return defaultAllPoints;
	}

	//计算光刀线的边界阈值，确定光刀线的宽度
	Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index);

	if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
	{
		LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
		return defaultAllPoints;
	}

	//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
	Mat LaserROIStripeCenters = DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

	for (int i = 0; i != LaserROIStripeCenters.rows;)
	{
		int num1 = 0, num2 = 0;
		double avg1 = 0, avg2 = 0;
		//向上最多取5个点比较一次
		for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--) //180517 原版
		{
			avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
			num1++;
		}
		if (num1 != 0)
		{
			avg1 = avg1 / num1;
			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10) //180517 原版
			{
				deleteRow(LaserROIStripeCenters, i);
				continue;
			}
		}

		//向下最多取5个点再比较一次
		for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++) //180517 原版
		{
			avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
			num2++;
		}
		if (num2 != 0)
		{
			avg2 = avg2 / num2;
			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10) //180517 原版
			{
				deleteRow(LaserROIStripeCenters, i);
				continue;
			}
		}

		i++;
	}

	//将感兴趣区域的光刀中心存储 
	allPoints.push_back(LaserROIStripeCenters);

	if (allPoints.empty())
		return defaultAllPoints;

	return allPoints;
}

Mat C3DLaserSensorFeatureExtraction::LaserSensorStudExtraction15(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, Rect &PicROI, Size2i KernelSize/* =Size */)
{
	Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultAllPoints;
	}
	if (Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	Mat allPoints;

	//对其先进行闭运算 modified in 20150515
	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray, FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement = getStructuringElement(MORPH_ELLIPSE, CKernalSize);
	morphologyEx(ImgFirstROIOpen, ImgFirstROI, MORPH_CLOSE, StructurElement);

	Rect ROIOutRect;  //轮廓的外接矩形	
	int Threshold;

	const int channels[1] = { 0 };
	const int histSize[1] = { 256 };
	float hranges[2] = { 0, 255 };
	const float* ranges[1] = { hranges };
	MatND hist;
	calcHist(&ImgFirstROI, 1, channels, Mat(), hist, 1, histSize, ranges);

	Threshold = GetIterativeBestThreshold(hist);
	if (Threshold < 10) return defaultAllPoints;

	Mat ImgBinary;
	vector<vector<Point>> Contours;
	threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
	//寻找轮廓findContours
	findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	//自定义函数 按轮廓周长大小排序
	std::sort(Contours.begin(), Contours.end(), LengthDown);
	ROIOutRect = boundingRect(Contours.at(0));
	//将感兴趣区域的矩形框在整张图片中表达出来

	PicROI.x = FirstRect.x + ROIOutRect.x;
	PicROI.y = FirstRect.y + ROIOutRect.y;
	PicROI.width = ROIOutRect.width;
	PicROI.height = ROIOutRect.height;

	//if (PicROI.x < 400)
	//{
	//	PicROI.width = PicROI.width - (400 - PicROI.x);
	//	PicROI.x = 400;
	//}
	//if ((PicROI.x + PicROI.width) > 900)
	//{
	//	PicROI.width = 900 - PicROI.x;
	//}
	//if (PicROI.width < 1)
	//{
	//	return defaultAllPoints;
	//}

	//修正感兴趣区域
	//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
	Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
	//不用进行感兴趣区域的复制
	Mat LaserStripeROI(ImgGray, ExtendRect);
	Mat LaserStripSmoothedUchar, LaserStripSmoothed;
	GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
	LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换

	//imwrite(strImagePath, LaserStripSmoothed);
	//imshow("", LaserStripSmoothedUchar);
	//waitKey(0);

	//确定光刀线的初始中心位置
	Mat Index = OriginalCenter(LaserStripSmoothed);

	if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
	{
		LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
		return defaultAllPoints;
	}

	//计算光刀线的边界阈值，确定光刀线的宽度
	Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index);

	if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
	{
		LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
		return defaultAllPoints;
	}

	//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
	Mat LaserROIStripeCenters = DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

	for (int i = 0; i != LaserROIStripeCenters.rows;)
	{
		int num1 = 0, num2 = 0;
		double avg1 = 0, avg2 = 0;
		//向上最多取5个点比较一次
		for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--) //180517 原版
		{
			avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
			num1++;
		}
		if (num1 != 0)
		{
			avg1 = avg1 / num1;
			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10) //180517 原版
			{
				deleteRow(LaserROIStripeCenters, i);
				continue;
			}
		}

		//向下最多取5个点再比较一次
		for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++) //180517 原版
		{
			avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
			num2++;
		}
		if (num2 != 0)
		{
			avg2 = avg2 / num2;
			if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10) //180517 原版
			{
				deleteRow(LaserROIStripeCenters, i);
				continue;
			}
		}

		i++;
	}

	//将感兴趣区域的光刀中心存储 
	allPoints.push_back(LaserROIStripeCenters);

	if (allPoints.empty())
		return defaultAllPoints;

	return allPoints;
}

Mat C3DLaserSensorFeatureExtraction::LaserSensorStudExtraction(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, int Threshold/* =150 */, int ArcLengthValue/* =50 */, Size2i KernelSize/* =Size */)  //返回柱面拟合所需的点云数据
{
    Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));

	Rect FirstRect;
	FirstRect.x = leftlimit;
	FirstRect.y = uplimit;
	FirstRect.width = rightlimit - leftlimit + 1;
	FirstRect.height = downlimint - uplimit + 1;

	Mat ImgGray;
	if (Img.rows < 1)
	{
		return defaultAllPoints;
	}
	if(Img.channels() == 3)
	{
		cvtColor(Img, ImgGray, CV_BGR2GRAY);
	}
	else
	{
		ImgGray = Img;
	}

	vector<Mat> allPoints;

	//对其先进行闭运算 modified in 20150515
	Mat ImgFirstROI;
	Mat ImgFirstROIOpen(ImgGray,FirstRect);
	const Size2i CKernalSize(5, 5);
	Mat StructurElement=getStructuringElement(MORPH_ELLIPSE,CKernalSize);
	morphologyEx(ImgFirstROIOpen,ImgFirstROI,MORPH_CLOSE,StructurElement); 

	Mat ImgBinary;
	vector<vector<Point>> Contours;
	Rect ROIOutRect;  //轮廓的外接矩形	
	Rect PicROI;
	Rect ExtendRect;  //扩展后的感兴趣区域
	//可以将阈值设置的高一些,可以减少初始提取感兴趣区域的面积
	//设置过大的阈值,导致感兴趣区域不能包含所有光刀区域
	threshold(ImgFirstROI, ImgBinary, Threshold, 255, THRESH_TOZERO);  //主要为了提取光刀线的轮廓
	//寻找轮廓findContours
	findContours(ImgBinary, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	//自定义函数 按轮廓周长大小排序
	std::sort(Contours.begin(), Contours.end(), LengthDown);
	for (vector<vector<Point>>::size_type ContoursNum = 0; ContoursNum < Contours.size(); ContoursNum++)  //遍历找出所有大于弧长阈值的轮廓线
	{
		if (arcLength(Contours.at(ContoursNum), true) > ArcLengthValue)
		{
			//mininal up-right bounding rectangle
			ROIOutRect = boundingRect(Contours.at(ContoursNum));
			//将感兴趣区域的矩形框在整张图片中表达出来
			PicROI.x = FirstRect.x + ROIOutRect.x;
			PicROI.y = FirstRect.y + ROIOutRect.y;
			PicROI.width = ROIOutRect.width;
			PicROI.height = ROIOutRect.height;
			//修正感兴趣区域
			//在不超出图片边界的情况下，将感兴趣区域向上下分别扩展10个像素
			Rect ExtendRect = ExtendedROI(ImgGray, PicROI);
			//不用进行感兴趣区域的复制
			Mat LaserStripeROI(ImgGray, ExtendRect);
			Mat LaserStripSmoothedUchar, LaserStripSmoothed;
			GaussianBlur(LaserStripeROI, LaserStripSmoothedUchar, KernelSize, 0, 0);
			LaserStripSmoothedUchar.convertTo(LaserStripSmoothed, CV_64FC1);  //类型转换
			//确定光刀线的初始中心位置
			Mat Index = OriginalCenter(LaserStripSmoothed);

			if (Index.cols != LaserStripSmoothed.cols)  //不相等则不进行计算 modified in 20151030
			{
				LaserSensorIO.WriteLog("Error(11):Index与LaserStripeSmoothed列数不等!");
				return defaultAllPoints;
			}

			//计算光刀线的边界阈值，确定光刀线的宽度
			Mat EdgeThresholdValue = DetermineEdgeValue(LaserStripSmoothed, Index); 

			if (EdgeThresholdValue.cols != LaserStripSmoothed.cols) /////// 不相等则不进行计算 modified in 20151030
			{
				LaserSensorIO.WriteLog("Error(11):EdgeThresholdValue与LaserStripeSmoothed列数不等!");
				return defaultAllPoints;
			}

			//利用初始中心和光刀线的边界阈值计算感兴趣区域的光刀线中心
			Mat LaserROIStripeCenters = DetermineCenters(LaserStripSmoothed, Index, EdgeThresholdValue, ExtendRect);

			for (int i = 0; i != LaserROIStripeCenters.rows;)
			{
			    int num1 = 0, num2 = 0;
				double avg1 = 0, avg2 = 0;
				//向上最多取5个点比较一次
				for (int rowNum = i - 1; rowNum > i - 6 && rowNum >= 0; rowNum--)
				{
                    avg1 += LaserROIStripeCenters.at<double>(rowNum, 0);
					num1 ++;
				}
				avg1 = avg1 / num1;
				if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg1) > 10)
				{
					deleteRow(LaserROIStripeCenters, i);
					continue;
				}

				//向下最多取5个点再比较一次
				for (int rowNum = i + 1; rowNum < i + 6 && rowNum < LaserROIStripeCenters.rows; rowNum++)
				{
					avg2 += LaserROIStripeCenters.at<double>(rowNum, 0);
					num2 ++;
				}
				avg2 = avg2 / num2;
				if (abs(LaserROIStripeCenters.at<double>(i, 0) - avg2) > 10)
				{
					deleteRow(LaserROIStripeCenters, i);
					continue;
				}

				i++;
			}

			if (LaserROIStripeCenters.rows < 100)
				continue;
			//将感兴趣区域的光刀中心存储 
			allPoints.push_back(LaserROIStripeCenters);
		} 
		else
		{
			break;  
		}  
	}

	if (allPoints.size() == 1)
	{
		return allPoints[0];
	}
	else if(allPoints.size() == 3)
	{
		if ((allPoints[0].at<double>(0,1) - allPoints[1].at<double>(0,1)) * (allPoints[0].at<double>(0,1) - allPoints[2].at<double>(0,1)) < 0)
		{
			return allPoints[0];
		}
		else if ((allPoints[1].at<double>(0,1) - allPoints[0].at<double>(0,1)) * (allPoints[1].at<double>(0,1) - allPoints[2].at<double>(0,1)) < 0)
		{
			return allPoints[1];
		}
		else
		{
			return allPoints[2];
		}
	}
	else
	{
		return defaultAllPoints;
	}
}

vector<Mat>  C3DLaserSensorFeatureExtraction::StudFit(Mat FittingPoints, double CircleMeanErrorThreshold/* =0.2*/, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)  //柱面返回中心坐标值+法线
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
	vector<Mat> studFeature(2);
	studFeature[0] = defaultStudCenter;

    Point3d axes =  calcPCA(FittingPoints);
	studFeature[1] = Mat(axes).t();  

	//轴线方向投影
	Mat axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	Mat meanPoint(1, FittingPoints.cols, CV_64FC1, cvScalar(0));
	for (int i = 0; i != FittingPoints.cols; i++)
	{
		meanPoint.at<double>(0, i) = cv::mean(FittingPoints.col(i)).val[0];
	}

	Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
		dataAdjust.row(cnt) = FittingPoints.row(cnt) - meanPoint;
    Mat projectedPoints = FittingPoints - dataAdjust * axesMat * axesMat.t();
    //旋转至与XY平面平行
    RotToZInfo rotInfo=RotToZ(projectedPoints, axesMat);

	//Hough圆检测去除异常点
	Mat finalFittingPoints = HoughCircle(projectedPoints, rotInfo.XYRoted, 2.8, 3.0, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
	if (finalFittingPoints.rows < 50)
	{
		LaserSensorIO.WriteLog("Error(11):2_柱面拟合点云数量<50");
		return studFeature;
	}

	//圆拟合
	CircleFit3DInfo studInfo = CircleFit3D(finalFittingPoints, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
    if (studInfo.Center.empty())
	{
		return studFeature;
	} 
	else
	{
		studFeature[0] = studInfo.Center.t();
		return studFeature;
	}
}

vector<Mat>  C3DLaserSensorFeatureExtraction::StudFit15(vector<Mat> &vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold/* =0.2*/, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)  //柱面返回中心坐标值+法线
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
	Mat defaultStudAxes(1, 3, CV_64FC1, Scalar(-1));
	vector<Mat> studFeature(2);
	studFeature[0] = defaultStudCenter;
	studFeature[1] = defaultStudAxes;

	//剔除只有一半的圆弧段
	int maxRows = 0;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows > maxRows)
			maxRows = vecFittingPoints[i].rows;
	}

	Mat FittingPoints;
	vector<bool> ChoosenGroup;
	for (int i = 0; i != vecFittingPoints.size(); ++i)
	{
		if (vecFittingPoints[i].rows >= maxRows * 2.0 / 3.0)
		{
			FittingPoints.push_back(vecFittingPoints[i]);
			ChoosenGroup.push_back(true);
		}
		else
		{
			ChoosenGroup.push_back(false);
		}
	}

	if (FittingPoints.empty())
		return studFeature;

	//if (FittingPoints2.empty())
	//	return studFeature;

	Point3d axes = calcPCA(FittingPoints);
	//axes.x = axes2.x;
	//axes.y = axes1.y;
	//axes.z = axes1.z;	

	//轴线方向投影
	Mat axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	normalize(axesMat, axesMat);

	Mat vecXYRoted;
	Mat vecProjectedPoints;
	Mat FittingPoints2;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows < maxRows * 2.0 / 3.0)
		{
			continue;
		}
		Mat dataAdjust2(vecFittingPoints[i].rows, vecFittingPoints[i].cols, CV_64FC1);
		Mat meanPoint(1, vecFittingPoints[i].cols, CV_64FC1, cvScalar(0));
		//求取平均值
		for (int j = 0; j != vecFittingPoints[i].cols; j++)
		{
			meanPoint.at<double>(0, j) = cv::mean(vecFittingPoints[i].col(j)).val[0];
		}
		for (int j = 0; j != dataAdjust2.rows; j++)
		{
			dataAdjust2.row(j) = vecFittingPoints[i].row(j) - meanPoint;
		}
		vecProjectedPoints = vecFittingPoints[i] - dataAdjust2 * axesMat * axesMat.t();
		RotToZInfo rotInfo2 = RotToZ(vecProjectedPoints, axesMat);
		vecXYRoted = rotInfo2.XYRoted;
		vector<PTS> points;
		double cen_z = vecXYRoted.at<double>(0, 2);
		for (int j = 0; j != vecXYRoted.rows; j++)
		{
			PTS pt(vecXYRoted.at<double>(j, 0), vecXYRoted.at<double>(j, 1));
			points.push_back(pt);
		}
		double cen_x = 0;
		double cen_y = -3;
		double cen_r = -1;
		CircleFitSolver *cfs = new CircleFitSolver();
		if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
		{
			//if ((cen_r > 3.65) && (cen_r < 4.5))
			{
				Mat rot2Z_cen(1, 3, CV_64FC1, Scalar(-1));
				rot2Z_cen.at<double>(0, 0) = cen_x;
				rot2Z_cen.at<double>(0, 1) = cen_y;
				rot2Z_cen.at<double>(0, 2) = cen_z;
				Mat temp = (rotInfo2.R.inv()*(rot2Z_cen.t())).t();
				FittingPoints2.push_back(temp);
			}
		}
	}

	bool cycleFlag = true;
	Point3d axes2;
	Mat NewFittingPoints2;
	NewFittingPoints2.resize(0);
	while ((FittingPoints2.rows != NewFittingPoints2.rows) && (FittingPoints2.rows > 2))
		//while (cycleFlag)
	{
		axes2 = calcPCA(FittingPoints2);
		cycleFlag = false;
		NewFittingPoints2 = FittingPoints2;
		Point3d Point1, Point2;
		Point1.x = cv::mean(NewFittingPoints2.col(0)).val[0];
		Point1.y = cv::mean(NewFittingPoints2.col(1)).val[0];
		Point1.z = cv::mean(NewFittingPoints2.col(2)).val[0];
		Point2.x = Point1.x + 1;
		Point2.y = axes2.y / axes2.x + Point1.y;
		Point2.z = axes2.z / axes2.x + Point1.z;
		vector<double> DistanceOfPointToLine;
		double ab = sqrt(pow((Point2.x - Point1.x), 2.0) + pow((Point2.y - Point1.y), 2.0) + pow((Point2.z - Point1.z), 2.0));
		double DisVar = 0;
		//double DisMax = 0;
		double DisMax2 = 0;
		for (int cnt = 0; cnt != NewFittingPoints2.rows; cnt++)
		{
			double as = sqrt(pow((Point1.x - NewFittingPoints2.at<double>(cnt, 0)), 2.0) + pow((Point1.y - NewFittingPoints2.at<double>(cnt, 1)), 2.0) + pow((Point1.z - NewFittingPoints2.at<double>(cnt, 2)), 2.0));
			double bs = sqrt(pow((NewFittingPoints2.at<double>(cnt, 0) - Point2.x), 2.0) + pow((NewFittingPoints2.at<double>(cnt, 1) - Point2.y), 2.0) + pow((NewFittingPoints2.at<double>(cnt, 2) - Point2.z), 2.0));
			double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
			double sin_A = sqrt(1 - pow(cos_A, 2.0));
			DistanceOfPointToLine.push_back(as * sin_A);
			//if ((similarVec[cnt] != cnt) && (DisMax < as * sin_A))
			//{
			//	DisMax = as * sin_A;
			//	MaxIndex = cnt;
			//}
			if (DisMax2 < as * sin_A)
			{
				DisMax2 = as * sin_A;
			}
			DisVar += pow(DistanceOfPointToLine[cnt], 2.0);
		}
		DisVar /= (2 * DistanceOfPointToLine.size() - 4);
		double DisStand = sqrt(DisVar);
		FittingPoints2.resize(0);
		if (((DisMax2>0.5) && (DisMax2 >(2 * DisStand))) || (DisMax2 >(2.5 * DisStand)))
			//if (DisMax2 >(2 * DisStand))
		{
			for (int i = 0; i != DistanceOfPointToLine.size(); ++i)
			{
				if (DistanceOfPointToLine[i] < DisMax2)
				{
					FittingPoints2.push_back(NewFittingPoints2.row(i));
				}
				else
				{
					int index = 0;
					for (int j = 0; j != ChoosenGroup.size(); ++j)
					{
						if (ChoosenGroup[j])
						{
							++index;
							if (index == (i + 1))
							{
								ChoosenGroup[j] = false;
								break;
							}
						}
					}
				}
			}
		}
		else
		{
			for (int i = 0; i != DistanceOfPointToLine.size(); ++i)
			{
				{
					FittingPoints2.push_back(NewFittingPoints2.row(i));
				}
			}
			break;
		}
	}
	axes = calcPCA(FittingPoints2);

	//轴线方向投影
	axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	normalize(axesMat, axesMat);
	studFeature[1] = axesMat.t();

	//FittingPoints.resize(0);
	//for (int i = 0; i != vecFittingPoints.size(); i++)
	//{
	//	if (ChoosenGroup[i])
	//	{
	//		FittingPoints.push_back(vecFittingPoints[i]);
	//	}
	//}

	//string strStudPointsPath = "studPoints.txt";
	//LaserSensorIO.WriteRowFirstAPP(strStudPointsPath, FittingPoints);

	Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
		dataAdjust.row(cnt) = FittingPoints.row(cnt) - planePoint;
	Mat projectedPoints = FittingPoints - dataAdjust * axesMat * axesMat.t();
	//旋转至与XY平面平行
	RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);

	//距离之和最小法，180912
	vector<PTS> points;
	//bool flag = false;
	double cen_z = rotInfo.XYRoted.at<double>(0, 2);

	for (int j = 0; j != rotInfo.XYRoted.rows; j++)
	{
		PTS pt(rotInfo.XYRoted.at<double>(j, 0), rotInfo.XYRoted.at<double>(j, 1));
		points.push_back(pt);
	}

	double cen_x = 0;
	double cen_y = -3;
	double cen_r = -1;
	Mat studCenter;
	CircleFitSolver *cfs = new CircleFitSolver();
	if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
	{
		Mat rot2Z_cen(1, 3, CV_64FC1, Scalar(-1));
		rot2Z_cen.at<double>(0, 0) = cen_x;
		rot2Z_cen.at<double>(0, 1) = cen_y;
		rot2Z_cen.at<double>(0, 2) = cen_z;

		studCenter = rotInfo.R.inv()*(rot2Z_cen.t());
	}

	if (studCenter.empty())
	{
		return studFeature;
	}
	else
	{
		studFeature[0] = studCenter.t();
		return studFeature;
	}
}

vector<Mat>  C3DLaserSensorFeatureExtraction::Reconstruction(Mat &FittingPoints, double &radius)
{
	//#define  MAX_POINT_NUM 20000
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
	Mat defaultStudAxes(1, 3, CV_64FC1, Scalar(-1));
	vector<Mat> studFeature(2);
	studFeature[0] = defaultStudCenter;
	studFeature[1] = defaultStudAxes;

	////剔除只有一半的圆弧段
	//int maxRows = 0;
	//for (int i = 0; i != vecFittingPoints.size(); i++)
	//{
	//	if (vecFittingPoints[i].rows > maxRows)
	//		maxRows = vecFittingPoints[i].rows;
	//}

	//Mat FittingPoints;
	//for (int i = 0; i != vecFittingPoints.size(); ++i)
	//{
	//	if (vecFittingPoints[i].rows >= maxRows * 2.0 / 3.0)
	//	{
	//		FittingPoints.push_back(vecFittingPoints[i]);
	//	}
	//}

	//if (FittingPoints.empty())
	//	return studFeature;

	int i = 0, j = 0;              //循环用变量
	int int_temp = FittingPoints.rows;           //用于存储待拟合的点数
	int loop_times = 0, pp = 0;    //loop_time等价于t，表示迭代循环次数
	double a = 1, b = 1, c = 1;
	double x0 = 0, y0 = 0, z0 = 0;
	double D = 0, s = 0, S = 0, dx = 0, dy = 0, dz = 0;
	double R = 0;                               //半径
	double d_temp1 = 0, d_temp2 = 0, d_temp3 = 0;
	//double B[MAX_POINT_NUM][7] = { 0 };
	//double L[MAX_POINT_NUM] = { 0 };
	//double worldVetex[MAX_POINT_NUM][3] = { 0 };  //用于存储点坐标
	Mat B(int_temp, 7, CV_64FC1, Scalar(-1));
	Mat L(int_temp, 1, CV_64FC1, Scalar(-1));

	double mean_x = 0, mean_y = 0, mean_z = 0;
	bool while_flag = 1;
	CvMat* C = cvCreateMat(2, 7, CV_64FC1);
	CvMat* W = cvCreateMat(2, 1, CV_64FC1);
	CvMat* N = cvCreateMat(9, 9, CV_64FC1);
	CvMat* N_inv = cvCreateMat(9, 9, CV_64FC1);
	CvMat* UU = cvCreateMat(9, 1, CV_64FC1);
	CvMat* para = cvCreateMat(9, 1, CV_64FC1);     //参数矩阵
	//变量初始化
	cvZero(C); cvZero(W); cvZero(N); cvZero(N_inv); cvZero(UU);
	cvSetIdentity(para);

	//求解重心、初始化
	for (i = 0; i<int_temp; i++)    //int_temp为点云总数
	{
		d_temp1 += FittingPoints.at<double>(i, 0);
		d_temp2 += FittingPoints.at<double>(i, 1);
		d_temp3 += FittingPoints.at<double>(i, 2);
	}
	mean_x = d_temp1 / int_temp; mean_y = d_temp2 / int_temp; mean_z = d_temp3 / int_temp;
	x0 = mean_x; y0 = mean_y; z0 = mean_z;
	R = 4.0;
	//迭代循环，最优化矩阵para
	while (while_flag == true)       //即(max(abs(para(1:7)))>0.00001)               
	{
		//1. a、b、c的符号修正
		if (a<0)
		{
			a = -a; b = -b; c = -c;
		}
		if (a == 0)
		{
			if (b<0)
			{
				b = -b; c = -c;
			}
			if (b == 0)
			{
				if (c<0)
					c = -c;
			}
		}
		s = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2)) + 0.0000001;       //防止a b c 同时为0
		a = a / s + 0.0000001; b = b / s + 0.0000001; c = c / s + 0.0000001;
		//2. 计算矩阵B和L
		for (i = 0; i<int_temp; i++)                              //int_temp为点云总数
		{
			//数据计算
			D = a*(FittingPoints.at<double>(i, 0) - x0) + b*(FittingPoints.at<double>(i, 1) - y0) + c*(FittingPoints.at<double>(i, 2) - z0);       //D=a*(X(i)-x0)+b*(Y(i)-y0)+c*(Z(i)-z0);
			dx = x0 + a*D - FittingPoints.at<double>(i, 0); dy = y0 + b*D - FittingPoints.at<double>(i, 1); dz = z0 + c*D - FittingPoints.at<double>(i, 2);//dx=x0+a*D-X(i);dy=y0+b*D-Y(i);dz=z0+c*D-Z(i);
			S = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));                                           //S=sqrt(dx^2+dy^2+dz^2);
			B.at<double>(i, 0) = (dx*(a*(FittingPoints.at<double>(i, 0) - x0) + D) + dy*b*(FittingPoints.at<double>(i, 0) - x0) + dz*c*(FittingPoints.at<double>(i, 0) - x0)) / S;   //b1
			B.at<double>(i, 1) = (dx*a*(FittingPoints.at<double>(i, 1) - y0) + dy*(b*(FittingPoints.at<double>(i, 1) - y0) + D) + dz*c*(FittingPoints.at<double>(i, 1) - y0)) / S;
			B.at<double>(i, 2) = (dx*a*(FittingPoints.at<double>(i, 2) - z0) + dy*b*(FittingPoints.at<double>(i, 2) - z0) + dz*(c*(FittingPoints.at<double>(i, 2) - z0) + D)) / S;
			B.at<double>(i, 3) = (dx*(1 - pow(a, 2)) - dy*a*b - dz*a*c) / S;
			B.at<double>(i, 4) = (-dx*a*b + dy*(1 - pow(b, 2)) - dz*b*c) / S;
			B.at<double>(i, 5) = (-dx*a*c - dy*b*c + dz*(1 - pow(c, 2))) / S;
			B.at<double>(i, 6) = -1;
			//数据存储

			//B=[B;b1 b2 b3 b4 b5 b6 b7]; 此步骤已整合至上10行的B[i][0-6]中
			L.at<double>(i, 0) = R - S;  //l=[R-S];
			//L=[L;l];
		}
		//3. 计算矩阵C(2*7)和W(2*1)
		d_temp1 = 1 - pow(a, 2) - pow(b, 2) - pow(c, 2);

		if (fabs(a) >= fabs(b) && fabs(a) >= fabs(c))
		{
			cvZero(C); cvZero(W);
			cvmSet(C, 0, 0, 2 * a); cvmSet(C, 0, 1, 2 * b);
			cvmSet(C, 0, 2, 2 * c); cvmSet(C, 1, 3, 1);
			cvmSet(W, 0, 0, d_temp1); cvmSet(W, 1, 0, mean_x - x0);
		}
		if (fabs(b) >= fabs(a) && fabs(b) >= fabs(c))
		{
			cvZero(C); cvZero(W);
			cvmSet(C, 0, 0, 2 * a); cvmSet(C, 0, 1, 2 * b);
			cvmSet(C, 0, 2, 2 * c); cvmSet(C, 1, 4, 1);
			cvmSet(W, 0, 0, d_temp1); cvmSet(W, 1, 0, mean_y - y0);
		}
		if (fabs(c) >= fabs(a) && fabs(c) >= fabs(b))
		{
			cvZero(C); cvZero(W);
			cvmSet(C, 0, 0, 2 * a); cvmSet(C, 0, 1, 2 * b);
			cvmSet(C, 0, 2, 2 * c); cvmSet(C, 1, 5, 1);
			cvmSet(W, 0, 0, d_temp1); cvmSet(W, 1, 0, mean_z - z0);
		}
		//4. 计算para矩阵
		//Nbb=B'*B;U=B'*L;
		//N=[Nbb C';C zeros(2)];
		//UU=[U;W];
		//para=inv(N)*UU;
		//4.1 计算矩阵N
		cvZero(N);        // N= |Nbb(7*7)  C'(7*2)|
		//    |C  (2*7)  O(2*2) |
		for (i = 0; i<7; i++)
		{
			for (j = 0; j<7; j++)
			{
				d_temp1 = 0;
				for (pp = 0; pp<int_temp; pp++)
				{
					d_temp1 += B.at<double>(pp, i) * B.at<double>(pp, j);
				}
				cvmSet(N, i, j, d_temp1);
			}
		}
		for (i = 0; i<2; i++)
			for (j = 0; j<7; j++)
				cvmSet(N, i + 7, j, cvmGet(C, i, j));
		for (i = 0; i<2; i++)
			for (j = 0; j<7; j++)
				cvmSet(N, j, i + 7, cvmGet(C, i, j));
		//4.2 计算矩阵UU
		for (i = 0; i<7; i++)
		{
			d_temp1 = 0;
			for (pp = 0; pp<int_temp; pp++)
			{
				d_temp1 += B.at<double>(pp, i) * L.at<double>(pp, 0);
			}
			cvmSet(UU, i, 0, d_temp1);
		}
		for (i = 0; i<2; i++)
			cvmSet(UU, i + 7, 0, cvmGet(W, i, 0));
		//4.2 计算矩阵para
		cvInvert(N, N_inv);           //para=inv(N)*UU;
		cvmMul(N_inv, UU, para);
		a = a + cvmGet(para, 0, 0);      //a=a+para(1);b=b+para(2);c=c+para(3);
		b = b + cvmGet(para, 1, 0);
		c = c + cvmGet(para, 2, 0);
		x0 = x0 + cvmGet(para, 3, 0);   //x0=x0+para(4);y0=y0+para(5);z0=z0+para(6);
		y0 = y0 + cvmGet(para, 4, 0);
		z0 = z0 + cvmGet(para, 5, 0);
		R = R + cvmGet(para, 6, 0);
		loop_times = loop_times + 1;     //t=t+1
		//5. 计算while标志while_flag为0或者1：若max(abs(para(1:7)))>0.00001，则while_flag=1；否则，为0.
		d_temp1 = cvmGet(para, 0, 0);
		for (i = 1; i<7; i++)
		{
			if (fabs(d_temp1)<fabs(cvmGet(para, i, 0)))
				d_temp1 = cvmGet(para, i, 0);
		}
		if (fabs(d_temp1)>0.00001)
			while_flag = 1;
		else
			while_flag = 0;
	}
	cout << "迭代循环次数为:" << loop_times << endl;
	cout << "拟合半径为:" << R << endl;
	cout << "圆柱轴线方向向量为:[" << a << ", " << b << ", " << c << "]" << endl;
	cout << "圆柱轴线起始点坐标为:[" << x0 << ", " << y0 << ", " << z0 << "]" << endl;
	studFeature[0].at<double>(0, 0) = x0;
	studFeature[0].at<double>(0, 1) = y0;
	studFeature[0].at<double>(0, 2) = z0;
	studFeature[1].at<double>(0, 0) = a;
	studFeature[1].at<double>(0, 1) = b;
	studFeature[1].at<double>(0, 2) = c;
	radius = R;
	//studAxes.at<double>(0, 0) = a;
	//studAxes.at<double>(0, 1) = b;
	//studAxes.at<double>(0, 2) = c;
	return studFeature;
}

vector<Mat>  C3DLaserSensorFeatureExtraction::StudFit14(vector<Mat> &vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold/* =0.2*/, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
	Mat defaultStudAxes(1, 3, CV_64FC1, Scalar(-1));
	vector<Mat> studFeature(2);
	studFeature[0] = defaultStudCenter;
	studFeature[1] = defaultStudAxes;

	//剔除只有一半的圆弧段
	int maxRows = 0;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows > maxRows)
			maxRows = vecFittingPoints[i].rows;
	}

	Mat FittingPoints;
	Mat FittingPoints3;
	vector<bool> ChoosenGroup;
	for (int i = 0; i != vecFittingPoints.size(); ++i)
	{
		if (vecFittingPoints[i].rows >= maxRows * 2.0 / 3.0)
		{
			FittingPoints.push_back(vecFittingPoints[i]);
		}
		FittingPoints3.push_back(vecFittingPoints[i]);
		ChoosenGroup.push_back(true);
	}

	if (FittingPoints.empty())
		return studFeature;

	vector<Mat> TempStudFeature(2);
	double radius = 0;
	TempStudFeature = Reconstruction(FittingPoints, radius);
	Mat axesMat = TempStudFeature[1];
	normalize(axesMat, axesMat);
	studFeature[1] = axesMat;

	//Point3d axes = calcPCA(FittingPoints);
	//Mat axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	//normalize(axesMat, axesMat);

	//Mat vecXYRoted;
	//Mat vecProjectedPoints;
	//Mat FittingPoints2;
	//for (int i = 0; i != vecFittingPoints.size(); i++)
	//{
	//	//if (vecFittingPoints[i].rows < maxRows * 1.0 / 2.0)
	//	//{
	//	//	continue;
	//	//}
	//	Mat dataAdjust2(vecFittingPoints[i].rows, vecFittingPoints[i].cols, CV_64FC1);
	//	Mat meanPoint(1, vecFittingPoints[i].cols, CV_64FC1, cvScalar(0));
	//	//求取平均值
	//	for (int j = 0; j != vecFittingPoints[i].cols; j++)
	//	{
	//		meanPoint.at<double>(0, j) = cv::mean(vecFittingPoints[i].col(j)).val[0];
	//	}
	//	for (int j = 0; j != dataAdjust2.rows; j++)
	//	{
	//		dataAdjust2.row(j) = vecFittingPoints[i].row(j) - meanPoint;
	//	}
	//	vecProjectedPoints = vecFittingPoints[i] - dataAdjust2 * axesMat * axesMat.t();
	//	RotToZInfo rotInfo2 = RotToZ(vecProjectedPoints, axesMat);
	//	vecXYRoted = rotInfo2.XYRoted;
	//	vector<PTS> points;
	//	double cen_z = vecXYRoted.at<double>(0, 2);
	//	for (int j = 0; j != vecXYRoted.rows; j++)
	//	{
	//		PTS pt(vecXYRoted.at<double>(j, 0), vecXYRoted.at<double>(j, 1));
	//		points.push_back(pt);
	//	}
	//	double cen_x = 0;
	//	double cen_y = -3;
	//	double cen_r = -1;
	//	CircleFitSolver *cfs = new CircleFitSolver();
	//	if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
	//	{
	//		{
	//			Mat rot2Z_cen(1, 3, CV_64FC1, Scalar(-1));
	//			rot2Z_cen.at<double>(0, 0) = cen_x;
	//			rot2Z_cen.at<double>(0, 1) = cen_y;
	//			rot2Z_cen.at<double>(0, 2) = cen_z;
	//			Mat temp = (rotInfo2.R.inv()*(rot2Z_cen.t())).t();
	//			FittingPoints2.push_back(temp);
	//		}
	//	}
	//}

	//bool cycleFlag = true;
	//double mindif = 10;
	//double radius = 0;
	//vector<Mat> TempStudFeature(2);
	//TempStudFeature = Reconstruction(FittingPoints3, radius);
	//Mat axesresult = TempStudFeature[1];
	//Mat axesMat = axesresult;
	//while (cycleFlag)
	//{
	//	double maxdifRadius = 0;
	//	int index = 0;
	//	for (int i = 0; i != vecFittingPoints.size(); i++)
	//	{
	//		if (ChoosenGroup[i])
	//		{
	//			Mat dataAdjust2(vecFittingPoints[i].rows, vecFittingPoints[i].cols, CV_64FC1);
	//			for (int j = 0; j != dataAdjust2.rows; j++)
	//			{
	//				dataAdjust2.row(j) = vecFittingPoints[i].row(j) - TempStudFeature[0];
	//			}
	//			Mat vecProjectedPoints = vecFittingPoints[i] - dataAdjust2 * axesMat.t() * axesMat;
	//			double difRadius = 0;
	//			for (int j = 0; j != vecProjectedPoints.rows; j++)
	//			{
	//				difRadius += sqrt(pow(vecProjectedPoints.at<double>(0, 0) - TempStudFeature[0].at<double>(0, 0), 2.0)
	//								+ pow(vecProjectedPoints.at<double>(0, 1) - TempStudFeature[0].at<double>(0, 1), 2.0)
	//								+ pow(vecProjectedPoints.at<double>(0, 2) - TempStudFeature[0].at<double>(0, 2), 2.0)) - radius;
	//			}
	//			difRadius /= vecProjectedPoints.rows;
	//			if (difRadius > maxdifRadius)
	//			{
	//				maxdifRadius = difRadius;
	//				index = i;
	//			}
	//		}
	//	}
	//	ChoosenGroup[index] = false;
	//	FittingPoints3.resize(0);
	//	for (int i = 0; i != vecFittingPoints.size(); i++)
	//	{
	//		if (ChoosenGroup[i])
	//		{
	//			FittingPoints3.push_back(vecFittingPoints[i]);
	//		}
	//	}
	//	if (FittingPoints3.rows < 1000)
	//	{
	//		break;
	//	}
	//	TempStudFeature = Reconstruction(FittingPoints3, radius);
	//	if (radius < 3.7)
	//	{
	//		break;
	//	}
	//	Mat axesnew = TempStudFeature[1];
	//	double dif = sqrt(pow((abs(axesnew.at<double>(0, 0)) - abs(axesMat.at<double>(0, 0))), 2.0) + pow((abs(axesnew.at<double>(0, 1)) - abs(axesMat.at<double>(0, 1))), 2.0) + pow((abs(axesnew.at<double>(0, 2)) - abs(axesMat.at<double>(0, 2))), 2.0));
	//	double threshold = sqrt(pow(0.005, 2.0) * 3);
	//	if (dif < threshold)
	//	{
	//		cycleFlag = false;
	//		axesresult = axesMat;
	//	}
	//	else
	//	{
	//		if (dif < mindif)
	//		{
	//			mindif = dif;
	//			axesresult = axesMat;
	//		}
	//	}
	//	axesMat = axesnew;
	//}
	//axesMat = axesresult;

	//////轴线方向投影
	////axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	//normalize(axesMat, axesMat);
	//studFeature[1] = axesMat;

	Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
		dataAdjust.row(cnt) = FittingPoints.row(cnt) - planePoint;
	Mat projectedPoints = FittingPoints - dataAdjust * axesMat.t() * axesMat;
	//旋转至与XY平面平行
	RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat.t());

	Mat rot2Z_cen = HoughCircle2(rotInfo.XYRoted, 4.8, 5.0, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
	Mat studCenter = rotInfo.R.inv()*(rot2Z_cen.t());

	////距离之和最小法，180912
	//vector<PTS> points;
	////bool flag = false;
	//double cen_z = rotInfo.XYRoted.at<double>(0, 2);

	//for (int j = 0; j != rotInfo.XYRoted.rows; j++)
	//{
	//	PTS pt(rotInfo.XYRoted.at<double>(j, 0), rotInfo.XYRoted.at<double>(j, 1));
	//	points.push_back(pt);
	//}

	//double cen_x = 0;
	//double cen_y = -3;
	//double cen_r = -1;
	//Mat studCenter;
	//CircleFitSolver *cfs = new CircleFitSolver();
	//if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
	//{
	//	Mat rot2Z_cen(1, 3, CV_64FC1, Scalar(-1));
	//	rot2Z_cen.at<double>(0, 0) = cen_x;
	//	rot2Z_cen.at<double>(0, 1) = cen_y;
	//	rot2Z_cen.at<double>(0, 2) = cen_z;

	//	studCenter = rotInfo.R.inv()*(rot2Z_cen.t());
	//}

	if (studCenter.empty())
	{
		return studFeature;
	}
	else
	{
		studFeature[0] = studCenter.t();
		return studFeature;
	}

}

void C3DLaserSensorFeatureExtraction::OpitmizePoints(Mat &FittingPoints2, vector<bool> &ChoosenGroup, vector<int> similarVec)
{
	Mat TempFittingPoints2;
	for (int i = 0; i != FittingPoints2.rows; ++i)
	{
		if (similarVec[i] != i)
		{
			Mat cen = (FittingPoints2.row(i) + FittingPoints2.row(similarVec[i])) / 2.0;
			TempFittingPoints2.push_back(cen);
			//vecFittingPoints[i].push_back(vecFittingPoints[similarVec[i]]);
			ChoosenGroup[similarVec[i]] = false;
			++i;
		}
		else
		{
			TempFittingPoints2.push_back(FittingPoints2.row(i));
		}
	}
	FittingPoints2 = TempFittingPoints2;

	bool cycleFlag = true;
	Point3d axes2;
	Mat NewFittingPoints2;
	NewFittingPoints2.resize(0);
	while ((FittingPoints2.rows != NewFittingPoints2.rows) && (FittingPoints2.rows > 2))
		//while (cycleFlag)
	{
		axes2 = calcPCA(FittingPoints2);
		cycleFlag = false;
		NewFittingPoints2 = FittingPoints2;
		Point3d Point1, Point2;
		Point1.x = cv::mean(NewFittingPoints2.col(0)).val[0];
		Point1.y = cv::mean(NewFittingPoints2.col(1)).val[0];
		Point1.z = cv::mean(NewFittingPoints2.col(2)).val[0];
		Point2.x = Point1.x + 1;
		Point2.y = axes2.y / axes2.x + Point1.y;
		Point2.z = axes2.z / axes2.x + Point1.z;
		vector<double> DistanceOfPointToLine;
		double ab = sqrt(pow((Point2.x - Point1.x), 2.0) + pow((Point2.y - Point1.y), 2.0) + pow((Point2.z - Point1.z), 2.0));
		double DisVar = 0;
		//double DisMax = 0;
		double DisMax2 = 0;
		for (int cnt = 0; cnt != NewFittingPoints2.rows; cnt++)
		{
			double as = sqrt(pow((Point1.x - NewFittingPoints2.at<double>(cnt, 0)), 2.0) + pow((Point1.y - NewFittingPoints2.at<double>(cnt, 1)), 2.0) + pow((Point1.z - NewFittingPoints2.at<double>(cnt, 2)), 2.0));
			double bs = sqrt(pow((NewFittingPoints2.at<double>(cnt, 0) - Point2.x), 2.0) + pow((NewFittingPoints2.at<double>(cnt, 1) - Point2.y), 2.0) + pow((NewFittingPoints2.at<double>(cnt, 2) - Point2.z), 2.0));
			double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
			double sin_A = sqrt(1 - pow(cos_A, 2.0));
			DistanceOfPointToLine.push_back(as * sin_A);
			//if ((similarVec[cnt] != cnt) && (DisMax < as * sin_A))
			//{
			//	DisMax = as * sin_A;
			//	MaxIndex = cnt;
			//}
			if (DisMax2 < as * sin_A)
			{
				DisMax2 = as * sin_A;
			}
			DisVar += pow(DistanceOfPointToLine[cnt], 2.0);
		}
		DisVar /= (2 * DistanceOfPointToLine.size() - 4);
		double DisStand = sqrt(DisVar);
		FittingPoints2.resize(0);
		if (((DisMax2>0.5) && (DisMax2 >(2 * DisStand))) || (DisMax2 >(2.5 * DisStand)))
			//if (DisMax2 >(2 * DisStand))
		{
			for (int i = 0; i != DistanceOfPointToLine.size(); ++i)
			{
				if (DistanceOfPointToLine[i] < DisMax2)
				{
					FittingPoints2.push_back(NewFittingPoints2.row(i));
				}
				else
				{
					//if (DisMax2 == DisMax)
					//{
					//	int index = similarVec[i];
					//	similarVec[index] = index;
					//	similarVec[i] = i;
					//}
					//int tempSize = similarVec.size() - 1;
					//for (int j = i; j < tempSize; ++j)
					//{
					//	similarVec[j] = similarVec[j + 1] - 1;
					//}
					//similarVec.resize(tempSize);
					int index = 0;
					for (int j = 0; j != ChoosenGroup.size(); ++j)
					{
						if (ChoosenGroup[j])
						{
							++index;
							if (index == (i + 1))
							{
								ChoosenGroup[j] = false;
								break;
							}
						}
					}
				}
			}
		}
		//else if ((DisMax == DisMax2) && (DisMax - DistanceOfPointToLine[similarVec[MaxIndex]] > 0.1))
		//{
		//	for (int i = 0; i != DistanceOfPointToLine.size(); ++i)
		//	{
		//		//if (DistanceOfPointToLine[i] < (2 * DisStand))
		//		if (DistanceOfPointToLine[i] != DisMax)
		//		{
		//			FittingPoints2.push_back(NewFittingPoints2.row(i));
		//		}
		//		else
		//		{
		//			int index = similarVec[i];
		//			similarVec[index] = index;
		//			similarVec[i] = i;
		//			int tempSize = similarVec.size() - 1;
		//			for (int j = i; j < tempSize; ++j)
		//			{
		//				similarVec[j] = similarVec[j + 1] - 1;
		//			}
		//			similarVec.resize(tempSize);
		//			index = 0;
		//			for (int j = 0; j != ChoosenGroup.size(); ++j)
		//			{
		//				if (ChoosenGroup[j])
		//				{
		//					++index;
		//					if (index == (i + 1))
		//					{
		//						ChoosenGroup[j] = false;
		//						break;
		//					}
		//				}
		//			}
		//		}
		//	}
		//}
		else
		{
			for (int i = 0; i != DistanceOfPointToLine.size(); ++i)
			{
				//if (similarVec[i] != i)
				//{
				//	Mat cen = (NewFittingPoints2.row(i) + NewFittingPoints2.row(similarVec[i])) / 2.0;
				//	FittingPoints2.push_back(cen);
				//	++i;
				//}
				//else
				{
					FittingPoints2.push_back(NewFittingPoints2.row(i));
				}
			}
			break;
		}
		//for (int i = 0; i != similarVec.size(); ++i)
		//{
		//	if (similarVec[i] != i)
		//	{
		//		cycleFlag = true;
		//		break;
		//	}
		//}
	}
}

vector<Mat>  C3DLaserSensorFeatureExtraction::StudFit2(vector<Mat> &vecFittingPoints, Mat planePoint, Mat planeVector, vector<int> similarVec, double CircleMeanErrorThreshold/* =0.2*/, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)  //柱面返回中心坐标值+法线
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
	Mat defaultStudAxes(1, 3, CV_64FC1, Scalar(-1));
	vector<Mat> studFeature(2);
	studFeature[0] = defaultStudCenter;
	studFeature[1] = defaultStudAxes;

	//剔除只有一半的圆弧段
	int maxRows = 0;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows > maxRows)
			maxRows = vecFittingPoints[i].rows;
	}

	Mat FittingPoints;
	vector<bool> ChoosenGroup;
	for (int i = 0; i != vecFittingPoints.size(); ++i)
	{
		if (vecFittingPoints[i].rows >= maxRows * 2.0 / 3.0)
		{
			FittingPoints.push_back(vecFittingPoints[i]);
			ChoosenGroup.push_back(true);
		}
		else
		{
			ChoosenGroup.push_back(false);
			if (similarVec[i] != i)
			{
				int index = similarVec[i];
				similarVec[index] = index;
				similarVec[i] = i;
			}
		}
	}

	if (FittingPoints.empty())
		return studFeature;

	//if (FittingPoints2.empty())
	//	return studFeature;

	Point3d axes = calcPCA(FittingPoints);
	//Point3d axes1 = axes;
	//axes.x = axes2.x;
	//axes.y = axes1.y;
	//axes.z = axes1.z;	

	//轴线方向投影
	Mat axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	normalize(axesMat, axesMat);

	//string strStudPointsPath = "studPoints.txt";
	//LaserSensorIO.WriteRowFirstAPP(strStudPointsPath, FittingPoints);

	//Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
	//for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
	//	dataAdjust.row(cnt) = FittingPoints.row(cnt) - planePoint;
	//Mat projectedPoints = FittingPoints - dataAdjust * axesMat * axesMat.t();

	//string strResultPath111 = "pro.txt";
	//LaserSensorIO.WriteRowFirstAPP(strResultPath111, projectedPoints);
	//旋转至与XY平面平行
	//RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);

	Mat FittingPoints2;
	for (int i = 0; i != vecFittingPoints.size(); ++i)
	{
		if (vecFittingPoints[i].rows >= maxRows * 2.0 / 3.0)
		{
			int num_closest = vecFittingPoints[i].rows / 2;
			int num_dismin = 0;
			double closest = abs(vecFittingPoints[i].at<double>(0, 2) - vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2));
			double dis_min = vecFittingPoints[i].at<double>(0, 2);
			if (vecFittingPoints[i].at<double>(0, 2) > vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2))
			{
				int num_pt = 0;
				for (int j = vecFittingPoints[i].rows - 1; j != 0; --j)
				{
					if (dis_min > vecFittingPoints[i].at<double>(j, 2))
					{
						dis_min = vecFittingPoints[i].at<double>(j, 2);
						num_dismin = j;
					}
				}
				for (int j = 1; j != num_dismin; ++j)
				{
					if (closest > abs(vecFittingPoints[i].at<double>(j, 2) - vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2)))
					{
						closest = abs(vecFittingPoints[i].at<double>(j, 2) - vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2));
						num_pt = j;
					}
				}
				num_closest = (num_pt + vecFittingPoints[i].rows - 1) / 2;
			}
			else
				if (vecFittingPoints[i].at<double>(0, 2) < vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2))
				{
					int num_pt = vecFittingPoints[i].rows - 1;
					for (int j = 1; j != vecFittingPoints[i].rows; ++j)
					{
						if (dis_min > vecFittingPoints[i].at<double>(j, 2))
						{
							dis_min = vecFittingPoints[i].at<double>(j, 2);
							num_dismin = j;
						}
					}
					for (int j = vecFittingPoints[i].rows - 1; j != num_dismin; --j)
					{
						if (closest > abs(vecFittingPoints[i].at<double>(j, 2) - vecFittingPoints[i].at<double>(0, 2)))
						{
							closest = abs(vecFittingPoints[i].at<double>(j, 2) - vecFittingPoints[i].at<double>(0, 2));
							num_pt = j;
						}
					}
					num_closest = num_pt / 2;
				}
			FittingPoints2.push_back(vecFittingPoints[i].row(num_closest));
		}
		//else
		//{
		//	int tempSize = similarVec.size() - 1;
		//	for (int j = i; j < tempSize; ++j)
		//	{
		//		similarVec[j] = similarVec[j + 1] - 1;
		//	}
		//	similarVec.resize(tempSize);
		//}
	}

	Mat vecXYRoted;
	Mat vecProjectedPoints;
	Mat FittingPoints3;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows < maxRows * 2.0 / 3.0)
		{
			//int tempSize = similarVec.size() - 1;
			//for (int j = i; j < tempSize; ++j)
			//{
			//	similarVec[j] = similarVec[j + 1] - 1;
			//}
			//similarVec.resize(tempSize);
			continue;
		}
		Mat dataAdjust2(vecFittingPoints[i].rows, vecFittingPoints[i].cols, CV_64FC1);
		Mat meanPoint(1, vecFittingPoints[i].cols, CV_64FC1, cvScalar(0));
		//求取平均值
		for (int j = 0; j != vecFittingPoints[i].cols; j++)
		{
			meanPoint.at<double>(0, j) = cv::mean(vecFittingPoints[i].col(j)).val[0];
		}
		for (int j = 0; j != dataAdjust2.rows; j++)
		{
			dataAdjust2.row(j) = vecFittingPoints[i].row(j) - meanPoint;
		}
		vecProjectedPoints = vecFittingPoints[i] - dataAdjust2 * axesMat * axesMat.t();
		RotToZInfo rotInfo2 = RotToZ(vecProjectedPoints, axesMat);
		vecXYRoted = rotInfo2.XYRoted;
		vector<PTS> points;
		double cen_z = vecXYRoted.at<double>(0, 2);
		for (int j = 0; j != vecXYRoted.rows; j++)
		{
			PTS pt(vecXYRoted.at<double>(j, 0), vecXYRoted.at<double>(j, 1));
			points.push_back(pt);
		}
		double cen_x = 0;
		double cen_y = -3;
		double cen_r = -1;
		CircleFitSolver *cfs = new CircleFitSolver();
		if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
		{
			//if ((cen_r > 3.65) && (cen_r < 4.5))
			{
				Mat rot2Z_cen(1, 3, CV_64FC1, Scalar(-1));
				rot2Z_cen.at<double>(0, 0) = cen_x;
				rot2Z_cen.at<double>(0, 1) = cen_y;
				rot2Z_cen.at<double>(0, 2) = cen_z;
				Mat temp = (rotInfo2.R.inv()*(rot2Z_cen.t())).t();
				FittingPoints3.push_back(temp);
			}
			//else
			//{
			//	if (similarVec[i] != i)
			//	{
			//		int index = similarVec[i];
			//		similarVec[index] = index;
			//		similarVec[i] = i;
			//	}
			//	ChoosenGroup[i] = false;
			//}
		}
	}

	//for (int i = 0; i != similarVec.size(); ++i)
	//{
	//	if (similarVec[i] != i)
	//	{
	//		vecFittingPoints[i].push_back(vecFittingPoints[similarVec[i]]);
	//		++i;
	//	}
	//}
	
	int indexj = 0;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (ChoosenGroup[i])
		{
			++indexj;
		}
		else
		{
			int tempSize = similarVec.size() - 1;
			for (int j = indexj; j < tempSize; ++j)
			{
				similarVec[j] = similarVec[j + 1] - 1;
			}
			similarVec.resize(tempSize);
		}
	}

	vector<bool> ChoosenGroup2 = ChoosenGroup;
	vector<bool> ChoosenGroup3 = ChoosenGroup;
	OpitmizePoints(FittingPoints2, ChoosenGroup2, similarVec);
	OpitmizePoints(FittingPoints3, ChoosenGroup3, similarVec);
	Point3d axes2 = calcPCA(FittingPoints2);
	Point3d axes3 = calcPCA(FittingPoints3);
	Mat axesMat2 = Mat(axes2);
	Mat axesMat3 = Mat(axes3);
	normalize(axesMat2, axesMat2);
	normalize(axesMat3, axesMat3);
	if (abs(axesMat2.dot(planeVector)) > abs(axesMat3.dot(planeVector)))
	{
		studFeature[1] = axesMat2.t();
		//axesMat = axesMat2;
	}
	else
	{
		studFeature[1] = axesMat3.t();
		//axesMat = axesMat3;
	}
	//axes = calcPCA(FittingPoints2);

	//轴线方向投影
	//axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	axesMat = planeVector;
	//axesMat = studFeature[1].t();
	normalize(axesMat, axesMat);

	string strStudAxesPath2 = "StudClosestAxes.txt";
	string strStudAxesPath3 = "StudCentreAxes.txt";
	LaserSensorIO.WriteRowFirstAPP(strStudAxesPath2, axesMat2.t());
	LaserSensorIO.WriteRowFirstAPP(strStudAxesPath3, axesMat3.t());

	//FittingPoints.resize(0);
	//for (int i = 0; i != vecFittingPoints.size(); i++)
	//{
	//	if (ChoosenGroup3[i])
	//	{
	//		FittingPoints.push_back(vecFittingPoints[i]);
	//	}
	//}

	//string strStudPointsPath = "studPoints.txt";
	//LaserSensorIO.WriteRowFirstAPP(strStudPointsPath, FittingPoints);

	Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
		dataAdjust.row(cnt) = FittingPoints.row(cnt) - planePoint;
	Mat projectedPoints = FittingPoints - dataAdjust * axesMat * axesMat.t();
	//旋转至与XY平面平行
	RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);

	//Mat rot2Z_cen = HoughCircle2(rotInfo.XYRoted, 4.8, 5.0, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
	//Mat studCenter = rotInfo.R.inv()*(rot2Z_cen.t());

	//距离之和最小法，180912
	vector<PTS> points;
	//bool flag = false;
	double cen_z = rotInfo.XYRoted.at<double>(0, 2);

	for (int j = 0; j != rotInfo.XYRoted.rows; j++)
	{
		PTS pt(rotInfo.XYRoted.at<double>(j, 0), rotInfo.XYRoted.at<double>(j, 1));
		points.push_back(pt);
	}

	double cen_x = 0;
	double cen_y = -3;
	double cen_r = -1;
	Mat studCenter;
	CircleFitSolver *cfs = new CircleFitSolver();
	if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
	{
		Mat rot2Z_cen(1, 3, CV_64FC1, Scalar(-1));
		rot2Z_cen.at<double>(0, 0) = cen_x;
		rot2Z_cen.at<double>(0, 1) = cen_y;
		rot2Z_cen.at<double>(0, 2) = cen_z;

		studCenter = rotInfo.R.inv()*(rot2Z_cen.t());
	}

	if (studCenter.empty())
	{
		return studFeature;
	}
	else
	{
		studFeature[0] = studCenter.t();
		return studFeature;
	}
}

vector<Mat>  C3DLaserSensorFeatureExtraction::StudFit2(int num, vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold/* =0.2*/, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)  //柱面返回中心坐标值+法线
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
	Mat defaultStudAxes(1, 3, CV_64FC1, Scalar(-1));
	vector<Mat> studFeature(2);
	studFeature[0] = defaultStudCenter;
	studFeature[1] = defaultStudAxes;

	//剔除只有一半的圆弧段
	int maxRows = 0;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows > maxRows)
			maxRows = vecFittingPoints[i].rows;
	}

	Mat FittingPoints;
	Mat FittingPoints2;
	Mat FittingPoints3;
	for (int i = 0; i != vecFittingPoints.size(); ++i)
	{
		if (vecFittingPoints[i].rows >= maxRows * 2.0 / 3.0)
		{
			FittingPoints.push_back(vecFittingPoints[i]);
			int num_closest = vecFittingPoints[i].rows / 2;
			int num_dismin = 0;
			double closest = abs(vecFittingPoints[i].at<double>(0, 2) - vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2));
			double dis_min = vecFittingPoints[i].at<double>(0, 2);
			if (vecFittingPoints[i].at<double>(0, 2) > vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2))
			{		
				int num_pt = 0;
				for (int j = vecFittingPoints[i].rows - 1; j != 0; --j)
				{
					if (dis_min > vecFittingPoints[i].at<double>(j, 2))
					{
						dis_min = vecFittingPoints[i].at<double>(j, 2);
						num_dismin = j;
					}
				}
				for (int j = 1; j != num_dismin; ++j)
				{
					if (closest > abs(vecFittingPoints[i].at<double>(j, 2) - vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2)))
					{
						closest = abs(vecFittingPoints[i].at<double>(j, 2) - vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2));
						num_pt = j;
					}
				}
				num_closest = (num_pt + vecFittingPoints[i].rows - 1) / 2;
			} 
			else
				if (vecFittingPoints[i].at<double>(0, 2) < vecFittingPoints[i].at<double>(vecFittingPoints[i].rows - 1, 2))
				{
					int num_pt = vecFittingPoints[i].rows - 1;
					for (int j = 1; j != vecFittingPoints[i].rows; ++j)
					{
						if (dis_min > vecFittingPoints[i].at<double>(j, 2))
						{
							dis_min = vecFittingPoints[i].at<double>(j, 2);
							num_dismin = j;
						}
					}
					for (int j = vecFittingPoints[i].rows - 1; j != num_dismin; --j)
					{
						if (closest > abs(vecFittingPoints[i].at<double>(j, 2) - vecFittingPoints[i].at<double>(0, 2)))
						{
							closest = abs(vecFittingPoints[i].at<double>(j, 2) - vecFittingPoints[i].at<double>(0, 2));
							num_pt = j;
						}
					}
					num_closest = num_pt / 2;
				}
			FittingPoints2.push_back(vecFittingPoints[i].row(num_closest));
			if ((num_closest - 25) < 0)
			{
				for (int j = 0; j != (2 * num_closest + 1); ++j)
				{
					FittingPoints3.push_back(vecFittingPoints[i].row(j));
				}
			}
			else
				if ((num_closest + 25) > (vecFittingPoints[i].rows - 1))
				{
					for (int j = vecFittingPoints[i].rows - 1; j != (2 * num_closest - vecFittingPoints[i].rows); --j)
					{
						FittingPoints3.push_back(vecFittingPoints[i].row(j));
					}
				}
				else
				{
					for (int j = num_closest - 25; j != (num_closest + 26); ++j)
					{
						FittingPoints3.push_back(vecFittingPoints[i].row(j));
					}
				}
		}
	}

	//double meanX = 0;
	//for (int i = 0; i != FittingPoints2.rows; ++i)
	//{
	//	meanX += FittingPoints2.at<double>(i, 0);
	//}
	//meanX /= FittingPoints2.rows;
	//double maxDis = 0;
	//Mat FittingPoints4;
	//for (int i = 0; i != FittingPoints2.rows; ++i)
	//{
	//	if (abs(FittingPoints2.at<double>(i, 0) - meanX) < 0.5)
	//	{
	//		FittingPoints4.push_back(FittingPoints2.row(i));
	//	}
	//}

	if (FittingPoints.empty())
		return studFeature;

	//if (FittingPoints2.empty())
	//	return studFeature;

	//Point3d axes1 = calcPCA(FittingPoints3);
	Point3d axes2;
	//Point3d axes=axes1;
	//axes.x = axes2.x;
	//axes.y = axes1.y;
	//axes.z = axes1.z;	
	
	Mat NewFittingPoints2;
	NewFittingPoints2.resize(0);
	while ((FittingPoints2.rows != NewFittingPoints2.rows) && (FittingPoints2.rows > 2))
	{
		axes2 = calcPCA(FittingPoints2);
		NewFittingPoints2 = FittingPoints2;
		Point3d Point1, Point2;
		Point1.x = cv::mean(NewFittingPoints2.col(0)).val[0];
		Point1.y = cv::mean(NewFittingPoints2.col(1)).val[0];
		Point1.z = cv::mean(NewFittingPoints2.col(2)).val[0];
		Point2.x = Point1.x + 1;
		Point2.y = axes2.y / axes2.x + Point1.y;
		Point2.z = axes2.z / axes2.x + Point1.z;
		vector<double> DistanceOfPointToLine;
		double ab = sqrt(pow((Point2.x - Point1.x), 2.0) + pow((Point2.y - Point1.y), 2.0) + pow((Point2.z - Point1.z), 2.0));
		double DisVar = 0;
		for (int cnt = 0; cnt != NewFittingPoints2.rows; cnt++)
		{
			double as = sqrt(pow((Point1.x - NewFittingPoints2.at<double>(cnt, 0)), 2.0) + pow((Point1.y - NewFittingPoints2.at<double>(cnt, 1)), 2.0) + pow((Point1.z - NewFittingPoints2.at<double>(cnt, 2)), 2.0));
			double bs = sqrt(pow((NewFittingPoints2.at<double>(cnt, 0) - Point2.x), 2.0) + pow((NewFittingPoints2.at<double>(cnt, 1) - Point2.y), 2.0) + pow((NewFittingPoints2.at<double>(cnt, 2) - Point2.z), 2.0));
			double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
			double sin_A = sqrt(1 - pow(cos_A, 2.0));
			DistanceOfPointToLine.push_back(as * sin_A);
			DisVar += pow(DistanceOfPointToLine[cnt], 2.0);
		}
		DisVar /= (2*DistanceOfPointToLine.size() - 4);
		double DisStand = sqrt(DisVar);
		FittingPoints2.resize(0);
		for (int i = 0; i != DistanceOfPointToLine.size(); ++i)
		{
			if (DistanceOfPointToLine[i] < (2.5 * DisStand))
			{
				FittingPoints2.push_back(NewFittingPoints2.row(i));
			}
		}
	}
	Point3d axes = calcPCA(FittingPoints2);

	//轴线方向投影
	Mat axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	normalize(axesMat, axesMat);
	studFeature[1] = axesMat.t();

	//string strStudPointsPath = "studPoints.txt";
	//LaserSensorIO.WriteRowFirstAPP(strStudPointsPath, FittingPoints);

	Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
		dataAdjust.row(cnt) = FittingPoints.row(cnt) - planePoint;
	Mat projectedPoints = FittingPoints - dataAdjust * axesMat * axesMat.t();

	//string strResultPath111 = "pro.txt";
	//LaserSensorIO.WriteRowFirstAPP(strResultPath111, projectedPoints);
	//旋转至与XY平面平行
	RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);

	//vector<Mat> vecXYRoted(vecFittingPoints.size());
	//vector<Mat> vecProjectedPoints(vecFittingPoints.size());
	//Mat studCenter;
	//double disMin = 2.5;
	//for (int i = 0; i != vecFittingPoints.size(); i++)
	//{
	//	if (vecFittingPoints[i].rows < maxRows * 2 / 3)
	//		continue;
	//	Mat dataAdjust2(vecFittingPoints[i].rows, vecFittingPoints[i].cols, CV_64FC1);
	//	for (int j = 0; j != dataAdjust2.rows; j++)
	//	{
	//		dataAdjust2.row(j) = vecFittingPoints[i].row(j) - planePoint;
	//	}
	//	vecProjectedPoints[i] = vecFittingPoints[i] - dataAdjust2 * axesMat * axesMat.t();
	//	RotToZInfo rotInfo2 = RotToZ(vecProjectedPoints[i], axesMat);
	//	vecXYRoted[i] = rotInfo2.XYRoted;
	//	vector<PTS> points;
	//	double cen_z = vecXYRoted[i].at<double>(0, 2);
	//	for (int j = 0; j != vecXYRoted[i].rows; j++)
	//	{
	//		PTS pt(vecXYRoted[i].at<double>(j, 0), vecXYRoted[i].at<double>(j, 1));
	//		points.push_back(pt);
	//	}
	//	double cen_x = 0;
	//	double cen_y = -3;
	//	double cen_r = -1;
	//	CircleFitSolver *cfs = new CircleFitSolver();
	//	if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
	//	{
	//		if (abs(cen_r - 2.5) < disMin)
	//		{
	//			disMin = abs(cen_r - 2.5);
	//			Mat rot2Z_cen(1, 3, CV_64FC1, Scalar(-1));
	//			rot2Z_cen.at<double>(0, 0) = cen_x;
	//			rot2Z_cen.at<double>(0, 1) = cen_y;
	//			rot2Z_cen.at<double>(0, 2) = cen_z;

	//			studCenter = rotInfo2.R.inv()*(rot2Z_cen.t());
	//		}
	//	}
	//}

	//180525
	//string strResultPath = "circlePoints.txt";
	//Mat picture = Mat::zeros(1000, 1000, CV_8UC3);
	//LaserSensorIO.WriteRowFirstAPP(strResultPath, rotInfo.XYRoted);
	//for (int i = 0; i != rotInfo.XYRoted.rows; i++)
	//{
	//	Point2d Pt;
	//	Pt.x = (rotInfo.XYRoted.at<double>(i, 0)) * 50;
	//	Pt.y = (115 + rotInfo.XYRoted.at<double>(i, 1)) * 50;
	//	circle(picture, Pt, 1, Scalar(0, 0, 255), -1);
	//}
	//stringstream a;
	//string strNum;
	//a << num;
	//a >> strNum;
	//string strImageSavePath = "Circle\\" + strNum + ".tiff";
	//imwrite(strImageSavePath, picture);

	//距离之和最小法，180912
	vector<PTS> points;
	//bool flag = false;
	double cen_z = rotInfo.XYRoted.at<double>(0, 2);

	//for (int i = 0; i != vecFittingPoints.size(); i++)
	//{
	//	if ((!flag) && (vecXYRoted[i].rows != 0))
	//	{
	//		cen_z = vecXYRoted[i].at<double>(0, 2);
	//		flag = true;
	//	}
	for (int j = 0; j != rotInfo.XYRoted.rows; j++)
	{
		PTS pt(rotInfo.XYRoted.at<double>(j, 0), rotInfo.XYRoted.at<double>(j, 1));
		points.push_back(pt);
	}
	//}
	//if (!flag)
	//{
	//	return studFeature;
	//}

	double cen_x = 0;
	double cen_y = -3;
	double cen_r = -1;
	Mat studCenter;
	CircleFitSolver *cfs = new CircleFitSolver();
	if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
	{
		Mat rot2Z_cen(1, 3, CV_64FC1, Scalar(-1));
		rot2Z_cen.at<double>(0, 0) = cen_x;
		rot2Z_cen.at<double>(0, 1) = cen_y;
		rot2Z_cen.at<double>(0, 2) = cen_z;

		studCenter = rotInfo.R.inv()*(rot2Z_cen.t());
	}
	//**********************//

	//Hough圆检测去除异常点，并拟合圆
	//Mat studCenter = HoughCircle(vecFittingPoints, vecXYRoted, rotInfo.XYRoted, 2.8, 3.0, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe); //180525 原版
	//Mat studCenter = HoughCircle2(axesMat, num, vecFittingPoints, vecXYRoted, rotInfo.XYRoted, 2.8, 3.0, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
	//Mat studCenter = HoughCircle(axesMat, num, vecFittingPoints, vecXYRoted, rotInfo.XYRoted, 2.8, 3.0, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);

	if (studCenter.empty())
	{
		return studFeature;
	}
	else
	{
		studFeature[0] = studCenter.t();
		return studFeature;
	}
}

Point3f C3DLaserSensorFeatureExtraction::calcLinePlaneIntersection(Mat linePoint, Mat lineVector, Mat planePoint, Mat planeVector)
{
	Point3f defaultIntersection(-1, -1, -1);

	double vpt = lineVector.dot(planeVector);
	if (vpt == 0)
	{
		LaserSensorIO.WriteLog("Error:直线与平面没有交点");
		return defaultIntersection;
	}

	double t = (planePoint.dot(planeVector) - linePoint.dot(planeVector)) / vpt;
	Point3f intersection; 
	intersection.x = linePoint.at<double>(0, 0) + lineVector.at<double>(0, 0) * t;
	intersection.y = linePoint.at<double>(0, 1) + lineVector.at<double>(0, 1) * t;
	intersection.z = linePoint.at<double>(0, 2) + lineVector.at<double>(0, 2) * t;
	
	return intersection;
}

vector<vector<Mat>> C3DLaserSensorFeatureExtraction::distLineAndStud15(vector<Mat> vecCameraCoors)//用于区分相机坐标系下的螺柱与平面点云
{
	vector<vector<Mat>> result(2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < vecCameraCoors.size(); ++i)
	{
		for (int j = 0; j < vecCameraCoors[i].rows; ++j)
		{
			pcl::PointXYZ pt;
			pt.x = vecCameraCoors[i].at<double>(j, 0);
			pt.y = vecCameraCoors[i].at<double>(j, 1);
			pt.z = vecCameraCoors[i].at<double>(j, 2);
			cloud->push_back(pt);
		}
	}
	//创建模型参数对象，这里指的是平面模型的方程ax+by+cz+d=0的参数
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//创建内点集合，用于存储分割得到的内点结果
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// 设置模型系数需要优化
	seg.setOptimizeCoefficients(true);
	// 必须配置
	seg.setModelType(0);
	//指定当前模型是平面模型
	seg.setMethodType(0);
	//指定分割算法采用随机采样一致性算法
	seg.setDistanceThreshold(0.05);//设置距离阈值
	seg.setInputCloud(cloud);//输入点云
	seg.segment(*inliers, *coefficients);//分割点云
	//std::cerr << "Cylinder coefficients: " << *coefficients << std::endl;
	//std::cerr << "模型内点个数: "
	//	<< inliers->indices.size() << std::endl;

	vector<Mat> studPoints(vecCameraCoors.size());
	vector<Mat> planePoints(vecCameraCoors.size());
	for (int i = 0; i != vecCameraCoors.size(); i++)
	{
		for (int j = 0; j != vecCameraCoors[i].rows; j++)
		{
			double distance = 0;
			for (int k = 0; k != 3; ++k)
			{
				distance += coefficients->values[k] * vecCameraCoors[i].at<double>(j, k);
			}
			distance += coefficients->values[3];
			distance = abs(distance);
			if (distance < 1)
			{
				planePoints[i].push_back(vecCameraCoors[i].row(j));
			}
			else
			{
				studPoints[i].push_back(vecCameraCoors[i].row(j));
			}
		}
	}

	result[0] = planePoints;
	result[1] = studPoints;
	return result;
}

vector<vector<Mat>> C3DLaserSensorFeatureExtraction::distLineAndStud(vector<Mat> vecCameraCoors, double r)//用于区分相机坐标系下的螺柱与平面点云
{
	Mat initPoints;
	if (vecCameraCoors.front().rows < vecCameraCoors.back().rows)//先判断扫描方向
	{
		//还是先剔除只有一半的圆弧段
        int maxRows = 0;
		//for (int i = 0; i < vecCameraCoors.size()/2 && i < 5; i++)
		for (int i = 0; i < vecCameraCoors.size() / 2 ; i++)
		{
			if (vecCameraCoors[i].rows > maxRows)
			    maxRows = vecCameraCoors[i].rows;
		}

		//for (int i = 0; i < vecCameraCoors.size()/2 && i < 5; i++)
		for (int i = 0; i < vecCameraCoors.size() / 2; i++)
		{
			if (vecCameraCoors[i].rows >= maxRows * 2/3)
			    initPoints.push_back(vecCameraCoors[i]);
		}
	}
	else
	{
		//还是先剔除只有一半的圆弧段
        int maxRows = 0;
		//for (int i = vecCameraCoors.size() - 1; i > vecCameraCoors.size()/2 && i > vecCameraCoors.size() - 6; i--)
		for (int i = vecCameraCoors.size() - 1; i > vecCameraCoors.size() / 2 ; i--)
		{
			if (vecCameraCoors[i].rows > maxRows)
			    maxRows = vecCameraCoors[i].rows;
		}

		//for (int i = vecCameraCoors.size() - 1; i > vecCameraCoors.size()/2 && i > vecCameraCoors.size() - 6; i--)
		for (int i = vecCameraCoors.size() - 1; i > vecCameraCoors.size() / 2 ; i--)
		{
			if (vecCameraCoors[i].rows >= maxRows * 2/3)
			    initPoints.push_back(vecCameraCoors[i]);
		}
	}

	Point3d initAxes = calcPCA(initPoints);
	//Mat initCenter = Kasa(initPoints, initAxes);

	//180730
	Mat axesMat = Mat(initAxes);
	Mat meanPoint(1, initPoints.cols, CV_64FC1, cvScalar(0));
	for (int i = 0; i != initPoints.cols; i++)
	{
		meanPoint.at<double>(0, i) = cv::mean(initPoints.col(i)).val[0];
	}
	Mat dataAdjust(initPoints.rows, initPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != initPoints.rows; cnt++)
		dataAdjust.row(cnt) = initPoints.row(cnt) - meanPoint;
	Mat projectedPoints = initPoints - dataAdjust * axesMat * axesMat.t();
	//旋转至与XY平面平行
	RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);

	Mat Z = rotInfo.XYRoted.col(0).mul(rotInfo.XYRoted.col(0)) + rotInfo.XYRoted.col(1).mul(rotInfo.XYRoted.col(1));
	Mat X(rotInfo.XYRoted.rows, 3, CV_64FC1, Scalar(0));
	X.col(0) = rotInfo.XYRoted.col(0) * 1;
	X.col(1) = rotInfo.XYRoted.col(1) * 1;
	X.col(2).setTo(cv::Scalar(1));
	Mat XTX = X.t() * X;
	Mat Par = XTX.inv(DECOMP_SVD) * (X.t() * Z);

	Mat initCenter(1, 3, CV_64FC1, Scalar(0));
	initCenter.at<double>(0, 0) = Par.at<double>(0, 0) / 2;
	initCenter.at<double>(0, 1) = Par.at<double>(1, 0) / 2;
	initCenter.at<double>(0, 2) = rotInfo.XYRoted.at<double>(0, 2);
	//initCenter = initCenter * rotInfo.R;

	vector<Mat> studPoints(vecCameraCoors.size());
	vector<Mat> planePoints(vecCameraCoors.size());
	for (int i = 0; i != vecCameraCoors.size(); i++)
	{
		for (int j = 0; j != vecCameraCoors[i].rows; j++)
		{
			//Mat AB = vecCameraCoors[i].row(j) - initCenter;

			//180730
			Mat temp = vecCameraCoors[i].row(j) - meanPoint;
			Mat projectedPointTemp = vecCameraCoors[i].row(j) - temp * axesMat * axesMat.t();
			RotToZInfo rotInfoTemp = RotToZ(projectedPointTemp, axesMat);
			Mat AB = rotInfoTemp.XYRoted - initCenter;
			double distance = sqrt(pow(AB.at<double>(0, 0), 2) + pow(AB.at<double>(0, 1), 2));

			//double costheta = (AB.dot(Mat(initAxes).t())) / (norm(AB) * norm(Mat(initAxes)));
			//double sintheta = cv::sqrt(1 - pow(costheta, 2));
			//double distance = norm(AB) * sintheta;
			//if (distance < r * 1.5)
			if (distance < (r + 0.5))
			{
				studPoints[i].push_back(vecCameraCoors[i].row(j));
			}
			else
			{
				planePoints[i].push_back(vecCameraCoors[i].row(j));
			}
		}
	}

	vector<vector<Mat>> vecPoints(2);
	vecPoints[0] = planePoints;
	vecPoints[1] = studPoints;

	return vecPoints;
}

vector<vector<Mat>> C3DLaserSensorFeatureExtraction::distLineAndStud(const vector<Mat> &vecCameraCoors, double r,const Mat &axesMat, const int &bestCircle)//用于区分相机坐标系下的螺柱与平面点云
{
	Mat initPoints = vecCameraCoors[bestCircle];

	//180730
	Mat meanPoint(1, initPoints.cols, CV_64FC1, cvScalar(0));
	for (int i = 0; i != initPoints.cols; i++)
	{
		meanPoint.at<double>(0, i) = cv::mean(initPoints.col(i)).val[0];
	}
	Mat dataAdjust(initPoints.rows, initPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != initPoints.rows; cnt++)
		dataAdjust.row(cnt) = initPoints.row(cnt) - meanPoint;
	Mat projectedPoints = initPoints - dataAdjust * axesMat * axesMat.t();
	//旋转至与XY平面平行
	RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);

	Mat Z = rotInfo.XYRoted.col(0).mul(rotInfo.XYRoted.col(0)) + rotInfo.XYRoted.col(1).mul(rotInfo.XYRoted.col(1));
	Mat X(rotInfo.XYRoted.rows, 3, CV_64FC1, Scalar(0));
	X.col(0) = rotInfo.XYRoted.col(0) * 1;
	X.col(1) = rotInfo.XYRoted.col(1) * 1;
	X.col(2).setTo(cv::Scalar(1));
	Mat XTX = X.t() * X;
	Mat Par = XTX.inv(DECOMP_SVD) * (X.t() * Z);

	Mat initCenter(1, 3, CV_64FC1, Scalar(0));
	initCenter.at<double>(0, 0) = Par.at<double>(0, 0) / 2;
	initCenter.at<double>(0, 1) = Par.at<double>(1, 0) / 2;
	initCenter.at<double>(0, 2) = rotInfo.XYRoted.at<double>(0, 2);
	//initCenter = initCenter * rotInfo.R;

	vector<Mat> studPoints(vecCameraCoors.size());
	vector<Mat> planePoints(vecCameraCoors.size());
	for (int i = 0; i != vecCameraCoors.size(); i++)
	{
		for (int j = 0; j != vecCameraCoors[i].rows; j++)
		{
			//Mat AB = vecCameraCoors[i].row(j) - initCenter;

			//180730
			Mat temp = vecCameraCoors[i].row(j) - meanPoint;
			Mat projectedPointTemp = vecCameraCoors[i].row(j) - temp * axesMat * axesMat.t();
			RotToZInfo rotInfoTemp = RotToZ(projectedPointTemp, axesMat);
			Mat AB = rotInfoTemp.XYRoted - initCenter;
			double distance = sqrt(pow(AB.at<double>(0, 0), 2) + pow(AB.at<double>(0, 1), 2));

			//double costheta = (AB.dot(Mat(initAxes).t())) / (norm(AB) * norm(Mat(initAxes)));
			//double sintheta = cv::sqrt(1 - pow(costheta, 2));
			//double distance = norm(AB) * sintheta;
			if (distance < r * 1.5)
			//if (distance < (r + 0.5))
			{
				studPoints[i].push_back(vecCameraCoors[i].row(j));
			}
			else
			{
				planePoints[i].push_back(vecCameraCoors[i].row(j));
			}
		}
	}

	//int maxRows = 0;
	//for (int i = 0; i != vecCameraCoors.size(); i++)
	//{
	//	if (studPoints[i].rows > maxRows)
	//		maxRows = studPoints[i].rows;
	//}
	//vector<Mat> vecXYRoted(vecCameraCoors.size());
	//vector<Mat> vecProjectedPoints(vecCameraCoors.size());
	//vector<double> centre_y(vecCameraCoors.size());
	//int num = 0;
	//double meanY = 0;
	//for (int i = 0; i != vecCameraCoors.size(); i++)
	//{
	//	if (studPoints[i].rows < maxRows * 2 / 3)
	//	{
	//		studPoints[i].resize(0);
	//		centre_y[i] = 0;
	//		continue;
	//	}
	//	Mat dataAdjust2(studPoints[i].rows, studPoints[i].cols, CV_64FC1);
	//	for (int j = 0; j != dataAdjust2.rows; j++)
	//	{
	//		dataAdjust2.row(j) = studPoints[i].row(j) - meanPoint;
	//	}
	//	vecProjectedPoints[i] = studPoints[i] - dataAdjust2 * axesMat * axesMat.t();
	//	RotToZInfo rotInfo2 = RotToZ(vecProjectedPoints[i], axesMat);
	//	vecXYRoted[i] = rotInfo2.XYRoted;
	//	vector<PTS> points;
	//	double cen_z = vecXYRoted[i].at<double>(0, 2);
	//	for (int j = 0; j != vecXYRoted[i].rows; j++)
	//	{
	//		PTS pt(vecXYRoted[i].at<double>(j, 0), vecXYRoted[i].at<double>(j, 1));
	//		points.push_back(pt);
	//	}

	//	double cen_x = 0;
	//	double cen_y = -3;
	//	double cen_r = -1;
	//	CircleFitSolver *cfs = new CircleFitSolver();
	//	if (cfs->circleFitL1(points, cen_x, cen_y, cen_r))
	//	{
	//		if (cen_r > r * 1.5)
	//		{
	//			studPoints[i].resize(0);
	//			centre_y[i] = 0;
	//			continue;
	//		}
	//		meanY += cen_y;
	//		++num;
	//		centre_y[i] = cen_y;
	//	}
	//}
	//meanY /= num;
	//for (int i = 0; i != vecCameraCoors.size(); i++)
	//{
	//	if (abs(centre_y[i] - meanY) > 0.3)
	//	{
	//		studPoints[i].resize(0);
	//	}
	//}

	vector<vector<Mat>> vecPoints(2);
	vecPoints[0] = planePoints;
	vecPoints[1] = studPoints;

	return vecPoints;
}


//Mat C3DLaserSensorFeatureExtraction::Kasa(Mat fittingPoints, Point3d axes)
//{
//	//轴线方向投影
//	Mat axesMat = Mat(axes);  //将Point3d转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
//	Mat meanPoint(1, fittingPoints.cols, CV_64FC1, cvScalar(0));
//	for (int i = 0; i != fittingPoints.cols; i++)
//	{
//		meanPoint.at<double>(0, i) = cv::mean(fittingPoints.col(i)).val[0];
//	}
//	Mat dataAdjust(fittingPoints.rows, fittingPoints.cols, CV_64FC1);
//	for (int cnt = 0; cnt != fittingPoints.rows; cnt++)
//		dataAdjust.row(cnt) = fittingPoints.row(cnt) - meanPoint;
//    Mat projectedPoints = fittingPoints - dataAdjust * axesMat * axesMat.t();
//    //旋转至与XY平面平行
//    RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);
//
//	Mat Z = rotInfo.XYRoted.col(0).mul(rotInfo.XYRoted.col(0)) + rotInfo.XYRoted.col(1).mul(rotInfo.XYRoted.col(1));
//    Mat X(rotInfo.XYRoted.rows, 3, CV_64FC1, Scalar(0));
//	X.col(0) = rotInfo.XYRoted.col(0) * 1;
//    X.col(1) = rotInfo.XYRoted.col(1) * 1;
//	X.col(2).setTo(cv::Scalar(1));
//	Mat XTX = X.t() * X;
//	Mat Par = XTX.inv(DECOMP_SVD) * (X.t() * Z);
//
//	Mat center(1, 3, CV_64FC1, Scalar(0));
//	center.at<double>(0, 0) = Par.at<double>(0, 0) / 2;
//	center.at<double>(0, 1) = Par.at<double>(1, 0) / 2;
//	center.at<double>(0, 2) = rotInfo.XYRoted.at<double>(0, 2);
//	//center = center * rotInfo.R;
//
//	return center;
//}

vector<Mat>  C3DLaserSensorFeatureExtraction::StudFit3(vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold/* =0.2*/, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)  //柱面返回中心坐标值+法线
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
	Mat defaultStudAxes(1, 3, CV_64FC1, Scalar(-1));
	vector<Mat> studFeature(2);
	studFeature[0] = defaultStudCenter;
	studFeature[1] = defaultStudAxes;

	//剔除只有一半的圆弧段
	int maxRows = 0;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows > maxRows)
			maxRows = vecFittingPoints[i].rows;
	}

	Mat FittingPoints;
	int cnt = 0;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows >= maxRows * 3/4)
		{
			FittingPoints.push_back(vecFittingPoints[i]);
			cnt ++;
		}
	}

	if (FittingPoints.empty())
		return studFeature;

	string strStudPointsPath = "studPoints.txt";
	//LaserSensorIO.WriteRowFirstTrunc(strStudPointsPath, FittingPoints);
	LaserSensorIO.WriteRowFirstAPP(strStudPointsPath, FittingPoints);

    Point3d axes =  calcPCA(FittingPoints);
	studFeature[1] = Mat(axes).t();  

	//轴线方向投影
	Mat axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
		dataAdjust.row(cnt) = FittingPoints.row(cnt) - planePoint;
    Mat projectedPoints = FittingPoints - dataAdjust * axesMat * axesMat.t();
					string strResultPath111 = "pro.txt";
				LaserSensorIO.WriteRowFirstAPP(strResultPath111, projectedPoints);
    //旋转至与XY平面平行
    RotToZInfo rotInfo=RotToZ(projectedPoints, axesMat);

	//Hough圆检测去除异常点
	Mat finalFittingPoints = HoughCircle(projectedPoints, rotInfo.XYRoted, 2.5, 2.8, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);

	if (finalFittingPoints.rows < 50)
	{
		LaserSensorIO.WriteLog("Error(11):2_柱面拟合点云数量<50");
		return studFeature;
	}
					string strResultPath1111 = "pro1.txt";
				LaserSensorIO.WriteRowFirstAPP(strResultPath1111, finalFittingPoints);

	//圆拟合
	CircleFit3DInfo studInfo = CircleFit3D(finalFittingPoints, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
	if (studInfo.Center.empty())
	{
		return studFeature;
	} 
	else
	{
		studFeature[0] = studInfo.Center.t();
		return studFeature;
	}
}

vector<Mat>  C3DLaserSensorFeatureExtraction::StudFit3(int num, vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold/* =0.2*/, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)  //柱面返回中心坐标值+法线
{
	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
	Mat defaultStudAxes(1, 3, CV_64FC1, Scalar(-1));
	vector<Mat> studFeature(2);
	studFeature[0] = defaultStudCenter;
	studFeature[1] = defaultStudAxes;

	//剔除只有一半的圆弧段
	int maxRows = 0;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows > maxRows)
			maxRows = vecFittingPoints[i].rows;
	}

	Mat FittingPoints2;
	for (int i = 0; i != vecFittingPoints.size(); i++)
	{
		if (vecFittingPoints[i].rows >= maxRows * 3 / 4)
		{
			FittingPoints2.push_back(vecFittingPoints[i]);
		}
	}

	Mat FittingPoints;
	if (vecFittingPoints[num].rows >= maxRows * 3 / 4)
	{
		FittingPoints.push_back(vecFittingPoints[num]);
	}

	if (FittingPoints.empty())
		return studFeature;

	string strStudPointsPath = "studPoints.txt";
	//LaserSensorIO.WriteRowFirstTrunc(strStudPointsPath, FittingPoints);
	LaserSensorIO.WriteRowFirstAPP(strStudPointsPath, FittingPoints);

	Point3d axes = calcPCA(FittingPoints2);
	studFeature[1] = Mat(axes).t();

	//轴线方向投影
	Mat axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
	Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
	for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
		dataAdjust.row(cnt) = FittingPoints.row(cnt) - planePoint;
	Mat projectedPoints = FittingPoints - dataAdjust * axesMat * axesMat.t();
	string strResultPath111 = "pro.txt";
	LaserSensorIO.WriteRowFirstAPP(strResultPath111, projectedPoints);
	//旋转至与XY平面平行
	RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);

	//Hough圆检测去除异常点
	//Mat finalFittingPoints = HoughCircle(projectedPoints, rotInfo.XYRoted, 2.8, 3.0, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
	//if (finalFittingPoints.rows < 50)
	//{
	//	LaserSensorIO.WriteLog("Error(11):2_柱面拟合点云数量<50");
	//	return studFeature;
	//}
	//string strResultPath1111 = "pro1.txt";
	//LaserSensorIO.WriteRowFirstAPP(strResultPath1111, finalFittingPoints);

	//180516
	//RotToZInfo rotInfo2 = RotToZ(finalFittingPoints, axesMat);
	//string strResultPath = "circlePoints.txt";
	//string strResultPath2 = "circlePointsFinal.txt";
	//LaserSensorIO.WriteRowFirstAPP(strResultPath, rotInfo.XYRoted);
	//LaserSensorIO.WriteRowFirstAPP(strResultPath2, rotInfo2.XYRoted);
	//Mat picture = Mat::zeros(1000, 1000, CV_8UC3);
	//Mat picture2 = Mat::zeros(1000, 1000, CV_8UC3);
	//for (int i = 0; i < rotInfo.XYRoted.rows; ++i)
	//{
	//	Point2d Pt;
	//	Pt.x = rotInfo.XYRoted.at<double>(i, 0) * 100;
	//	Pt.y = (105 + rotInfo.XYRoted.at<double>(i, 1)) * 100;
	//	circle(picture, Pt, 1, Scalar(0, 0, 255), -1);
	//}
	//for (int i = 0; i < rotInfo2.XYRoted.rows; ++i)
	//{
	//	Point2d Pt;
	//	Pt.x = rotInfo2.XYRoted.at<double>(i, 0)*100;
	//	Pt.y = (105+rotInfo2.XYRoted.at<double>(i, 1))*100;
	//	circle(picture2, Pt, 1, Scalar(0, 0, 255), -1);
	//}
	//stringstream a;
	//string strNum;
	//a << num;
	//a >> strNum;
	//string strImageSavePath = "Circle\\" + strNum + ".tiff";
	//string strImageSavePath2 = "CircleFinal\\"+strNum+".tiff";
	//imwrite(strImageSavePath, picture);
	//imwrite(strImageSavePath2, picture2);

	//圆拟合
	CircleFit3DInfo studInfo = CircleFit3D(projectedPoints, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
	if (studInfo.Center.empty())
	{
		return studFeature;
	}
	else
	{
		studFeature[0] = studInfo.Center.t();
		return studFeature;
	}
}

//vector<Mat>  C3DLaserSensorFeatureExtraction::StudFit3(int num, vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold/* =0.2*/, double CircleGrossErrorCoe/* =0.1 */, double CircleNormalErrorCoe/* =3 */)  //柱面返回中心坐标值+法线
//{
//	Mat defaultStudCenter(1, 3, CV_64FC1, Scalar(-1));
//	Mat defaultStudAxes(1, 3, CV_64FC1, Scalar(-1));
//	vector<Mat> studFeature(2);
//	studFeature[0] = defaultStudCenter;
//	studFeature[1] = defaultStudAxes;
//
//	//剔除只有一半的圆弧段
//	int maxRows = 0;
//	for (int i = 0; i != vecFittingPoints.size(); i++)
//	{
//		if (vecFittingPoints[i].rows > maxRows)
//			maxRows = vecFittingPoints[i].rows;
//	}
//
//	Mat FittingPoints;
//	int cnt = 0;
//	for (int i = 0; i != vecFittingPoints.size(); i++)
//	{
//		if (vecFittingPoints[i].rows >= maxRows * 3 / 4)
//		{
//			FittingPoints.push_back(vecFittingPoints[i]);
//			cnt++;
//		}
//	}
//
//	if (FittingPoints.empty())
//		return studFeature;
//
//	string strStudPointsPath = "studPoints.txt";
//	//LaserSensorIO.WriteRowFirstTrunc(strStudPointsPath, FittingPoints);
//	LaserSensorIO.WriteRowFirstAPP(strStudPointsPath, FittingPoints);
//
//	Point3d axes = calcPCA(FittingPoints);
//	studFeature[1] = Mat(axes).t();
//
//	//轴线方向投影
//	Mat axesMat = Mat(axes);  //将Point3f转换为Mat类型，方便转置以及使用已有函数，axesMat为3*1
//	Mat dataAdjust(FittingPoints.rows, FittingPoints.cols, CV_64FC1);
//	for (int cnt = 0; cnt != FittingPoints.rows; cnt++)
//		dataAdjust.row(cnt) = FittingPoints.row(cnt) - planePoint;
//	Mat projectedPoints = FittingPoints - dataAdjust * axesMat * axesMat.t();
//	string strResultPath111 = "pro.txt";
//	LaserSensorIO.WriteRowFirstAPP(strResultPath111, projectedPoints);
//	//旋转至与XY平面平行
//	RotToZInfo rotInfo = RotToZ(projectedPoints, axesMat);
//
//	//Hough圆检测去除异常点
//	Mat finalFittingPoints = HoughCircle(projectedPoints, rotInfo.XYRoted, 2.8, 3.0, 0.02, 0.02, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
//	if (finalFittingPoints.rows < 50)
//	{
//		LaserSensorIO.WriteLog("Error(11):2_柱面拟合点云数量<50");
//		return studFeature;
//	}
//	string strResultPath1111 = "pro1.txt";
//	LaserSensorIO.WriteRowFirstAPP(strResultPath1111, finalFittingPoints);
//
//	//180516
//	//RotToZInfo rotInfo2 = RotToZ(finalFittingPoints, axesMat);
//	//string strResultPath = "circlePoints.txt";
//	//string strResultPath2 = "circlePointsFinal.txt";
//	//LaserSensorIO.WriteRowFirstAPP(strResultPath, rotInfo.XYRoted);
//	//LaserSensorIO.WriteRowFirstAPP(strResultPath2, rotInfo2.XYRoted);
//	//Mat picture = Mat::zeros(1000, 1000, CV_8UC3);
//	//Mat picture2 = Mat::zeros(1000, 1000, CV_8UC3);
//	//for (int i = 0; i < rotInfo.XYRoted.rows; ++i)
//	//{
//	//	Point2d Pt;
//	//	Pt.x = rotInfo.XYRoted.at<double>(i, 0) * 100;
//	//	Pt.y = (105 + rotInfo.XYRoted.at<double>(i, 1)) * 100;
//	//	circle(picture, Pt, 1, Scalar(0, 0, 255), -1);
//	//}
//	//for (int i = 0; i < rotInfo2.XYRoted.rows; ++i)
//	//{
//	//	Point2d Pt;
//	//	Pt.x = rotInfo2.XYRoted.at<double>(i, 0)*100;
//	//	Pt.y = (105+rotInfo2.XYRoted.at<double>(i, 1))*100;
//	//	circle(picture2, Pt, 1, Scalar(0, 0, 255), -1);
//	//}
//	//stringstream a;
//	//string strNum;
//	//a << num;
//	//a >> strNum;
//	//string strImageSavePath = "Circle\\" + strNum + ".tiff";
//	//string strImageSavePath2 = "CircleFinal\\"+strNum+".tiff";
//	//imwrite(strImageSavePath, picture);
//	//imwrite(strImageSavePath2, picture2);
//
//	//圆拟合
//	CircleFit3DInfo studInfo = CircleFit3D(finalFittingPoints, CircleMeanErrorThreshold, CircleGrossErrorCoe, CircleNormalErrorCoe);
//	if (studInfo.Center.empty())
//	{
//		return studFeature;
//	}
//	else
//	{
//		studFeature[0] = studInfo.Center.t();
//		return studFeature;
//	}
//}
#pragma endregion

#pragma endregion 

#pragma endregion

void C3DLaserSensorFeatureExtraction::Mat2Point(vector<Point> &CloudPoints,Mat ImageCoor)
{
    for (int RowIndex=0; RowIndex<ImageCoor.rows; RowIndex++)
    {
		Point Pt;
		Pt.x=(int)ImageCoor.at<double>(RowIndex,1);
		Pt.y=(int)ImageCoor.at<double>(RowIndex,0);
		CloudPoints.push_back(Pt);
	}
}

Mat C3DLaserSensorFeatureExtraction::FindMax(vector<Mat> &GrayPics)
{
	//Mat MaxPic(GrayPics.at(0).rows,GrayPics.at(0).cols,GrayPics.at(0).depth(),Scalar(0));
	for (vector<Mat>::size_type RowIndex = 1; RowIndex < GrayPics.size(); RowIndex++)
	{
		addWeighted(GrayPics[0], 1, GrayPics[RowIndex], 1, 0.0, GrayPics[0]);
	}
	return GrayPics[0];
	//for (int Rows=0;Rows<MaxPic.rows;Rows++)
	//{
	//	for (int Cols=0; Cols<MaxPic.cols; Cols++)
	//	{
	//		uchar MaxValue=0;
	//		for (vector<Mat>::size_type RowIndex=0; RowIndex<GrayPics.size(); RowIndex++)
	//		{
	//			if (MaxValue<GrayPics.at(RowIndex).at<uchar>(Rows,Cols))
	//			{
	//				MaxValue=GrayPics.at(RowIndex).at<uchar>(Rows,Cols)+0;
	//			}
	//		}
	//		MaxPic.at<uchar>(Rows,Cols)=MaxValue;
	//	}
	//}

	//return MaxPic;
}

Rect C3DLaserSensorFeatureExtraction::ExtendedOrignalROI(Rect OriginalRect,int VOffsetPixel,int HOffsetPixel,int HightValue,int WidthValue)
{
	Rect StaticROI;
	StaticROI.x=(OriginalRect.x-HOffsetPixel>0)?OriginalRect.x-HOffsetPixel:10;
	StaticROI.y=(OriginalRect.y-VOffsetPixel>0)?OriginalRect.y-VOffsetPixel:10;
	StaticROI.width=(StaticROI.x+OriginalRect.width+2*HOffsetPixel>WidthValue)?(WidthValue-10-StaticROI.x):(2*HOffsetPixel+OriginalRect.width);
	StaticROI.height=(StaticROI.y+OriginalRect.height+2*VOffsetPixel>HightValue)?(HightValue-10-StaticROI.y):(2*VOffsetPixel+OriginalRect.height);
	return StaticROI;
}

bool C3DLaserSensorFeatureExtraction::LaserScanSensor3DFeatrueExtraction()															 																		 
{
	//////////////////////////////
	const double ImageHoleDistance=30; ///////点间距
	const double ImageSlotDistance=30;
	const double ImageSquareDistance=30;
	//////////////////////
	const double FirstImageHoleDistance=100;////// 第一张图片，圆孔像素之差
	const double FirstImageSlotDistance=100;////// 第一张图片，腰槽孔像素之差
	const double FirstImageSquareDistance=100;////// 第一张图片，方孔像素之差
	////////////////////
	bool FirstPicFlag=true;
	const double LastCameraHoleDistance=3.0;/////最后一张图片两点云之间的距离3mm
	const double LastCameraSlotDistance=3.0;/////最后一张图片两点云之间的距离6mm
	const double LastCameraSquareDistance=3.0;/////最后一张图片两点云之间的距离3mm
	//////////////////////////
	//////////////// 去除异常点的参数配置
	const double PlaneMeanError=0.5;
	const double PlaneGrossError=0.5;
	const double PlaneNormalError=3;

	const double CircleMeanError=0.2;
	const double CircleGrossError=0.1;
	const double CircleNormalError=3;

	const double NutCircleMeanError=0.2;
	const double NutCircleGrossError=0.1;
	const double NutCircleNormalError=3;

	const double Line3DMeanError=0.2;
	const double Line3DGrossError=0.5;
	const double Line3DNormalError=3.01;

	const double ImageLineMeanError=10;
	const double ImageLineGrossError=1;
	const double ImageLineNormalError=10;

	//////////////////////////////////
     int m_type=0;
	//////////////////////////////////
	cout<<"待测特征类型:"<<endl;
	cin>>m_type;
	//////// 将图像坐标系下的坐标值转换到相机坐标系下
	switch(m_type)
	{
	case 0: ////////// 存储全点云光刀数据
		{

			string strReadPath = "NormalPart\\2\\";
			vector<Point> CloudPoints;
			vector<Mat> GrayPics;
			//Mat OneRGBPic;//////需要批量显示的图片
			Rect StaticROI;
			for (int i = 0; i != 23; i++)
		    {
                stringstream s;
			    string StrFileNum;
		        s << i;
			    s >> StrFileNum;
		        string strCylinderPath = strReadPath + StrFileNum + ".tiff";
                Mat Image = imread(strCylinderPath);
				GrayPics.push_back(Image);
		        Mat coor = LaserSensorPointCloudsExtraction(Image,30, 100);
			    if ((coor.at<double>(0, 0) < 0) || (coor.at<double>(0, 1) < 0))  //allPoints出现默认值
			    {
				    continue;
			    }
				Mat2Point(CloudPoints,coor);
		    }
			Mat OneRGBPic = FindMax(GrayPics);
			//cvtColor(GrayPic, OneRGBPic, CV_GRAY2BGR);
			//显示光刀点数据
			for (vector<Point>::size_type RowIndex = 0; RowIndex < CloudPoints.size(); RowIndex++)
			{
				circle(OneRGBPic, CloudPoints.at(RowIndex), 4, Scalar(0,0,255),-1);
			}
			//感兴趣区域提取及显示
			Rect OriginalRect = boundingRect( CloudPoints);
			//圆柱特征
			const int HOffsetValue = 130;
			const int VOffsetValue = 130;
			StaticROI = ExtendedOrignalROI(OriginalRect, VOffsetValue, HOffsetValue, OneRGBPic.rows, OneRGBPic.cols);
			rectangle(OneRGBPic, StaticROI, Scalar(255,0,0), 4);
			rectangle(OneRGBPic, Rect(200, 100, 800,800), Scalar(0,0,128), 2);

			//存储图片
			string strImageSavePath="totalImages.tiff";
			LaserSensorIO.CreFiles(strImageSavePath);
			cv::imwrite(strImageSavePath, OneRGBPic);
			break;
		}
	case  1://///////存储修边光刀数据
		{
			for (int cnt = 1; cnt != 2; cnt++)
			{
				stringstream a;
				string strNum;
				a << cnt;
				a >> strNum;
				string strReadPath = "NormalPart\\" + strNum + "\\";

				Mat CameraMatrix = LaserSensorIO.ReadCameraMatrix();
				Mat DistCoeffs = LaserSensorIO.ReadDistCoeffs();
				Mat LaserRotationPlaneParas = LaserSensorIO.ReadLaserRotatingPlanePara(23);

				vector<Mat> vecCoors(23);
				vector<Mat> vecCameraCoors;

				for (int i = 0; i != 23; i++)
				{
					stringstream s;
					string StrFileNum;
					s << i;
					s >> StrFileNum;
					string strCylinderPath = strReadPath + StrFileNum + ".tiff";
					Mat Image = imread(strCylinderPath);

					vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 400);

					//利用相机内参数和光平面标定参数将图像坐标值转换到相机坐标系下的坐标值
					if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
					{
						vecCameraCoors.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
					}
				}
			}
			break;
		}

	case 9://尝试不含第7张图 180517
			//2016.03.12  使用distLineAndStud函数区分直线与螺柱，并且用最简单的HoughCircle直接进行提取点云拟合圆
		{
			for (int cnt = 1; cnt != 101; cnt++)
			{
				/*180516*/
				//vector<Mat> GrayPics;
				//vector<Point> CloudPoints;

				stringstream a;
				string strNum;
				a << cnt;
				a >> strNum;
				string strReadPath = "NormalPart\\" + strNum + "\\Images\\";

			    Mat CameraMatrix = LaserSensorIO.ReadCameraMatrix();
			    Mat DistCoeffs = LaserSensorIO.ReadDistCoeffs();
			    Mat LaserRotationPlaneParas = LaserSensorIO.ReadLaserRotatingPlanePara(23); 

				vector<Mat> vecCoors(23);
			    vector<Mat> vecCameraCoors;
                for (int i = 0; i != 23; i++)
			    {
					if (i == 6)//180517
						continue;

                    stringstream s;
				    string StrFileNum;
			        s << i;
				    s >> StrFileNum;
			        string strCylinderPath = strReadPath + StrFileNum + ".tiff";
                    Mat Image = imread(strCylinderPath);

					//GrayPics.push_back(Image);//180516

			       //vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 400);//180517 此行为原版
					//vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 200);
					if (i < 7)
					{
						vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 400);
					}
					else
					{
						vecCoors[i] = LaserSensorStudExtraction3(Image, 100, 900, 400, 900, 45, 400);
					}

					//利用相机内参数和光平面标定参数将图像坐标值转换到相机坐标系下的坐标值
				    if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
				    {
				  	    vecCameraCoors.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
						//Mat2Point(CloudPoints, vecCoors[i]);//180516
				    }
			    }

				if (vecCameraCoors.size() < 8)
				{
					LaserSensorIO.WriteLog("Error:vecCameraCoors.size() < 8");
					return false;
				}

				/*180516*/
				//Mat OneRGBPic = FindMax(GrayPics);
				//for (vector<Point>::size_type RowIndex = 0; RowIndex < CloudPoints.size(); RowIndex++)
				//{
				//	circle(OneRGBPic, CloudPoints.at(RowIndex), 4, Scalar(0, 0, 255), -1);
				//}
				//string strImagePath = "Totalimage\\" + strNum + ".tiff";
				//imwrite(strImagePath, OneRGBPic);

				vector<vector<Mat>> vecPoints = distLineAndStud(vecCameraCoors, 3.0);

				Mat planeCoors;
				for (int i = 0; i != vecPoints[0].size(); i++)
				{
					if (vecPoints[0][i].rows > 50)
						planeCoors.push_back(vecPoints[0][i]);
				}
				//存储拟合数据
				string strPlanePointsPath = "planePoints.txt";
				LaserSensorIO.WriteRowFirstAPP(strPlanePointsPath, planeCoors);

				//拟合平面
				if (planeCoors.empty())
					return false;
                Mat finalPlaneCoors = PlaneThreshod(planeCoors, PlaneMeanError, PlaneGrossError, PlaneNormalError);
				string strResultPath2 = "finalPlaneCoors.txt";
				LaserSensorIO.WriteRowFirstAPP(strResultPath2, finalPlaneCoors);
				if (finalPlaneCoors.rows < 10)
					return false;
				PlaneFit3DInfo planeInfo = PlaneFit3D(finalPlaneCoors);
				string strPlanePointPath = "PlanePoint.txt";
				string strPlaneVectorPath = "PlaneVector.txt";
				LaserSensorIO.WriteRowFirstAPP(strPlanePointPath, planeInfo.Points.t());
				LaserSensorIO.WriteRowFirstAPP(strPlaneVectorPath, planeInfo.NormVector.t());

				//拟合螺柱
				if (vecPoints[1].empty())
					return false;
			    vector<Mat> studFeature = StudFit3(vecPoints[1], planeInfo.Points.t());
			    string strStudPointPath = "StudPoint.txt";
                string strStudAxesPath = "StudAxes.txt";
			    LaserSensorIO.WriteRowFirstAPP(strStudPointPath, studFeature[0]);
			    LaserSensorIO.WriteRowFirstAPP(strStudAxesPath, studFeature[1]);

				//计算直线与平面的交点
				Point3f studCenter = calcLinePlaneIntersection(studFeature[0], studFeature[1], planeInfo.Points.t(), planeInfo.NormVector.t());
				string strStudCenterPath = "StudCenter.txt";
				LaserSensorIO.WriteRowFirstAPP(strStudCenterPath, studCenter);
			}
			break;
		}
	case 11:  //获取圆柱图像坐标值
		{
			for (int i = 0, j = 30; i != 23; i++, j++)
			{
                stringstream s;
				string StrFileNum;
			    s << i;
				s >> StrFileNum;
			    string strCylinderPath = "NormalPart\\" + StrFileNum + ".tiff";
                Mat Image = imread(strCylinderPath);
			    //vector<Mat> vecCoor = LaserSensorCylinderExtraction(Image, 10, 900, 300, 1000, 30, 250);
				Mat vecCoor;
				if (i < 6)
				{
					vecCoor = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 400);
				}
				else
				{
					vecCoor = LaserSensorStudExtraction3(Image, 100, 900, 400, 900, 30, 400);
				}
				//vecCoor = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 400);
			    //显示光刀点数据
			    vector<Point> CloudPoints;
			    //Mat2Point(CloudPoints, vecCoor[0]);
				Mat2Point(CloudPoints, vecCoor);
				stringstream s2;
				string StrFileNum2;
				s2 << j;
				s2 >> StrFileNum2;
                string strImagePath = "NormalPart\\" + StrFileNum2 + ".tiff";
			    for (vector<Point>::size_type RowIndex = 0; RowIndex < CloudPoints.size(); RowIndex++)
			    {
				    circle(Image, CloudPoints.at(RowIndex), 2, Scalar(0,0,255), -1);
			    }
			    imwrite(strImagePath, Image);
			}
			break;
		}
	case 12://尝试不含第7张图 180517
			//2016.03.11 使用distLineAndStud函数区分直线和螺柱，暂时选择1.8r作为界限，并且使用StudFit2剔除只有半段的圆弧，使用HoughCircle直接提取某一整条光刀线进行圆的拟合
		{
			for (int cnt = 1; cnt != 101; cnt++)
			{
				stringstream a;
				string strNum;
				a << cnt;
				a >> strNum;
				string strReadPath = "NormalPart\\" + strNum + "\\Images\\";

			    Mat CameraMatrix = LaserSensorIO.ReadCameraMatrix();
			    Mat DistCoeffs = LaserSensorIO.ReadDistCoeffs();
			    Mat LaserRotationPlaneParas = LaserSensorIO.ReadLaserRotatingPlanePara(23); 

				vector<Mat> vecCoors(23);
			    vector<Mat> vecCameraCoors;

                for (int i = 0; i != 23; i++)
			    {
					if (i == 6)//180517
						continue;
					
                    stringstream s;
				    string StrFileNum;
			        s << i;
				    s >> StrFileNum;
			        string strCylinderPath = strReadPath + StrFileNum + ".tiff";
                    Mat Image = imread(strCylinderPath);

					string strImagePath = "Hist\\" + strNum + "\\" + StrFileNum + ".tiff";

					if (i < 7)
					{
						vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 400);
					}
					else
					{
						vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 45, 400);
					}

					//利用相机内参数和光平面标定参数将图像坐标值转换到相机坐标系下的坐标值
				    if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
				    {
				  	    vecCameraCoors.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
				    }
			    }

				if (vecCameraCoors.size() < 8)
				{
					LaserSensorIO.WriteLog("Error:vecCameraCoors.size() < 8");
					return false;
				}

				vector<vector<Mat>> vecPoints = distLineAndStud(vecCameraCoors, 3.0);

				Mat planeCoors;
				for (int i = 0; i != vecPoints[0].size(); i++)
				{
					if (vecPoints[0][i].rows > 50)
						planeCoors.push_back(vecPoints[0][i]);
				}
				//存储拟合数据
				string strPlanePointsPath = "planePoints.txt";
				LaserSensorIO.WriteRowFirstAPP(strPlanePointsPath, planeCoors);

				//拟合平面
				if (planeCoors.empty())
					return false;
                Mat finalPlaneCoors = PlaneThreshod(planeCoors, PlaneMeanError, PlaneGrossError, PlaneNormalError);
				string strResultPath2 = "finalPlaneCoors.txt";
				LaserSensorIO.WriteRowFirstAPP(strResultPath2, finalPlaneCoors);
				if (finalPlaneCoors.rows < 10)
					return false;
				PlaneFit3DInfo planeInfo = PlaneFit3D(finalPlaneCoors);
				string strPlanePointPath = "PlanePoint.txt";
				string strPlaneVectorPath = "PlaneVector.txt";
				LaserSensorIO.WriteRowFirstAPP(strPlanePointPath, planeInfo.Points.t());
				LaserSensorIO.WriteRowFirstAPP(strPlaneVectorPath, planeInfo.NormVector.t());

				//拟合螺柱
				if (vecPoints[1].empty())
					return false;
			    //vector<Mat> studFeature = StudFit2(cnt, vecPoints[1], planeInfo.Points.t());
				vector<Mat> studFeature = StudFit2(cnt, vecPoints[1], planeInfo.Points.t());//180525原版
			    string strStudPointPath = "StudPoint.txt";
                string strStudAxesPath = "StudAxes.txt";
			    LaserSensorIO.WriteRowFirstAPP(strStudPointPath, studFeature[0]);
			    LaserSensorIO.WriteRowFirstAPP(strStudAxesPath, studFeature[1]);

				//计算直线与平面的交点
				Point3f studCenter = calcLinePlaneIntersection(studFeature[0], studFeature[1], planeInfo.Points.t(), planeInfo.NormVector.t());
				string strStudCenterPath = "StudCenter.txt";
				LaserSensorIO.WriteRowFirstAPP(strStudCenterPath, studCenter);
			}
			break;
		}
	case 13://试验区（张祎）
	{
		for (int cnt = 1; cnt != 101; cnt++)
		{
			/*180516*/
			//vector<Mat> GrayPics;
			//vector<Point> CloudPoints;

			stringstream a;
			string strNum;
			a << cnt;
			a >> strNum;
			string strReadPath = "NormalPart\\" + strNum + "\\Images\\";

			Mat CameraMatrix = LaserSensorIO.ReadCameraMatrix();
			Mat DistCoeffs = LaserSensorIO.ReadDistCoeffs();
			Mat LaserRotationPlaneParas = LaserSensorIO.ReadLaserRotatingPlanePara(23);

			vector<Mat> vecCoors(23);
			vector<Mat> vecCameraCoors;

			for (int i = 0; i != 23; i++)
			{
				if (i == 6)//180517
					continue;

				stringstream s;
				string StrFileNum;
				s << i;
				s >> StrFileNum;
				string strCylinderPath = strReadPath + StrFileNum + ".tiff";
				Mat Image = imread(strCylinderPath);

				//GrayPics.push_back(Image);//180516

				string strImagePath = "Hist\\" + strNum + "\\" + StrFileNum + ".tiff";

				if (i < 7)
				{
					vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 400);
				}
				else
				{
					double left = 550;
					double up = 370 + 20 * (i - 7);
					double right = 760;
					double down = up+60;
					//vecCoors[i] = LaserSensorStudExtraction2(i, strImagePath, Image, 100, 900, 400, 900, 400);
					vecCoors[i] = LaserSensorStudExtraction2(strImagePath, Image, up, down, left, right, 400);
				}

				//利用相机内参数和光平面标定参数将图像坐标值转换到相机坐标系下的坐标值
				if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
				{
					vecCameraCoors.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
					//Mat2Point(CloudPoints, vecCoors[i]);//180516
				}
			}

			/*180516*/
			//Mat OneRGBPic = FindMax(GrayPics);
			////for (vector<Point>::size_type RowIndex = 0; RowIndex < CloudPoints.size(); RowIndex++)
			////{
			////	circle(OneRGBPic, CloudPoints.at(RowIndex), 4, Scalar(0, 0, 255), -1);
			////}
			//string strImagePath = "Totalimage\\" + strNum + ".tiff";
			//imwrite(strImagePath, OneRGBPic);

			if (vecCameraCoors.size() < 8)
			{
				LaserSensorIO.WriteLog("Error:vecCameraCoors.size() < 8");
				return false;
			}

			vector<vector<Mat>> vecPoints = distLineAndStud(vecCameraCoors, 3.0);

			Mat planeCoors;
			for (int i = 0; i != vecPoints[0].size(); i++)
			{
				if (vecPoints[0][i].rows > 50)
					planeCoors.push_back(vecPoints[0][i]);
			}
			//存储拟合数据
			string strPlanePointsPath = "planePoints.txt";
			LaserSensorIO.WriteRowFirstAPP(strPlanePointsPath, planeCoors);

			//拟合平面
			if (planeCoors.empty())
				return false;
			Mat finalPlaneCoors = PlaneThreshod(planeCoors, PlaneMeanError, PlaneGrossError, PlaneNormalError);
			//string strResultPath2 = "finalPlaneCoors.txt";
			//LaserSensorIO.WriteRowFirstAPP(strResultPath2, finalPlaneCoors);
			if (finalPlaneCoors.rows < 10)
				return false;
			PlaneFit3DInfo planeInfo = PlaneFit3D(finalPlaneCoors);
			//string strPlanePointPath = "PlanePoint.txt";
			//string strPlaneVectorPath = "PlaneVector.txt";
			//LaserSensorIO.WriteRowFirstAPP(strPlanePointPath, planeInfo.Points.t());
			//LaserSensorIO.WriteRowFirstAPP(strPlaneVectorPath, planeInfo.NormVector.t());

			//拟合螺柱
			if (vecPoints[1].empty())
				return false;
			vector<Mat> studFeature = StudFit2(cnt, vecPoints[1], planeInfo.Points.t());//180525原版

			string strStudPointPath = "StudPoint.txt";
			string strStudAxesPath = "StudAxes.txt";
			LaserSensorIO.WriteRowFirstAPP(strStudPointPath, studFeature[0]);
			LaserSensorIO.WriteRowFirstAPP(strStudAxesPath, studFeature[1]);

			//计算直线与平面的交点
			Point3f studCenter = calcLinePlaneIntersection(studFeature[0], studFeature[1], planeInfo.Points.t(), planeInfo.NormVector.t());
			string strStudCenterPath = "StudCenter.txt";
			LaserSensorIO.WriteRowFirstAPP(strStudCenterPath, studCenter);
		}
		break;
	}
	case 14://试验区
	{
		for (int cnt = 0; cnt != 100; cnt++)
		{
			//验证自适应
			//int MinThresholdArray[23] = { 14, 14, 14, 14, 76, 14, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76 };

			/*180516*/
			vector<Mat> GrayPics;
			vector<Point> CloudPoints;

			stringstream a;
			string strNum;
			a << cnt;
			a >> strNum;
			string strReadPath = "NormalPart3\\" + strNum;
			//string strReadPath = "NormalPart\\" + strNum + "\\Images\\";

			Mat CameraMatrix = LaserSensorIO.ReadCameraMatrix();
			Mat DistCoeffs = LaserSensorIO.ReadDistCoeffs();
			Mat LaserRotationPlaneParas = LaserSensorIO.ReadLaserRotatingPlanePara(23);

			vector<Mat> vecCoors(23);
			vector<Mat> vecCameraCoors;
			//vector<Mat> vecCameraCoors2;

			int maxHeight = 0;
			int maxWidth = 0;
			int indexW;
			int studLeft = 1279;
			int studRight = 0;
			//int studCentre;
			vector<Rect> ROIRect;
			//vector<Rect> ROIRectL, ROIRectR;
			for (int i = 0; i != 23; i++)
			{
				stringstream s;
				string StrFileNum;
				s << (i + 1);
				//s << i;//张祎文件读法，别忘改相机参数
				s >> StrFileNum;
				string strCylinderPath = strReadPath + "\\" + StrFileNum + ".tiff";
				//string strCylinderPath = strReadPath + StrFileNum + ".tiff";
				Mat Image = imread(strCylinderPath);

				ROIRect.push_back(getRect(Image, 0, 959, 80, 1200, true));
				//ROIRect.push_back(getRect(MinThresholdArray[i], Image, 0, 959, 80, 1200, true));

				//if (ROIRect[i].width > 640)
				//{
				//	Rect tempRect = getRect(Image, 0, 959, 0, 1279);
				//}
				if (maxWidth < ROIRect[i].width)
				{
					maxWidth = ROIRect[i].width;
					indexW = i;
				}
			}
			if (indexW < 12)
			{
				int times = 0;
				for (int i = 0; i != 23; i++)
				{
					stringstream s;
					string StrFileNum;
					s << (i + 1);
					//s << i;//张祎文件读法，别忘改相机参数
					s >> StrFileNum;
					string strCylinderPath = strReadPath + "\\" + StrFileNum + ".tiff";
					//string strCylinderPath = strReadPath + StrFileNum + ".tiff";
					Mat Image = imread(strCylinderPath);
					if (i < 13)
					{
						if (times > 1)
						{
							//ROIRect[i] = getRect(Image, 0, 959, 400, 900, false);
							ROIRect[i] = getRect(MinThresholdArray[i], Image, 0, 959, 400, 900, false);
						}
						else
						{
							if (ROIRect[i].width > maxWidth / 2.0)
							{
								++times;
							}
						}
					}
					else
					{
						ROIRect[i] = getRect(Image, 0, 959, 400, 900, false);
						//ROIRect[i] = getRect(MinThresholdArray[i], Image, 0, 959, 400, 900, false);
					}
				}
			}
			else
			{
				int times = 0;
				for (int i = 22; i != -1; i--)
				{
					stringstream s;
					string StrFileNum;
					s << (i + 1);
					//s << i;//张祎文件读法，别忘改相机参数
					s >> StrFileNum;
					string strCylinderPath = strReadPath + "\\" + StrFileNum + ".tiff";
					//string strCylinderPath = strReadPath + StrFileNum + ".tiff";
					Mat Image = imread(strCylinderPath);
					if (i > 11)
					{
						if (times > 1)
						{
							ROIRect[i] = getRect(Image, 0, 959, 400, 900, false);
						}
						else
						{
							if (ROIRect[i].width > maxWidth / 2.0)
							{
								++times;
							}
						}
					}
					else
					{
						ROIRect[i] = getRect(Image, 0, 959, 400, 900, false);
					}
				}
			}

			vector<int> indexLine;
			indexLine.push_back(indexW);
			bool twolines = false;
			int maxRows = 0;
			int indexRows;
			int indexStud;
			if (indexW < 12) //记得将 > 12 的代码块统一
			{
				indexStud = indexW + 1;
				for (int i = 22; i != 12; i--)
				{
					if (ROIRect[i].width > maxRows)
					{
						maxRows = ROIRect[i].width;
						indexRows = i;
					}
					if (ROIRect[i].height > maxHeight)
					{
						maxHeight = ROIRect[i].height;
					}
					if ((ROIRect[i].x<studLeft) && (ROIRect[i].x > 401))
					{
						studLeft = ROIRect[i].x;
					}
					if (((ROIRect[i].x + ROIRect[i].width - 1)>studRight) && (ROIRect[i].x + ROIRect[i].width < 900))
					{
						studRight = ROIRect[i].x + ROIRect[i].width - 1;
					}
				}
				for (int i = 0; i != indexW; ++i)
				{
					if ((ROIRect[i].y < ROIRect[indexW].y) && (maxWidth - ROIRect[i].width) < (ROIRect[i].width - maxRows))
					{
						indexLine.push_back(i);
						twolines = true;
					}
				}
				if (!twolines)
				{
					indexLine.push_back(indexW + 1);
					twolines = true;
					indexStud = indexW + 2;
				}
				for (int i = indexStud; i != 13; ++i)
				{
					//if (ROIRect[i].width > 500)
					//{
					//	ROIRect[i].width = 500;
					//}
					double HdivWi = (ROIRect[i].height * 1.0) / (ROIRect[i].width * 1.0);
					double HdivWW = (ROIRect[indexW].height * 1.0) / (ROIRect[indexW].width * 1.0);
					double HdivWR = (ROIRect[indexRows].height * 1.0) / (ROIRect[indexRows].width * 1.0);
					if (((maxWidth - ROIRect[i].width) > (ROIRect[i].width - maxRows)) && (HdivWW > HdivWR)
						&& (((HdivWW - HdivWR) / 2.0 + HdivWR)> HdivWi))
					{
						if (ROIRect[i].height > maxHeight)
						{
							maxHeight = ROIRect[i].height;
						}
						if ((ROIRect[i].x<studLeft) && (ROIRect[i].x > 401))
						{
							studLeft = ROIRect[i].x;
						}
						if (((ROIRect[i].x + ROIRect[i].width - 1)>studRight) && (ROIRect[i].x + ROIRect[i].width < 900))
						{
							studRight = ROIRect[i].x + ROIRect[i].width - 1;
						}
					}
					else
						if (((maxWidth - ROIRect[i].width) > (ROIRect[i].width - maxRows)) && (HdivWW <= HdivWR)
							&& (((HdivWW - HdivWR) / 2.0 + HdivWR)< HdivWi))
						{
							if (ROIRect[i].height > maxHeight)
							{
								maxHeight = ROIRect[i].height;
							}
							if ((ROIRect[i].x<studLeft) && (ROIRect[i].x > 401))
							{
								studLeft = ROIRect[i].x;
							}
							if (((ROIRect[i].x + ROIRect[i].width - 1)>studRight) && (ROIRect[i].x + ROIRect[i].width < 900))
							{
								studRight = ROIRect[i].x + ROIRect[i].width - 1;
							}
						}
						//else
						//	if (!twolines)
						//	{
						//		indexLine.push_back(i);
						//		twolines = true;
						//		indexStud = i + 1;
						//	}
					//if ((maxWidth - ROIRect[i].width) > (ROIRect[i].width - maxRows)
					//	&& (abs(HdivWi / HdivWW - 1) > abs(HdivWi / HdivWR - 1)))
					//{
					//	indexStud = i;
					//	for (int j = i; j != 23; ++j)
					//	{
					//		if (ROIRect[j].height > maxHeight)
					//		{
					//			maxHeight = ROIRect[j].height;
					//		}
					//		if ((ROIRect[j].x<studLeft) && (ROIRect[j].x > 401))
					//		{
					//			studLeft = ROIRect[j].x;
					//		}
					//		if (((ROIRect[j].x + ROIRect[j].width - 1)>studRight) && (ROIRect[j].x + ROIRect[j].width < 900))
					//		{
					//			studRight = ROIRect[j].x + ROIRect[j].width - 1;
					//		}
					//	}
					//	//studCentre = (studLeft + studRight) / 2.0;
					//	//break;
					//}
					//else
					//{
					//	indexLine.push_back(i);
					//	twolines = true;
					//}
				}
			}
			else
			{
				indexStud = indexW - 1;
				for (int i = 0; i != 11; i++)
				{
					if (ROIRect[i].width > maxRows)
					{
						maxRows = ROIRect[i].width;
						indexRows = i;
					}
					if (ROIRect[i].height > maxHeight)
					{
						maxHeight = ROIRect[i].height;
					}
					if ((ROIRect[i].x<studLeft) && (ROIRect[i].x > 401))
					{
						studLeft = ROIRect[i].x;
					}
					if (((ROIRect[i].x + ROIRect[i].width - 1)>studRight) && (ROIRect[i].x + ROIRect[i].width < 900))
					{
						studRight = ROIRect[i].x + ROIRect[i].width - 1;
					}
				}
				for (int i = 22; i != indexW; --i)
				{
					if ((ROIRect[i].y < ROIRect[indexW].y) && (maxWidth - ROIRect[i].width) < (ROIRect[i].width - maxRows))
					{
						indexLine.push_back(i);
						twolines = true;
					}
				}
				if (!twolines)
				{
					indexLine.push_back(indexW - 1);
					twolines = true;
					indexStud = indexW - 2;
				}
				for (int i = indexStud; i != 10; --i)
				{
					double HdivWi = (ROIRect[i].height * 1.0) / (ROIRect[i].width * 1.0);
					double HdivWW = (ROIRect[indexW].height * 1.0) / (ROIRect[indexW].width * 1.0);
					double HdivWR = (ROIRect[indexRows].height * 1.0) / (ROIRect[indexRows].width * 1.0);
					if (((maxWidth - ROIRect[i].width) > (ROIRect[i].width - maxRows)) && (HdivWW > HdivWR)
						&& (((HdivWW - HdivWR) / 2.0 + HdivWR)> HdivWi))
					{
						if (ROIRect[i].height > maxHeight)
						{
							maxHeight = ROIRect[i].height;
						}
						if ((ROIRect[i].x<studLeft) && (ROIRect[i].x > 401))
						{
							studLeft = ROIRect[i].x;
						}
						if (((ROIRect[i].x + ROIRect[i].width - 1)>studRight) && (ROIRect[i].x + ROIRect[i].width < 900))
						{
							studRight = ROIRect[i].x + ROIRect[i].width - 1;
						}
					}
					else
						if (((maxWidth - ROIRect[i].width) >(ROIRect[i].width - maxRows)) && (HdivWW <= HdivWR)
							&& (((HdivWW - HdivWR) / 2.0 + HdivWR)< HdivWi))
						{
							if (ROIRect[i].height > maxHeight)
							{
								maxHeight = ROIRect[i].height;
							}
							if ((ROIRect[i].x<studLeft) && (ROIRect[i].x > 401))
							{
								studLeft = ROIRect[i].x;
							}
							if (((ROIRect[i].x + ROIRect[i].width - 1)>studRight) && (ROIRect[i].x + ROIRect[i].width < 900))
							{
								studRight = ROIRect[i].x + ROIRect[i].width - 1;
							}
						}
						//else
						//	if (!twolines)
						//	{
						//		indexLine.push_back(i);
						//		twolines = true;
						//		indexStud = i + 1;
						//	}
				}
			}
			//if (!twolines) return false;

			//Mat planeCoors;
			vector<Mat> planeCoorsVec;
			int bestCircle;
			bool firstStud = true;
			vector<int> similarVecUsed;
			vector<int> similarVec1;
			vector<int> similarVec2;
			for (int i = 0; i != 23; i++)
			{
				//if (i == 10 || i == 19)
				//{
				//	Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));
				//	vecCoors[i] = defaultAllPoints;
				//	continue;
				//}
				stringstream s;
				string StrFileNum;
				s << (i + 1);
				//s << i;//张祎文件读法
				s >> StrFileNum;
				string strCylinderPath = strReadPath + "\\" + StrFileNum + ".tiff";
				//string strCylinderPath = strReadPath + StrFileNum + ".tiff";
				Mat Image = imread(strCylinderPath);

				GrayPics.push_back(Image);//180516

				//string strImagePath = "Hist\\" + strNum + "\\" + StrFileNum + ".tiff";

				if ((ROIRect[i].width == 0) || (ROIRect[i].height == 0))
				{
					Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));
					vecCoors[i] = defaultAllPoints;
					continue;
				}

				if (((indexW < 12) && (i < indexStud)) || ((indexW >= 12) && (i > indexStud)))
				{
					int findIndex = find(indexLine.begin(), indexLine.end(), i) - indexLine.begin();
					if (findIndex == indexLine.size())
					{
						continue;
					}
					int left = 0;
					int right = 1279;
					//int up = ROIRect[i].y - (1 - ROIRect[i].width / maxWidth)*ROIRect[indexW].height / 2;
					//int down = up + ROIRect[indexW].height;
					int up = 0;
					int down = 959;
					//finalRect = getFinalRect(Image, up, down, left, right, false);

					//if ((up < 0) || (down>959))
					//{
					//	Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));
					//	vecCoors[i] = defaultAllPoints;
					//	continue;
					//}
					//if (up < 0)
					//{
					//	up = 0;
					//}
					//if (down > 959)
					//{
					//	down = 959;
					//}

					vecCoors[i] = LaserSensorStudExtraction(i, Image, up, down, left, right, false);
					//vecCoors[i] = LaserSensorStudExtraction(MinThresholdArray[i], i, Image, up, down, left, right, false);
					if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
					{
						planeCoorsVec.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
						Mat2Point(CloudPoints, vecCoors[i]);//180516
					}
				}
				else
				{
					int left = studLeft;
					int right = studRight;
					int up = ROIRect[i].y;
					int down = up + maxHeight;
					//finalRect = getFinalRect(Image, up, down, left, right, true, studCentre, maxRows);

					if ((up < 0) || (down>959))
					{
						Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));
						vecCoors[i] = defaultAllPoints;
						continue;
					}

					vecCoors[i] = LaserSensorStudExtraction(i, Image, up, down, left, right, true, maxRows);
					//vecCoors[i] = LaserSensorStudExtraction(MinThresholdArray[i], i, Image, up, down, left, right, true, maxRows);

					if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
					{
						int OriginSize = similarVec1.size();
						similarVec1.push_back(OriginSize);
						similarVec2.push_back(OriginSize);
						if (firstStud)
						{
							firstStud = false;
						}
						else
						{
							if ((abs(ROIRect[i].y - ROIRect[i - 1].y) <= 20) && (!((vecCoors[i - 1].at<double>(0, 0) < 0) || (vecCoors[i - 1].at<double>(0, 1) < 0))))
							{	
								similarVec1[OriginSize - 1] = OriginSize;
								similarVec1[OriginSize] = OriginSize - 1;
							}
							if ((abs(ROIRect[i].y - ROIRect[i - 1].y) <= 10) && (!((vecCoors[i - 1].at<double>(0, 0) < 0) || (vecCoors[i - 1].at<double>(0, 1) < 0))))
							{
								similarVec2[OriginSize - 1] = OriginSize;
								similarVec2[OriginSize] = OriginSize - 1;
							}
						}
					}

					//Mat tempCoors = LaserSensorStudExtraction4(Image, up, down, left, right, true, maxRows);
					//if (!((tempCoors.at<double>(0, 0) < 0) || (tempCoors.at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
					//{
					//	vecCameraCoors2.push_back(CameraCoorDatas(tempCoors, CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
					//}
				}

				//vecCoors[i] = LaserSensorStudExtraction2(strImagePath, Image, 0, 960, 400, 900, 400);
				//vecCoors[i] = LaserSensorStudExtraction2(Image, 0, 959, 400, 900, 100, 400);

				//if (i < 7)
				//{
				//	vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 30, 400);
				//}
				//else
				//{
				//	double left = 550;
				//	double up = 370 + 20 * (i - 7);
				//	double right = 760;
				//	double down = up+60;
				//	//vecCoors[i] = LaserSensorStudExtraction2(i, strImagePath, Image, 100, 900, 400, 900, 400);
				//	vecCoors[i] = LaserSensorStudExtraction2(strImagePath, Image, up, down, left, right, 400);
				//}

				//利用相机内参数和光平面标定参数将图像坐标值转换到相机坐标系下的坐标值
				//if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
				//{
				//	vecCameraCoors.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
				//	Mat2Point(CloudPoints, vecCoors[i]);//180516
				//}
			}

			bool UseSimilarVec2 = false;
			for (int i = 0; i != similarVec1.size(); ++i)
			{
				if ((similarVec1[i] != i) && (similarVec1[similarVec1[i]] != i))
				{
					UseSimilarVec2 = true;
					break;
				}
			}
			if (!UseSimilarVec2)
			{
				similarVecUsed = similarVec1;
			}
			else
			{
				similarVecUsed = similarVec2;
			}

			for (int i = 0; i != 23; i++)
			{
				if (((indexW < 12) && (i >= indexStud)) || ((indexW >= 12) && (i <= indexStud)))
				{
					if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
					{
						if (i == indexRows)
						{
							bestCircle = vecCameraCoors.size();
						}
						vecCameraCoors.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
						Mat2Point(CloudPoints, vecCoors[i]);//180516
					}
				}
			}

			/*180516*/
			Mat OneRGBPic = FindMax(GrayPics);
			for (vector<Point>::size_type RowIndex = 0; RowIndex < CloudPoints.size(); RowIndex++)
			{
				circle(OneRGBPic, CloudPoints.at(RowIndex), 4, Scalar(0, 0, 255), -1);
			}
			string strImagePath = "Totalimage\\" + strNum + ".tiff";
			imwrite(strImagePath, OneRGBPic);

			//if (vecCameraCoors.size() < 8)
			//{
			//	continue;
			//}
			if (vecCameraCoors.size() < 6)
			{
				continue;
			}

			//vector<vector<Mat>> vecPoints = distLineAndStud(vecCameraCoors, 5.0);

			Mat planeCoors;
			for (int i = 0; i != planeCoorsVec.size(); i++)
			{
				//if (planeCoorsVec[i].rows > 50)
					planeCoors.push_back(planeCoorsVec[i]);
			}
			//存储拟合数据
			//string strPlanePointsPath = "planePoints.txt";
			//LaserSensorIO.WriteRowFirstAPP(strPlanePointsPath, planeCoors);

			//拟合平面
			if (planeCoors.empty())
				return false;
			Mat finalPlaneCoors = PlaneThreshod(planeCoors, PlaneMeanError, PlaneGrossError, PlaneNormalError);
			//string strResultPath2 = "finalPlaneCoors.txt";
			//LaserSensorIO.WriteRowFirstAPP(strResultPath2, finalPlaneCoors);
			if (finalPlaneCoors.rows < 10)
				return false;
			PlaneFit3DInfo planeInfo = PlaneFit3D(finalPlaneCoors);
			//Mat NormVector(3, 1, CV_64FC1, Scalar(0));
			//NormVector.at<double>(0, 0) = -0.00107;
			//NormVector.at<double>(1, 0) = -0.78339;
			//NormVector.at<double>(2, 0) = 0.621532;
			//planeInfo.NormVector = NormVector;
			//Mat Point(3, 1, CV_64FC1, Scalar(0));
			//Point.at<double>(0, 0) = 0.688849;
			//Point.at<double>(1, 0) = -10.1293;
			//Point.at<double>(2, 0) = 121.894;
			//planeInfo.Points = Point;
			string strPlanePointPath = "PlanePoint.txt";
			string strPlaneVectorPath = "PlaneVector.txt";
			LaserSensorIO.WriteRowFirstAPP(strPlanePointPath, planeInfo.Points.t());
			LaserSensorIO.WriteRowFirstAPP(strPlaneVectorPath, planeInfo.NormVector.t());

			vector<vector<Mat>> vecPoints = distLineAndStud(vecCameraCoors, 5.0, planeInfo.NormVector, bestCircle);
			//vector<vector<Mat>> vecPoints2 = distLineAndStud(vecCameraCoors2, 5.0, planeInfo.NormVector, bestCircle);

			//for (int i = 0; i != vecPoints[1].size(); i++)
			//{
			//	if (vecPoints[1][i].rows > 50)
			//	{
			//		Mat finalPlaneCoors = PlaneThreshod(vecPoints[1][i], PlaneMeanError, PlaneGrossError, PlaneNormalError);
			//		if (finalPlaneCoors.rows < 10)
			//			return false;
			//		PlaneFit3DInfo studPlaneInfo = PlaneFit3D(finalPlaneCoors);
			//		//string strPlanePointPath = "PlanePoint.txt";
			//		//string strPlaneVectorPath = "PlaneVector.txt";
			//		//LaserSensorIO.WriteRowFirstAPP(strPlanePointPath, studPlaneInfo.Points.t());
			//		//LaserSensorIO.WriteRowFirstAPP(strPlaneVectorPath, studPlaneInfo.NormVector.t());
			//		CircleFit3DInfo studInfo = CircleFit3D(vecPoints[1][i]);
			//		Point3f studCenter = calcLinePlaneIntersection(studInfo.Center.t(), studPlaneInfo.NormVector.t(), planeInfo.Points.t(), planeInfo.NormVector.t());
			//		string strStudCenterPath = "StudCenter.txt";
			//		LaserSensorIO.WriteRowFirstAPP(strStudCenterPath, studCenter);
			//	}			
			//}

			//拟合螺柱
			if (vecPoints[1].empty())
				return false;
			vector<Mat> studFeature = StudFit2(vecPoints[1], planeInfo.Points.t(), planeInfo.NormVector, similarVecUsed);//180525原版
			//vector<Mat> studFeature = Reconstruction(vecPoints[1]);
			//vector<Mat> studFeature = StudFit14(vecPoints[1], planeInfo.Points.t());//180525原版

			string strStudPointPath = "StudPoint.txt";
			string strStudAxesPath = "StudAxes.txt";
			LaserSensorIO.WriteRowFirstAPP(strStudPointPath, studFeature[0]);
			LaserSensorIO.WriteRowFirstAPP(strStudAxesPath, studFeature[1]);

			//Mat dist = finalPlaneCoors.row(0) - studFeature[0];
			//Mat bestP = finalPlaneCoors.row(0);
			//double Mindist = sqrt(pow(dist.at<double>(0, 0), 2) + pow(dist.at<double>(0, 1), 2) + pow(dist.at<double>(0, 2), 2));
			//for (int i = 1; i != finalPlaneCoors.rows; ++i)
			//{
			//	dist = finalPlaneCoors.row(i) - studFeature[0];
			//	double temp = sqrt(pow(dist.at<double>(0, 0), 2) + pow(dist.at<double>(0, 1), 2) + pow(dist.at<double>(0, 2), 2));
			//	if (temp < Mindist)
			//	{
			//		bestP = finalPlaneCoors.row(i);
			//		Mindist = temp;
			//	}
			//}
			/*for (int i = 0; i != vecPoints[0].size(); ++i)
			{
				if (vecPoints[0][i].rows < 50)
					continue;
				double MeanX = 0;
				double MeanY = 0;
				double MeanZ = 0;
				for (int j = 0; j != vecPoints[0][i].rows; ++j)
				{
					MeanX += vecPoints[0][i].at<double>(i, 0);
					MeanY += vecPoints[0][i].at<double>(i, 1);
					MeanZ += vecPoints[0][i].at<double>(i, 2);
					dist = vecPoints[0][i].row(j) - studFeature[0];
					double temp = sqrt(pow(dist.at<double>(0, 0), 2) + pow(dist.at<double>(0, 1), 2) + pow(dist.at<double>(0, 2), 2));
					if (temp < Mindist)
					{
						bestP = vecPoints[0][i].row(j);
						Mindist = temp;
					}
				}
				Mat aPoint(1, 3, CV_64FC1, Scalar(0));
				aPoint.at<double>(0, 0) = MeanX / vecPoints[0][i].rows;
				aPoint.at<double>(0, 1) = MeanY / vecPoints[0][i].rows;
				aPoint.at<double>(0, 2) = MeanZ / vecPoints[0][i].rows;
				dist = aPoint - studFeature[0];
				double temp = sqrt(pow(dist.at<double>(0, 0), 2) + pow(dist.at<double>(0, 1), 2) + pow(dist.at<double>(0, 2), 2));
				if (temp < Mindist)
				{
					bestP = aPoint;
					Mindist = temp;
				}
			}*/

			//计算直线与平面的交点
			Point3f studCenter = calcLinePlaneIntersection(studFeature[0], studFeature[1], planeInfo.Points.t(), planeInfo.NormVector.t());
			//Point3f studCenter = calcLinePlaneIntersection(studFeature[0], studFeature[1], bestP, planeInfo.NormVector.t());
			string strStudCenterPath = "StudCenter.txt";
			LaserSensorIO.WriteRowFirstAPP(strStudCenterPath, studCenter);
		}
		break;
	}
	case 15:
	{
		for (int cnt = 0; cnt != 100; cnt++)
		{
			/*180516*/
			vector<Mat> GrayPics;
			vector<Point> CloudPoints;

			stringstream a;
			string strNum;
			a << cnt;
			a >> strNum;
			string strReadPath = "NormalPart8\\" + strNum;
			//string strReadPath = "NormalPart\\" + strNum + "\\Images\\";

			Mat CameraMatrix = LaserSensorIO.ReadCameraMatrix();
			Mat DistCoeffs = LaserSensorIO.ReadDistCoeffs();
			Mat LaserRotationPlaneParas = LaserSensorIO.ReadLaserRotatingPlanePara(23);

			vector<Mat> vecCoors(23);
			vector<Mat> vecCameraCoors;
			//vector<Mat> vecCameraCoors2;

			int maxHeight = 0;
			int maxWidth = 0;
			int indexW;
			int studLeft = 1279;
			int studRight = 0;

			//Mat planeCoors;
			int bestCircle;
			vector<Rect> ROIRect(23);
			for (int i = 0; i != 23; i++)
			{
				stringstream s;
				string StrFileNum;
				s << (i + 1);
				//s << i;//张祎文件读法
				s >> StrFileNum;
				string strCylinderPath = strReadPath + "\\" + StrFileNum + ".tiff";
				//string strCylinderPath = strReadPath + StrFileNum + ".tiff";
				Mat Image = imread(strCylinderPath);

				GrayPics.push_back(Image);//180516

				int left = studLeft;
				int right = studRight;
				int up = 0;
				int down = 959;

				vecCoors[i] = LaserSensorStudExtraction15(Image, 0, 959, 80, 1200, ROIRect[i]);
	
			}

			//for (int i = 1; i != 23; i++)
			//{
			//	if (abs(ROIRect[i].y - ROIRect[i - 1].y) < 10)
			//	{
			//		if (vecCoors[i].rows < vecCoors[i - 1].rows)
			//		{
			//			Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));
			//			vecCoors[i] = defaultAllPoints;
			//		}
			//		else
			//		{
			//			Mat defaultAllPoints(1, 2, CV_64FC1, cvScalar(-1));
			//			vecCoors[i - 1] = defaultAllPoints;
			//		}
			//	}
			//}
			for (int i = 0; i != 23; i++)
			{
				if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
				{
					vecCameraCoors.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
					Mat2Point(CloudPoints, vecCoors[i]);//180516
				}
			}

			/*180516*/
			Mat OneRGBPic = FindMax(GrayPics);
			for (vector<Point>::size_type RowIndex = 0; RowIndex < CloudPoints.size(); RowIndex++)
			{
				circle(OneRGBPic, CloudPoints.at(RowIndex), 4, Scalar(0, 0, 255), -1);
			}
			string strImagePath = "Totalimage\\" + strNum + ".tiff";
			imwrite(strImagePath, OneRGBPic);

			//if (vecCameraCoors.size() < 8)
			//{
			//	continue;
			//}
			if (vecCameraCoors.size() < 6)
			{
				continue;
			}

			vector<vector<Mat>> vecPoints = distLineAndStud15(vecCameraCoors);

			Mat planeCoors;
			for (int i = 0; i != vecPoints[0].size(); i++)
			{
				//if (planeCoorsVec[i].rows > 50)
				planeCoors.push_back(vecPoints[0][i]);
			}
			//存储拟合数据
			//string strPlanePointsPath = "planePoints.txt";
			//LaserSensorIO.WriteRowFirstAPP(strPlanePointsPath, planeCoors);

			//拟合平面
			if (planeCoors.empty())
				return false;
			Mat finalPlaneCoors = PlaneThreshod(planeCoors, PlaneMeanError, PlaneGrossError, PlaneNormalError);
			//string strResultPath2 = "finalPlaneCoors.txt";
			//LaserSensorIO.WriteRowFirstAPP(strResultPath2, finalPlaneCoors);
			if (finalPlaneCoors.rows < 10)
				return false;
			PlaneFit3DInfo planeInfo = PlaneFit3D(finalPlaneCoors);
			string strPlanePointPath = "PlanePoint.txt";
			string strPlaneVectorPath = "PlaneVector.txt";
			LaserSensorIO.WriteRowFirstAPP(strPlanePointPath, planeInfo.Points.t());
			LaserSensorIO.WriteRowFirstAPP(strPlaneVectorPath, planeInfo.NormVector.t());

			//vector<vector<Mat>> vecPoints2 = distLineAndStud(vecCameraCoors2, 5.0, planeInfo.NormVector, bestCircle);

			//拟合螺柱
			if (vecPoints[1].empty())
				return false;
			vector<Mat> studFeature = StudFit15(vecPoints[1], planeInfo.Points.t());//180525原版
			//vector<Mat> studFeature = Reconstruction(vecPoints[1]);

			string strStudPointPath = "StudPoint.txt";
			string strStudAxesPath = "StudAxes.txt";
			LaserSensorIO.WriteRowFirstAPP(strStudPointPath, studFeature[0]);
			LaserSensorIO.WriteRowFirstAPP(strStudAxesPath, studFeature[1]);

			//string strPlanePointPath = "PlanePoint.txt";
			//string strPlaneVectorPath = "PlaneVector.txt";
			//LaserSensorIO.WriteRowFirstAPP(strPlanePointPath, planeInfo.Points.t());
			//LaserSensorIO.WriteRowFirstAPP(strPlaneVectorPath, planeInfo.NormVector.t());

			//计算直线与平面的交点
			Point3f studCenter = calcLinePlaneIntersection(studFeature[0], studFeature[1], planeInfo.Points.t(), planeInfo.NormVector.t());
			//Point3f studCenter = calcLinePlaneIntersection(studFeature[0], studFeature[1], bestP, planeInfo.NormVector.t());
			string strStudCenterPath = "StudCenter.txt";
			LaserSensorIO.WriteRowFirstAPP(strStudCenterPath, studCenter);
		}
		break;
	}
	case 16://2016.03.12  使用distLineAndStud函数区分直线与螺柱，并且用最简单的HoughCircle直接进行提取点云拟合圆
	{
		for (int cnt = 0; cnt != 100; cnt++)
		{
			stringstream a;
			string strNum;
			a << cnt;
			a >> strNum;
			string strReadPath = "NormalPart3\\" + strNum;

			Mat CameraMatrix = LaserSensorIO.ReadCameraMatrix();
			Mat DistCoeffs = LaserSensorIO.ReadDistCoeffs();
			Mat LaserRotationPlaneParas = LaserSensorIO.ReadLaserRotatingPlanePara(23);

			vector<Mat> vecCoors(23);
			vector<Mat> vecCameraCoors;

			for (int i = 0; i != 23; i++)
			{
				stringstream s;
				string StrFileNum;
				s << (i + 1);
				//s << i;//张祎文件读法
				s >> StrFileNum;
				string strCylinderPath = strReadPath + "\\" + StrFileNum + ".tiff";
				//string strCylinderPath = strReadPath + StrFileNum + ".tiff";
				Mat Image = imread(strCylinderPath);

				string strStudPath = "studPoints.txt";
				vecCoors[i] = LaserSensorStudExtraction2(Image, 100, 900, 400, 900, 45, 400);

				//利用相机内参数和光平面标定参数将图像坐标值转换到相机坐标系下的坐标值
				if (!((vecCoors[i].at<double>(0, 0) < 0) || (vecCoors[i].at<double>(0, 1) < 0)))  //排除allPoints出现默认值的情况
				{
					vecCameraCoors.push_back(CameraCoorDatas(vecCoors[i], CameraMatrix, DistCoeffs, LaserRotationPlaneParas.row(i)));
				}
			}

			if (vecCameraCoors.size() < 8)
			{
				LaserSensorIO.WriteLog("Error:vecCameraCoors.size() < 8");
				return false;
			}

			vector<vector<Mat>> vecPoints = distLineAndStud(vecCameraCoors, 5.0);

			Mat planeCoors;
			for (int i = 0; i != vecPoints[0].size(); i++)
			{
				if (vecPoints[0][i].rows > 50)
					planeCoors.push_back(vecPoints[0][i]);
			}
			//存储拟合数据
			string strPlanePointsPath = "planePoints.txt";
			LaserSensorIO.WriteRowFirstAPP(strPlanePointsPath, planeCoors);

			//拟合平面
			if (planeCoors.empty())
				return false;
			Mat finalPlaneCoors = PlaneThreshod(planeCoors, PlaneMeanError, PlaneGrossError, PlaneNormalError);
			string strResultPath2 = "finalPlaneCoors.txt";
			LaserSensorIO.WriteRowFirstAPP(strResultPath2, finalPlaneCoors);
			if (finalPlaneCoors.rows < 10)
				return false;
			PlaneFit3DInfo planeInfo = PlaneFit3D(finalPlaneCoors);
			string strPlanePointPath = "PlanePoint.txt";
			string strPlaneVectorPath = "PlaneVector.txt";
			LaserSensorIO.WriteRowFirstAPP(strPlanePointPath, planeInfo.Points.t());
			LaserSensorIO.WriteRowFirstAPP(strPlaneVectorPath, planeInfo.NormVector.t());

			//拟合螺柱
			if (vecPoints[1].empty())
				return false;
			vector<Mat> studFeature = StudFit3(vecPoints[1], planeInfo.Points.t());
			string strStudPointPath = "StudPoint.txt";
			string strStudAxesPath = "StudAxes.txt";
			LaserSensorIO.WriteRowFirstAPP(strStudPointPath, studFeature[0]);
			LaserSensorIO.WriteRowFirstAPP(strStudAxesPath, studFeature[1]);

			//计算直线与平面的交点
			Point3f studCenter = calcLinePlaneIntersection(studFeature[0], studFeature[1], planeInfo.Points.t(), planeInfo.NormVector.t());
			string strStudCenterPath = "StudCenter.txt";
			LaserSensorIO.WriteRowFirstAPP(strStudCenterPath, studCenter);
		}
		break;
	}
	default://///////其它表明特征类型输入错误
		{
			LaserSensorIO.WriteLog("Error:特征类型错误");
			break;
		}

	}/////switch(m_type)
	return true;
}