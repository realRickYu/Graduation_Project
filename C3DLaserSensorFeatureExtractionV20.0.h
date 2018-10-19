/*! 
 *	\section About 关于LaserSensor3D视觉传感器特征提取的算法类
 *
 *  \section Announce 
 类型说明:
 0 全部点云数据    1 切边     2 光孔     3 修边    4 腰槽孔  5 方孔
 6 曲面点    7 球   8 光柱   9 螺母孔   10 螺柱
 *  
 *  \section Log 日志

 *\n 20150704 V20.0 
       1. 修改相机参数设置中的blacklevelraw的设置，增强背景与光刀线的对比度;
	   2. 对于圆孔、方孔、腰槽孔的边界点提取算法中，由于能提取错误点比提取不到光刀点强，故将最长轮廓边缘点距离
	   感兴趣区域的边界距离修改为200Pixel，大约为6mm的距离，从而保证能提取到曲线拟合所用的光刀点；
	   3. 方孔Hough4Lines参数矩阵的维数需要修改为 22*22*102，
	   4. 寻找方孔所需的4条直线，houghLines寻找聚集个数大于1的索引，从而最大程度保证能找到所需的4条直线；
	   5. 螺母孔情况复杂，设置与单独的特征提取参数从而在去除异常点的同时较多的保留拟合光刀点数目,以便无法获取拟合光孔的光刀数目时修改该参数;


 *\n  20150624 V19.0 修改LaserScanSensor3DFeatrueExtraction中的参数 传入TeachParameters参数
       1. 修改方孔点云提取，左右两边点云均能提取才输出结果，且将闭运算核函数修改为10*10
	   2. 减少方孔矩阵的维数便于准确的提取直线，同时修改直线提取的标准(0.05的误差)，修改提出到所需直线条数的退出条件
	      修改寻找满足HoughLine直线误差范围的条件
	   3. 将参数矩阵的维数进一步缩小到22*22*1002，将HoughLine的误差系数设置为1.96,同时在误差均满足限定范围的情况，
	   取前4个点云作为 拟合数据,3DFitLine的误差系数设置为3.01 
	   4. 直线法向量的单位化操作，以及将直线的角度误差控制在5°之内
	   5. 将参数矩阵进一步缩小到22*22*602，
	   6. 由于光刀点提取到方孔反射的轮廓上，会造成方孔边界点的平面度误差大，进而导致上述的一系列问题，
	   故将尝试不采用闭运算操作

 *\n  20150616 V18.0 添加HoughLines函数，并修改了squarefit函数,并修改HoughCircels中r的范围,修改RotToZ函数(VT==Z)
       2.\\\  注释CameraCoorDatas()函数中多余的判断,注释 PlaneThreshod()函数中多余的默认值判断;Hole3DFit 多余的默认值判断;
	   Line3DThreshold 多余的默认判断
	   3.\\\\\ 在特征代号 0/2/3/4/5/6/7中添加了错误日志输出;
	   4.\\\\\ 在特征代号2处，前后两张图片均加以限制保证重复精度
       5.\\\\\ 修改 RangeImageLineProjection()函数
	   6.\\\\\ 修改了 SphereFit()函数，做成迭代拟合的形式;
	   7.\\\\\ 修改了Opencv求取逆矩阵的算法(SVD--LU)  LSSphereFit、LSCircleFit2D、Hough4Lines中的最小二乘法
	   8.\\\\\\ 修改了 EliminateGrossError()函数避免去除过多非异常点的操作
       9.\\\\\Hough4Lines 忘记释放HoughSpace的内存
	   10.\\\\\\  修改EliminateGrossError()函数中的参数，避免在求折边等特征中去除过多的光刀点从而造成误差被放大
	   11.\\\\\\  修改程序中=cv::sqrt()函数，该函数没有返回值，修改为=std::sqrt()返回值

 *\n  201506012 V17.0 
       圆孔边缘点太近，当成噪声点去除(断点1mm,约为30个像素)
       修改了平面、圆的判断EliminateGrossError函数，避免去除的点云过多，
      并修改了max(10,Size/2)进行Hough3Circle的运算修改了平面、圆的判断EliminateGrossError函数，避免去除的点云过多，并修改了max(10,Size/2)进行Hough3Circle的运算

 *  \n 20150603 V16.0 添加异常点剔除的算法,并修改相应的算法,添加C++求组合的函数,添加Hough3Circle函数,并修改了Hole3DFit的声明和函数实现

 *  \n 20150525 V15.0 修改了一些笔误（如 Laser1StripeExtraction 和LaserStripeExtraction函数中对彩色图片进行操作，已经修改传入的变量为Img，对ImgGray进行操作）
 修改了Laser1StripeExtraction 函数中未传入初始的感兴趣区域造成计算光刀中心错误；添加折边特征操作
 *  \n 20150520 V15.0  修边采用直线拟合求取边界点、空间圆先采用平面拟合再采用空间圆拟合去掉噪声点，提高拟合精度
    添加腰槽孔特征提取算法操作  添加方孔特征提取算法操作 添加曲面点特征提取操作 添加球面特征拟合操作
 *  \n 20150516 V14.0  修改示教参数范围的限制;增加修边特征提取操作(3);
 *  \n 20150515 V13.0  添加在提取轮廓之前先进行闭运算操作，添加平面拟合的筛选操作，筛选不满足平面的拟合点 
 *  \n 201500514 V12.0  修改了相机硬触发.h文件，并在特征提取.cpp中的做了相应修改
 *  \n 20150513 V11.0 修改了空间圆拟合最后反投影的笔误，增加了提取光刀线全部点云的程序，
     另外修改了不满足特征提取条件返回默认值 不输出点云的操作
 *  \n 20150513 V10.0  修改了 空间圆中心计算的笔误 R'*2Center
 *  \n 20150511 V9.0 修改了感兴趣区域超出图片界限的bug，并添加全点云提取程序
 *  \n 20150507 V8.0 添加最终的特征测量函数---圆孔
 *  \n 20150506 V7.0 添加直线拟合、空间圆拟合、平面拟合、球拟合等基本特征拟合
 *  \n 20150505 V6.0 修改了利用相机内参数和畸变系数以及光平面参数进行圆孔边界点相机坐标系坐标值的求取，每张图片进行一次坐标系的转换操作
 *  \n 20150504 V5.0 添加了利用相机内参数和畸变系数以及光平面参数进行圆孔边界点相机坐标系坐标值的求取
 *  \n 20140427 V4.0 添加Class TeachParameters类和读取示教参数的程序,示教参数的输入为车型编码
 *  \n 20140424 V0.3 添加写日志的功能 WriteLog()成员函数
 *  \n 20140424 V0.2 修改和增加了 Append和Truncate 两个输出成员变量
 *  \n 20150424 v0.1 定义了C3DLaserSensorRSerialIO类，并完成6个成员函数的编写
 */
#pragma once
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "C3DLaserSensorRSerialIOV11.0.h"
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;


#pragma region /////////特征拟合所需的结构体 

struct PlaneFit3DInfo //////平面拟合
{
	Mat NormVector;
	Mat Points;
	Mat PlaneFitError;
};


////////////////// 空间圆拟合
struct LSCircleFitInfo /////最小二乘法拟合圆的信息
{
	Mat Par;
};

struct CircleFitGradientInfo////圆的梯度信息
{
	Mat J;
	Mat g;
	double F;
};

struct LMCircleFitInfo
{
	Mat Par;/// 存储圆心的坐标值和半径
	int Iter;///// 存储迭代的次数
};
struct PProjectToPlaneInfo////空间点投影到平面上的信息
{
	Mat ProjectedXY;
	Mat VT;
};
struct CircleFit3DInfo////空间圆拟合的信息
{
	Mat Center;
	double Radius;
	Mat V;
	Mat CircleFit3DError;
};

struct RotToZInfo///平面法向量与Z轴平行的信息
{
	Mat XYRoted;///投影后的数据
	Mat R;//////// 旋转矩阵
};
//////
 


#pragma endregion

class C3DLaserSensorFeatureExtraction
{
public:
	C3DLaserSensorFeatureExtraction(void);
	~C3DLaserSensorFeatureExtraction(void);

    C3DLaserSensorRSerialIO  LaserSensorIO;
	Mat m_BaseEye;////用于坐标系变换的参数
	Point3f m_SphereCenter;////// 保存球心数据



#pragma region ////// 程序最终需要调用的函数
  
	bool LaserScanSensor3DFeatrueExtraction();

#pragma endregion 


#pragma region /////通用处理算法

#pragma region ///// 特征提取函数的实现
	//////////////////////两步提取被测特征值

	Mat LaserSensorPointCloudsExtraction( Mat Img, int Threshold=150, int ArcLengthValue=50, Size KernelSize=Size(5,5));////// 返回被测特征的全部点云数据

	vector<Mat> LaserSensorCylinderExtraction( Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, 
		int Threshold=150, int ArcLengthValue=50, Size KernelSize=Size(5,5));////// 返回柱面拟合所需的点云数据
	
	//Mat LaserSensorStudExtraction4(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit,
	//	bool studFlag, int widthValue = 0, Size KernelSize = Size(5, 5));////// 返回柱面拟合所需的点云数据
	Mat LaserSensorStudExtraction(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, 
		bool studFlag, int widthValue = 0, Size KernelSize = Size(5, 5));////// 返回柱面拟合所需的点云数据
	Mat LaserSensorStudExtraction3(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit,
		int Threshold = 150, int ArcLengthValue = 50, Size KernelSize = Size(5, 5));////// 返回柱面拟合所需的点云数据，针对螺牙的优化，效果一般
	Mat LaserSensorStudExtraction2( Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, 
		int Threshold=150, int ArcLengthValue=50, Size KernelSize=Size(5,5));////// 返回柱面拟合所需的点云数据
	Mat LaserSensorStudExtraction2(string strImagePath, Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit,
		int ArcLengthValue = 50, Size KernelSize = Size(5, 5));////// 返回柱面拟合所需的点云数据
	Mat LaserSensorStudExtraction( Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, 
		int Threshold=150, int ArcLengthValue=50, Size KernelSize=Size(5,5));////// 返回柱面拟合所需的点云数据
	vector<Mat>  StudFit(Mat FittingPoints, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);///////柱面可能返回中心坐标值+法线
	vector<Mat>  StudFit(vector<Mat> vecFittingPoints, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);///////柱面可能返回中心坐标值+法线
	vector<Mat>  StudFit2(vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);///////柱面可能返回中心坐标值+法线
	vector<Mat>  StudFit2(int num, vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);///////柱面可能返回中心坐标值+法线
    vector<Mat>  StudFit3(vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//柱面返回中心坐标值+法线
	vector<Mat>  StudFit3(int num, vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//柱面返回中心坐标值+法线

#pragma endregion 

#pragma region //////////////////////// 光刀中心提取所需的基本函数
	//////自适应光刀中心提取函数
	Mat LaserStripeExtraction(Mat ImgGray,int Threshold=150,int ArcLengthValue=50,Size KernelSize=Size(5,5));


	///////将提取的轮廓按照周长由大到小排序
	static bool LengthDown(vector<Point> elem1,vector<Point> elem2);

	//////////////修正感兴趣区域
	Rect ModifyROI(Mat ImgGray,Rect OriginalROI,int Threshold=150);


	////////////扩展感兴趣区域,在不超过图片边界的情况下
	/////////// 向上向下扩展一定的像素(上下分别扩展10个像素肯定够了)
	Rect ExtendedROI(Mat ImgGray,Rect OutRect,int PixelNum=10);

	//////////// 确定初始光刀线的中心位置
	Mat OriginalCenter(Mat LaserStripSmoothed);

	/////////// 确定光刀线的边界灰度阈值，确定光刀线的宽度
	Mat DetermineEdgeValue(Mat LaserStripSmoothed,Mat Index);

	////////////
	Mat DetermineCenters(Mat LaserStripSmoothed,Mat Index, Mat ThresholdValue,Rect ROIRect);

	Mat CameraCoorDatas(Mat ImageDatas, Mat CameraMatrix, Mat DistCoeffs, Mat LaserPlaneParas);///////利用相机内参数和光平面参数进行3维重建

	//Mat TwoPointsCameraCoorDatas(Mat ImageDatas, Mat CameraMatrix, Mat DistCoeffs, Mat ReadLaserRotatingPlaneParas);///////利用相机内参数和光平面参数进行3维重建

	void C3DLaserSensorFeatureExtraction::deleteRow(Mat& sample, int rowNum);
#pragma endregion

#pragma  region ///////// 特征拟合所需的基本类

	PlaneFit3DInfo PlaneFit3D(Mat X,int flag=0);/////// 平面拟合

	/////////////

	/////////// 空间圆拟合
	LSCircleFitInfo LSCircleFit2D(Mat X);
	CircleFitGradientInfo CircleCurrentIteration(Mat Par,Mat X);
	LMCircleFitInfo LMCircleFit2D(Mat X,int MaxIter=100,double LambdaIni=1);
	PProjectToPlaneInfo PProjectToPlane(Mat XY);
	RotToZInfo RotToZ(Mat XY,Mat VT);
	Mat Angvec2r(double theta,Mat k);

	/////////// moidified in 20150603
	double EliminateGrossError(Mat Error,double MeanErrorThreshold=0.2,double GrossErrorCoe=0.5,double NormalErrorCoe=3);           //////// 剔除粗大误差
	Mat PlaneThreshod(Mat PlaneFitData, double PlaneMeanErrorThreshold=0.1,double PlaneGrossErrorCoe=0.1,double PlaneNormalErrorCoe=3); 
	CircleFit3DInfo CircleFit3D(Mat X,double CircleMeanErrorThreshold=0.2,double CircleGrossErrorCoe=0.1,double CircleNormalErrorCoe=3); 

	static bool PointValueAsc(Point2d pt1,Point2d pt2);
    Mat HoughCircle(Mat Camera3DData, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold=0.2,double CircleGrossErrorCoe=0.1,double CircleNormalErrorCoe=3);//标准Hough圆检测消除异常点
    Mat HoughCircle(vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold=0.2,double CircleGrossErrorCoe=0.1,double CircleNormalErrorCoe=3);//标准Hough圆检测消除异常点
	Mat HoughCircle(Mat axesMat, int num, vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//标准Hough圆检测消除异常点
	Mat HoughCircle2(vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//标准Hough圆检测消除异常点 //180525 取票数排在前N名的点
	Mat HoughCircle2(Mat axesMat, int num, vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//标准Hough圆检测消除异常点 //180525 取票数排在前N名的点

	Point3d calcPCA(Mat Camera3DData);//三维数据PCA算法求取柱体轴线

	Point3f calcLinePlaneIntersection(Mat linePoint, Mat lineVector, Mat planePoint, Mat planeVector);//计算螺柱轴线与平面的交点
	vector<vector<Mat>> distLineAndStud(vector<Mat> vecCameraCoors, double r);//用于区分相机坐标系下的螺柱与平面点云
	Mat Kasa(Mat fittingPoints, Point3d axesMat);//最小二乘拟合圆,返回圆心坐标
	void C3DLaserSensorFeatureExtraction::Mat2Point(vector<Point> &CloudPoints,Mat ImageCoor);
	Mat FindMax(vector<Mat> GrayPics);
	Rect ExtendedOrignalROI(Rect OriginalRect,int VExtendValue,int HExtendValue,int HightValue,int WidthValue);
#pragma endregion

#pragma endregion

	int GetIterativeBestThreshold(MatND HistGram);
	int GetOSTUThreshold(MatND HistGram);

	Rect getRect(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit);
	//Rect getFinalRect(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, bool studFlag,
	//	 int centre=0, int widthValue=0);
	vector<vector<Mat>> distLineAndStud(vector<Mat> vecCameraCoors, double r, Mat planeVector, int bestCircle);
};


