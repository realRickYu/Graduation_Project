/*! 
 *	\section About ����LaserSensor3D�Ӿ�������������ȡ���㷨��
 *
 *  \section Announce 
 ����˵��:
 0 ȫ����������    1 �б�     2 ���     3 �ޱ�    4 ���ۿ�  5 ����
 6 �����    7 ��   8 ����   9 ��ĸ��   10 ����
 *  
 *  \section Log ��־

 *\n 20150704 V20.0 
       1. �޸�������������е�blacklevelraw�����ã���ǿ������⵶�ߵĶԱȶ�;
	   2. ����Բ�ס����ס����ۿ׵ı߽����ȡ�㷨�У���������ȡ��������ȡ�����⵶��ǿ���ʽ��������Ե�����
	   ����Ȥ����ı߽�����޸�Ϊ200Pixel����ԼΪ6mm�ľ��룬�Ӷ���֤����ȡ������������õĹ⵶�㣻
	   3. ����Hough4Lines���������ά����Ҫ�޸�Ϊ 22*22*102��
	   4. Ѱ�ҷ��������4��ֱ�ߣ�houghLinesѰ�Ҿۼ���������1���������Ӷ����̶ȱ�֤���ҵ������4��ֱ�ߣ�
	   5. ��ĸ��������ӣ������뵥����������ȡ�����Ӷ���ȥ���쳣���ͬʱ�϶�ı�����Ϲ⵶����Ŀ,�Ա��޷���ȡ��Ϲ�׵Ĺ⵶��Ŀʱ�޸ĸò���;


 *\n  20150624 V19.0 �޸�LaserScanSensor3DFeatrueExtraction�еĲ��� ����TeachParameters����
       1. �޸ķ��׵�����ȡ���������ߵ��ƾ�����ȡ�����������ҽ�������˺����޸�Ϊ10*10
	   2. ���ٷ��׾����ά������׼ȷ����ȡֱ�ߣ�ͬʱ�޸�ֱ����ȡ�ı�׼(0.05�����)���޸����������ֱ���������˳�����
	      �޸�Ѱ������HoughLineֱ����Χ������
	   3. �����������ά����һ����С��22*22*1002����HoughLine�����ϵ������Ϊ1.96,ͬʱ�����������޶���Χ�������
	   ȡǰ4��������Ϊ �������,3DFitLine�����ϵ������Ϊ3.01 
	   4. ֱ�߷������ĵ�λ���������Լ���ֱ�ߵĽǶ���������5��֮��
	   5. �����������һ����С��22*22*602��
	   6. ���ڹ⵶����ȡ�����׷���������ϣ�����ɷ��ױ߽���ƽ������󣬽�������������һϵ�����⣬
	   �ʽ����Բ����ñ��������

 *\n  20150616 V18.0 ���HoughLines���������޸���squarefit����,���޸�HoughCircels��r�ķ�Χ,�޸�RotToZ����(VT==Z)
       2.\\\  ע��CameraCoorDatas()�����ж�����ж�,ע�� PlaneThreshod()�����ж����Ĭ��ֵ�ж�;Hole3DFit �����Ĭ��ֵ�ж�;
	   Line3DThreshold �����Ĭ���ж�
	   3.\\\\\ ���������� 0/2/3/4/5/6/7������˴�����־���;
	   4.\\\\\ ����������2����ǰ������ͼƬ���������Ʊ�֤�ظ�����
       5.\\\\\ �޸� RangeImageLineProjection()����
	   6.\\\\\ �޸��� SphereFit()���������ɵ�����ϵ���ʽ;
	   7.\\\\\ �޸���Opencv��ȡ�������㷨(SVD--LU)  LSSphereFit��LSCircleFit2D��Hough4Lines�е���С���˷�
	   8.\\\\\\ �޸��� EliminateGrossError()��������ȥ��������쳣��Ĳ���
       9.\\\\\Hough4Lines �����ͷ�HoughSpace���ڴ�
	   10.\\\\\\  �޸�EliminateGrossError()�����еĲ��������������۱ߵ�������ȥ������Ĺ⵶��Ӷ�������Ŵ�
	   11.\\\\\\  �޸ĳ�����=cv::sqrt()�������ú���û�з���ֵ���޸�Ϊ=std::sqrt()����ֵ

 *\n  201506012 V17.0 
       Բ�ױ�Ե��̫��������������ȥ��(�ϵ�1mm,ԼΪ30������)
       �޸���ƽ�桢Բ���ж�EliminateGrossError����������ȥ���ĵ��ƹ��࣬
      ���޸���max(10,Size/2)����Hough3Circle�������޸���ƽ�桢Բ���ж�EliminateGrossError����������ȥ���ĵ��ƹ��࣬���޸���max(10,Size/2)����Hough3Circle������

 *  \n 20150603 V16.0 ����쳣���޳����㷨,���޸���Ӧ���㷨,���C++����ϵĺ���,���Hough3Circle����,���޸���Hole3DFit�������ͺ���ʵ��

 *  \n 20150525 V15.0 �޸���һЩ������ Laser1StripeExtraction ��LaserStripeExtraction�����жԲ�ɫͼƬ���в������Ѿ��޸Ĵ���ı���ΪImg����ImgGray���в�����
 �޸���Laser1StripeExtraction ������δ�����ʼ�ĸ���Ȥ������ɼ���⵶���Ĵ�������۱���������
 *  \n 20150520 V15.0  �ޱ߲���ֱ�������ȡ�߽�㡢�ռ�Բ�Ȳ���ƽ������ٲ��ÿռ�Բ���ȥ�������㣬�����Ͼ���
    ������ۿ�������ȡ�㷨����  ��ӷ���������ȡ�㷨���� ��������������ȡ���� �������������ϲ���
 *  \n 20150516 V14.0  �޸�ʾ�̲�����Χ������;�����ޱ�������ȡ����(3);
 *  \n 20150515 V13.0  �������ȡ����֮ǰ�Ƚ��б�������������ƽ����ϵ�ɸѡ������ɸѡ������ƽ�����ϵ� 
 *  \n 201500514 V12.0  �޸������Ӳ����.h�ļ�������������ȡ.cpp�е�������Ӧ�޸�
 *  \n 20150513 V11.0 �޸��˿ռ�Բ������ͶӰ�ı�����������ȡ�⵶��ȫ�����Ƶĳ���
     �����޸��˲�����������ȡ��������Ĭ��ֵ ��������ƵĲ���
 *  \n 20150513 V10.0  �޸��� �ռ�Բ���ļ���ı��� R'*2Center
 *  \n 20150511 V9.0 �޸��˸���Ȥ���򳬳�ͼƬ���޵�bug�������ȫ������ȡ����
 *  \n 20150507 V8.0 ������յ�������������---Բ��
 *  \n 20150506 V7.0 ���ֱ����ϡ��ռ�Բ��ϡ�ƽ����ϡ�����ϵȻ����������
 *  \n 20150505 V6.0 �޸�����������ڲ����ͻ���ϵ���Լ���ƽ���������Բ�ױ߽���������ϵ����ֵ����ȡ��ÿ��ͼƬ����һ������ϵ��ת������
 *  \n 20150504 V5.0 �������������ڲ����ͻ���ϵ���Լ���ƽ���������Բ�ױ߽���������ϵ����ֵ����ȡ
 *  \n 20140427 V4.0 ���Class TeachParameters��Ͷ�ȡʾ�̲����ĳ���,ʾ�̲���������Ϊ���ͱ���
 *  \n 20140424 V0.3 ���д��־�Ĺ��� WriteLog()��Ա����
 *  \n 20140424 V0.2 �޸ĺ������� Append��Truncate ���������Ա����
 *  \n 20150424 v0.1 ������C3DLaserSensorRSerialIO�࣬�����6����Ա�����ı�д
 */
#pragma once
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "C3DLaserSensorRSerialIOV11.0.h"
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;


#pragma region /////////�����������Ľṹ�� 

struct PlaneFit3DInfo //////ƽ�����
{
	Mat NormVector;
	Mat Points;
	Mat PlaneFitError;
};


////////////////// �ռ�Բ���
struct LSCircleFitInfo /////��С���˷����Բ����Ϣ
{
	Mat Par;
};

struct CircleFitGradientInfo////Բ���ݶ���Ϣ
{
	Mat J;
	Mat g;
	double F;
};

struct LMCircleFitInfo
{
	Mat Par;/// �洢Բ�ĵ�����ֵ�Ͱ뾶
	int Iter;///// �洢�����Ĵ���
};
struct PProjectToPlaneInfo////�ռ��ͶӰ��ƽ���ϵ���Ϣ
{
	Mat ProjectedXY;
	Mat VT;
};
struct CircleFit3DInfo////�ռ�Բ��ϵ���Ϣ
{
	Mat Center;
	double Radius;
	Mat V;
	Mat CircleFit3DError;
};

struct RotToZInfo///ƽ�淨������Z��ƽ�е���Ϣ
{
	Mat XYRoted;///ͶӰ�������
	Mat R;//////// ��ת����
};
//////
 


#pragma endregion

class C3DLaserSensorFeatureExtraction
{
public:
	C3DLaserSensorFeatureExtraction(void);
	~C3DLaserSensorFeatureExtraction(void);

    C3DLaserSensorRSerialIO  LaserSensorIO;
	Mat m_BaseEye;////��������ϵ�任�Ĳ���
	Point3f m_SphereCenter;////// ������������



#pragma region ////// ����������Ҫ���õĺ���
  
	bool LaserScanSensor3DFeatrueExtraction();

#pragma endregion 


#pragma region /////ͨ�ô����㷨

#pragma region ///// ������ȡ������ʵ��
	//////////////////////������ȡ��������ֵ

	Mat LaserSensorPointCloudsExtraction( Mat Img, int Threshold=150, int ArcLengthValue=50, Size KernelSize=Size(5,5));////// ���ر���������ȫ����������

	vector<Mat> LaserSensorCylinderExtraction( Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, 
		int Threshold=150, int ArcLengthValue=50, Size KernelSize=Size(5,5));////// ���������������ĵ�������
	
	//Mat LaserSensorStudExtraction4(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit,
	//	bool studFlag, int widthValue = 0, Size KernelSize = Size(5, 5));////// ���������������ĵ�������
	Mat LaserSensorStudExtraction(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, 
		bool studFlag, int widthValue = 0, Size KernelSize = Size(5, 5));////// ���������������ĵ�������
	Mat LaserSensorStudExtraction3(Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit,
		int Threshold = 150, int ArcLengthValue = 50, Size KernelSize = Size(5, 5));////// ���������������ĵ������ݣ�����������Ż���Ч��һ��
	Mat LaserSensorStudExtraction2( Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, 
		int Threshold=150, int ArcLengthValue=50, Size KernelSize=Size(5,5));////// ���������������ĵ�������
	Mat LaserSensorStudExtraction2(string strImagePath, Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit,
		int ArcLengthValue = 50, Size KernelSize = Size(5, 5));////// ���������������ĵ�������
	Mat LaserSensorStudExtraction( Mat Img, int uplimit, int downlimint, int leftlimit, int rightlimit, 
		int Threshold=150, int ArcLengthValue=50, Size KernelSize=Size(5,5));////// ���������������ĵ�������
	vector<Mat>  StudFit(Mat FittingPoints, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);///////������ܷ�����������ֵ+����
	vector<Mat>  StudFit(vector<Mat> vecFittingPoints, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);///////������ܷ�����������ֵ+����
	vector<Mat>  StudFit2(vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);///////������ܷ�����������ֵ+����
	vector<Mat>  StudFit2(int num, vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);///////������ܷ�����������ֵ+����
    vector<Mat>  StudFit3(vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//���淵����������ֵ+����
	vector<Mat>  StudFit3(int num, vector<Mat> vecFittingPoints, Mat planePoint, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//���淵����������ֵ+����

#pragma endregion 

#pragma region //////////////////////// �⵶������ȡ����Ļ�������
	//////����Ӧ�⵶������ȡ����
	Mat LaserStripeExtraction(Mat ImgGray,int Threshold=150,int ArcLengthValue=50,Size KernelSize=Size(5,5));


	///////����ȡ�����������ܳ��ɴ�С����
	static bool LengthDown(vector<Point> elem1,vector<Point> elem2);

	//////////////��������Ȥ����
	Rect ModifyROI(Mat ImgGray,Rect OriginalROI,int Threshold=150);


	////////////��չ����Ȥ����,�ڲ�����ͼƬ�߽�������
	/////////// ����������չһ��������(���·ֱ���չ10�����ؿ϶�����)
	Rect ExtendedROI(Mat ImgGray,Rect OutRect,int PixelNum=10);

	//////////// ȷ����ʼ�⵶�ߵ�����λ��
	Mat OriginalCenter(Mat LaserStripSmoothed);

	/////////// ȷ���⵶�ߵı߽�Ҷ���ֵ��ȷ���⵶�ߵĿ��
	Mat DetermineEdgeValue(Mat LaserStripSmoothed,Mat Index);

	////////////
	Mat DetermineCenters(Mat LaserStripSmoothed,Mat Index, Mat ThresholdValue,Rect ROIRect);

	Mat CameraCoorDatas(Mat ImageDatas, Mat CameraMatrix, Mat DistCoeffs, Mat LaserPlaneParas);///////��������ڲ����͹�ƽ���������3ά�ؽ�

	//Mat TwoPointsCameraCoorDatas(Mat ImageDatas, Mat CameraMatrix, Mat DistCoeffs, Mat ReadLaserRotatingPlaneParas);///////��������ڲ����͹�ƽ���������3ά�ؽ�

	void C3DLaserSensorFeatureExtraction::deleteRow(Mat& sample, int rowNum);
#pragma endregion

#pragma  region ///////// �����������Ļ�����

	PlaneFit3DInfo PlaneFit3D(Mat X,int flag=0);/////// ƽ�����

	/////////////

	/////////// �ռ�Բ���
	LSCircleFitInfo LSCircleFit2D(Mat X);
	CircleFitGradientInfo CircleCurrentIteration(Mat Par,Mat X);
	LMCircleFitInfo LMCircleFit2D(Mat X,int MaxIter=100,double LambdaIni=1);
	PProjectToPlaneInfo PProjectToPlane(Mat XY);
	RotToZInfo RotToZ(Mat XY,Mat VT);
	Mat Angvec2r(double theta,Mat k);

	/////////// moidified in 20150603
	double EliminateGrossError(Mat Error,double MeanErrorThreshold=0.2,double GrossErrorCoe=0.5,double NormalErrorCoe=3);           //////// �޳��ִ����
	Mat PlaneThreshod(Mat PlaneFitData, double PlaneMeanErrorThreshold=0.1,double PlaneGrossErrorCoe=0.1,double PlaneNormalErrorCoe=3); 
	CircleFit3DInfo CircleFit3D(Mat X,double CircleMeanErrorThreshold=0.2,double CircleGrossErrorCoe=0.1,double CircleNormalErrorCoe=3); 

	static bool PointValueAsc(Point2d pt1,Point2d pt2);
    Mat HoughCircle(Mat Camera3DData, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold=0.2,double CircleGrossErrorCoe=0.1,double CircleNormalErrorCoe=3);//��׼HoughԲ��������쳣��
    Mat HoughCircle(vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold=0.2,double CircleGrossErrorCoe=0.1,double CircleNormalErrorCoe=3);//��׼HoughԲ��������쳣��
	Mat HoughCircle(Mat axesMat, int num, vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//��׼HoughԲ��������쳣��
	Mat HoughCircle2(vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//��׼HoughԲ��������쳣�� //180525 ȡƱ������ǰN���ĵ�
	Mat HoughCircle2(Mat axesMat, int num, vector<Mat> Camera3DData, vector<Mat> vecXYRoted, Mat XYRoted, double min_r, double max_r, double step_r = 0.1, double step_theta = 0.01, double CircleMeanErrorThreshold = 0.2, double CircleGrossErrorCoe = 0.1, double CircleNormalErrorCoe = 3);//��׼HoughԲ��������쳣�� //180525 ȡƱ������ǰN���ĵ�

	Point3d calcPCA(Mat Camera3DData);//��ά����PCA�㷨��ȡ��������

	Point3f calcLinePlaneIntersection(Mat linePoint, Mat lineVector, Mat planePoint, Mat planeVector);//��������������ƽ��Ľ���
	vector<vector<Mat>> distLineAndStud(vector<Mat> vecCameraCoors, double r);//���������������ϵ�µ�������ƽ�����
	Mat Kasa(Mat fittingPoints, Point3d axesMat);//��С�������Բ,����Բ������
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


