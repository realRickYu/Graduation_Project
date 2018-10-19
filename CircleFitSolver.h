#ifndef CIRCLEFITSOLVER_H
#define CIRCLEFITSOLVER_H

#include <complex>
#include <vector>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>

using namespace std;

typedef complex<double> PTS;

bool circleLeastFit(const vector<PTS> &points, double &center_x, double &center_y, double &radius);

class CircleFitSolver
{
public:
	CircleFitSolver();
	~CircleFitSolver();
	void setMaxIter(int iter) { m_max_iter = iter; }

	/**
	* @brief circleFitL1  ���Բ������о�Ϊ���ݵ㵽���Բ�ľ������ֵ֮����С��
	* @param points ����������洢�������ݵ㡣
	* @param center_x radius > 0 ʱ��Ϊ�����㷨�ĳ�ʼֵ��������ɺ󷵻����Բ��Բ�� X ����
	* @param center_y radius > 0 ʱ��Ϊ�����㷨�ĳ�ʼֵ��������ɺ󷵻����Բ��Բ�� Y ����
	* @param radius   radius < 0 ʱ������С������ϵĽ����Ϊ�����㷨�ĳ�ʼֵ��������ɺ󷵻����Բ�İ뾶��
	* @return true ��ʾ��ϳɹ����������ʧ�ܡ�
	*/
	bool circleFitL1(const vector<PTS> &points, double &center_x, double &center_y, double &radius);
private:
	gsl_multimin_function m_function;
	gsl_multimin_fminimizer * m_fminimizer;

	int m_max_iter; // �����㷨������������

	gsl_vector *m_start_point; // �����㷨�ĳ�ʼֵ
	gsl_vector *m_step_size; // �����㷨�ĳ�ʼ����

	void setStartPoint(double center_x, double center_y, double radius);

	static double L1_distance(const gsl_vector * v, void * params);

};

#endif // CIRCLEFITSOLVER_H