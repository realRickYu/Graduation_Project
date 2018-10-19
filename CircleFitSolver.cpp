#include "circlefitsolver.h"
#include <cmath>

using namespace std;

/**
* 最小二乘法拟合圆
* 拟合出的圆以圆心坐标和半径的形式表示
* 此代码改编自 newsmth.net 的 jingxing 在 Graphics 版贴出的代码。
* 版权归 jingxing， 我只是搬运工外加一些简单的修改工作。
*/
bool circleLeastFit(const std::vector<PTS> &points, double &center_x, double &center_y, double &radius)
{
	center_x = 0.0f;
	center_y = 0.0f;
	radius = 0.0f;
	if (points.size() < 3)
	{
		return false;
	}

	double sum_x = 0.0f, sum_y = 0.0f;
	double sum_x2 = 0.0f, sum_y2 = 0.0f;
	double sum_x3 = 0.0f, sum_y3 = 0.0f;
	double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

	int N = points.size();
	for (int i = 0; i < N; i++)
	{
		double x = points[i].real();
		double y = points[i].imag();
		double x2 = x * x;
		double y2 = y * y;
		sum_x += x;
		sum_y += y;
		sum_x2 += x2;
		sum_y2 += y2;
		sum_x3 += x2 * x;
		sum_y3 += y2 * y;
		sum_xy += x * y;
		sum_x1y2 += x * y2;
		sum_x2y1 += x2 * y;
	}

	double C, D, E, G, H;
	double a, b, c;

	C = N * sum_x2 - sum_x * sum_x;
	D = N * sum_xy - sum_x * sum_y;
	E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

	center_x = a / (-2);
	center_y = b / (-2);
	radius = sqrt(a * a + b * b - 4 * c) / 2;
	return true;
}

double CircleFitSolver::L1_distance(const gsl_vector * v, void * params)
{
	vector<PTS> *vect = (vector<PTS> *)params;
	int N = vect->size();

	double a, b, r;
	a = gsl_vector_get(v, 0);
	b = gsl_vector_get(v, 1);
	r = gsl_vector_get(v, 2);

	double sum = 0;
	for (int i = 0; i < N; i++)
	{
		const PTS p = vect->at(i);
		double xi = p.real() - a;
		double yi = p.imag() - b;
		double dist = sqrt(xi * xi + yi * yi) - r;
		sum += fabs(dist);
	}
	return sum;
}


inline void CircleFitSolver::setStartPoint(double center_x, double center_y, double radius)
{
	gsl_vector_set(m_start_point, 0, center_x);
	gsl_vector_set(m_start_point, 1, center_y);
	gsl_vector_set(m_start_point, 2, radius);
}

bool CircleFitSolver::circleFitL1(const vector<PTS> &points, double &center_x, double &center_y, double &radius)
{
	m_function.params = (void *)&points;

	if (radius < 0)
	{
		// 用最小二乘拟合的结果作为初始值
		if (!circleLeastFit(points, center_x, center_y, radius))
		{
			return false;
		}
	}

	setStartPoint(center_x, center_y, radius);
	/* 经验值，初始步长设置为半径的十分之一 */
	gsl_vector_set(m_step_size, 0, radius / 10.0);
	gsl_vector_set(m_step_size, 1, radius / 10.0);
	gsl_vector_set(m_step_size, 2, radius / 10.0);

	gsl_multimin_fminimizer_set(m_fminimizer, &m_function, m_start_point, m_step_size);

	int iter = 0;
	int status;
	do
	{
		iter++;
		status = gsl_multimin_fminimizer_iterate(m_fminimizer);
		if (status == GSL_ENOPROG) // 表示无法找到更好的解了
		{
			break;
		}
		double size = gsl_multimin_fminimizer_size(m_fminimizer);
		status = gsl_multimin_test_size(size, 1e-2);
	} while (status == GSL_CONTINUE && iter < m_max_iter);

	gsl_vector * out = gsl_multimin_fminimizer_x(m_fminimizer);

	center_x = gsl_vector_get(out, 0);
	center_y = gsl_vector_get(out, 1);
	radius = gsl_vector_get(out, 2);

	return true;
}

CircleFitSolver::CircleFitSolver()
{
	m_max_iter = 100; // 默认最大迭代 100 步

	m_function.n = 3;
	m_function.f = L1_distance;

	m_start_point = gsl_vector_alloc(m_function.n);
	m_step_size = gsl_vector_alloc(m_function.n);

	m_fminimizer = gsl_multimin_fminimizer_alloc(gsl_multimin_fminimizer_nmsimplex, 3);
}

CircleFitSolver::~CircleFitSolver()
{
	gsl_vector_free(m_start_point);
	gsl_vector_free(m_step_size);

	gsl_multimin_fminimizer_free(m_fminimizer);
}