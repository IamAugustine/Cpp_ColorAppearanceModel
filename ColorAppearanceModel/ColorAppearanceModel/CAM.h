#pragma once
#include "Vars.h"
#include <math.h>
constexpr auto PI = 3.1415927;;
inline double DegreeToRadian(double x)
{
	return x/180.0 * PI;
}
inline double RadianToDegree(double x)
{
	return x * 180.0/ PI;
}
inline double Cartesian2PolarAngle(double x, double y)
{
	double angle = atan2(y, x);
	return angle < 0 ? 360 + RadianToDegree(angle) : RadianToDegree(angle);
}
class CAM
{
public:
	CAM();
	~CAM();
public:
	double Nc;
	double F;
	double c;
	double xyz_white[3];
	double xyz_ref[3];
	double Yb;
	double La;

public:
	virtual CAMOuptut GetForwardValue(double* xyz);
	virtual double* GetInverseValue(CAMOuptut* output);

	
public:
	double* Multiply3x3WithVector(const double* M, const double* x)
	{
		double result[3];
		for (size_t i = 0; i < 3; i++)
		{
			result[i] = M[i * 3] * x[0] + M[i * 3 + 1] * x[1] + M[i * 3 + 2] * x[2];
		}
		return result;
	}

};

