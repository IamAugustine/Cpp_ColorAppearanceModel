#pragma once
#include "Vars.h"
#include <math.h>
#ifndef CAM_H
#define CAM_H
#endif

#define CAM_API __declspec(dllexport)




constexpr auto PI = 3.1415927;;
extern inline double DegreeToRadian(double x)
{
	return x/180.0 * PI;
}
extern inline double RadianToDegree(double x)
{
	return x * 180.0/ PI;
}
extern inline double Cartesian2PolarAngle(double x, double y)
{
	double angle = atan2(y, x);
	return angle < 0 ? 360 + RadianToDegree(angle) : RadianToDegree(angle);
}
class CAM_API CAM
{
public:
	CAM();
	~CAM();
public:
	double Nc;
	double F;
	double c;
	double* xyz_white;
	double xyz_ref[3];
	double Yb;
	double La;

public:
	virtual CAMOuptut GetForwardValue(const double* xyz) =0;
	virtual double* GetInverseValue(const CAMOuptut* output) =0;
	virtual double* CalculateColorDifference(const double* xyz1, const double* xyz2) =0;
	
public:
	double* Multiply3x3WithVector(const double* M, const double* x)
	{
		double* result = new double[3];
		for (size_t i = 0; i < 3; i++)
		{
			result[i] = M[i * 3] * x[0] + M[i * 3 + 1] * x[1] + M[i * 3 + 2] * x[2];
		}
		return result;
	}

};

