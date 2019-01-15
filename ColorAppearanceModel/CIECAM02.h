#pragma once

#ifndef _CIECAM02_H
#define _CIECAM02_H
#endif // !_CIECAM02_H


#include "CAM.h"
#include <vector>
#include <algorithm>

using namespace std;


class CAM_API CIECAM02:public CAM
{
public:
	CIECAM02(CAMEnviroment* enviroment);
	~CIECAM02();
private:
	double Mcat02[3][3] = { {0.7328,0.4296,-0.1624}, {-0.7036,1.6975,0.0061}, {0.0030,0.0136,0.9834} };
	double Mcat02Inv[3][3] = { {1.0961,-0.2789,0.1827}, {0.4544,0.4735,0.0721}, {-0.0096,-0.0057,1.0153} };
	double Mhpe[3][3] = { {0.38971,0.68898,-0.07868}, {-0.22981,1.18340,0.04641}, {0,0,1} };
	double MhpeInv[3][3] = { {1.9101, -1.1121, 0.2019}, {0.3709, 0.6291, 0.0000}, {0.0000, 0.0000, 1.0000} };
	vector<double> h { 20.14, 90, 164.25, 237.53, 380.14 };
	vector<double> e { 0.8, 0.7, 1.0, 1.2, 0.8 };
	vector<double> H { 0, 100, 200, 300, 400 };
public:
	CAMOuptut GetForwardValue(const double* xyz);
	double* GetInverseValue(const CAMOuptut* output);
	double* ChromaticAdaption(const double* input);
	double* InverseChromaticAdaption(const double* input);
	double* HPETransform(const double* input);
	double* HPEInverseTransform(const double* input);
	double CalculateColorDifference(const double* xyz1, const double* xyz2);
	void Initialize();
private:
	double CalculateHueComposition(const double huaAngle);
	double GetAchromaticResponse(const double * rgb);
	double NonlinearCompression(const double v);
	double CalculateHueAngle(const double H);
	double* GetLMSFromAchromaticResponse(double p2, double a, double b);
	double InverseNonlinearCompression(double input);
private:
	double D, FL, k, n, Nbb, Ncb, z;
};

