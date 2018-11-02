#include "stdafx.h"
#include "CIECAM02.h"
#include "CAM.h"
#include <math.h>
//#define PI 3.1415927
#include <vector>
#include <algorithm>
#define IsLargerThan(x1, x2) x1 > x2? 1:0; 


using namespace std;

CIECAM02::CIECAM02(CAMEnviroment * enviroment)
{
	switch (enviroment->Condition)
	{
	case ViewConditionParam::Average:
		Nc = 1.0;
		c = 0.69;
		F = 1.0;
		break;
	case ViewConditionParam::Dim:
		Nc = 0.95;
		c = 0.59;
		F = 0.9;
		break;
	case ViewConditionParam::Dark:
		Nc = 0.8;
		c = 0.525;
		F = 0.8;
		break;
	default:
		Nc = 1.0;
		c = 0.69;
		F = 1.0;
		break;
	}
	La = enviroment->La;
	Yb = enviroment->Yb;

}

CIECAM02::~CIECAM02()
{
}

CAMOuptut CIECAM02::GetForwardValue(const double * xyz)
{
	double* lmsW = ChromaticAdaption(xyz_white);
	double* lms = ChromaticAdaption(xyz);
	double* lmsCorrsp = new double[3];//Adapted Cone Response
	for (size_t i = 0; i < 3 ;i++)
	{
		lmsCorrsp[i] = (D*(xyz_white[1] / lmsW[i]) + (1 - D))*lms[i];
	}
	double* lmsPrime = HPETransform(InverseChromaticAdaption (lmsCorrsp));
	double* lmsWhitePrime = HPETransform(InverseChromaticAdaption(lmsW));
	double* lmsPostAdaption = new double[3];
	double* lmsWhitePostAdaption = new double[3];

	
	for (size_t i = 0; i < 3; i++)
	{
		lmsPostAdaption[i] = NonlinearCompression(lmsPrime[i]);
		lmsWhitePostAdaption[i] = NonlinearCompression(lmsWhitePrime[i]);
	}
	double a = lmsPostAdaption[0] - 12 * lmsPostAdaption[1] / 11 + lmsPostAdaption[2] / 11;
	double b = (lmsPostAdaption[0] + lmsPostAdaption[1] - 2 * lmsPostAdaption[2]) / 9;
	double hueAngle = Cartesian2PolarAngle(a, b);
	double et = 0.25*(cos(hueAngle + 2) + 3.8);
	double t = (50000 * Nc*Ncb / 13)* et * sqrt(a*a + b * b) / (lmsPostAdaption[0] + lmsPostAdaption[1] + 1.05*lmsPostAdaption[2]);
	hueAngle = RadianToDegree(hueAngle);
	double A = GetAchromaticResponse(lmsPostAdaption);
	double Aw = GetAchromaticResponse(lmsWhitePostAdaption);
	CAMOuptut* output = new CAMOuptut;
	output->a = a;
	output->b = b;
	output->h = hueAngle;
	output->H = CalculateHueComposition(hueAngle);
	output->J = 100 * pow(A / Aw, c*z);
	output->Q = 4 / c * sqrt(output->J / 100)*(Aw + 4)*pow(FL, 0.25);
	output->C = pow(t, 0.9)*sqrt(output->J / 100)*pow((1.64 - pow(0.29, n)), 0.73);
	output->M = output->C*pow(FL, 0.25);
	output->s = 100 * sqrt(output->M / output->Q);
	return *output;
}
double CIECAM02::CalculateHueComposition(const double hueAngle)
{
	double hp = hueAngle < H[0] ? hueAngle + 360 : hueAngle;
	
	double hi;
	vector<double>::iterator it = std::find_if(h.begin(), h.end(), [&hi](const double& hp)->bool {return  hi >= hp; });
	int i = std::distance(it, h.begin()) - 1;
	
	return H[i] + 100 * (hp - h[i]) / e[i] / ((hp - h[i] / e[i] + (h[i + 1] - hp) / e[i + 1]));
}
double CIECAM02::GetAchromaticResponse(const double* rgb)
{
	return (2 * rgb[0] + rgb[1] + rgb[2] / 20 - 0.305)*Nbb;
}
double CIECAM02::NonlinearCompression(double v)
{
	double tempV = pow(FL * v / 100, 0.42);
	double y = 400 * tempV / (27.13 + tempV) + 0.1;
	return y < 0 ?  -y : y;
}
double* CIECAM02::GetInverseValue(const CAMOuptut* output)
{
	double* lmsW = ChromaticAdaption(xyz_white);
	double* lmsWhitePrime = HPETransform(InverseChromaticAdaption(lmsW));
	double* lmsWhitePostAdaption = new double[3];
	//std::transform(lmsWhitePrime, lmsWhitePrime+3, lmsWhitePostAdaption, NonlinearCompression);
	for (size_t i = 0; i < 3; i++)
	{
		lmsWhitePostAdaption[i] = NonlinearCompression(lmsWhitePrime[i]);
	}
	double Aw = GetAchromaticResponse(lmsWhitePostAdaption);

	double hueAngle = CalculateHueAngle(output->H);
	double t = pow(output->C / (sqrt(output->C / 100.0) * pow(1.64 - pow(0.29, n), 0.73)), 10 / 9);
	double et = 0.25 * (cos(DegreeToRadian(output->h) + 2.0) + 3.8);
	double a = pow(output->J / 100.0, 1.0 / (c * z)) * Aw;

	double p1 = ((50000.0 / 13.0) * Nc * Ncb) * et / t;
	double p2 = (a / Nbb) + 0.305;
	double p3 = 21.0 / 20.0;
	double p4, p5, ca, cb;
	double hr = DegreeToRadian(hueAngle);
	if (abs(sin(hr)) >= abs(cos(hr))) {
		p4 = p1 / sin(hr);
		cb = (p2 * (2.0 + p3) * (460.0 / 1403.0)) /
			(p4 + (2.0 + p3) * (220.0 / 1403.0) *
			(cos(hr) / sin(hr)) - (27.0 / 1403.0) +
				p3 * (6300.0 / 1403.0));
		ca = cb * (cos(hr) / sin(hr));
	}
	else {
		p5 = p1 / cos(hr);
		ca = (p2 * (2.0 + p3) * (460.0 / 1403.0)) /
			(p5 + (2.0 + p3) * (220.0 / 1403.0) -
			((27.0 / 1403.0) - p3 * (6300.0 / 1403.0)) *
				(sin(hr) / cos(hr)));
		cb = ca * (sin(hr) / cos(hr));
	}
	double* lmsPostAdaption = GetLMSFromAchromaticResponse(p2, ca, cb);
	double* lmsPrime = new double[3];
	for (size_t i = 0; i < 3; i++)
	{
		lmsPrime[i] = InverseNonlinearCompression(lmsPostAdaption[i]);
	}
	double* lmsCorrsp = ChromaticAdaption(HPEInverseTransform(lmsPrime));
	double* lms = new double[3];
	for (size_t i = 0; i < 3; i++)
	{
		lms[i] = lmsCorrsp[i] / (((xyz_white[1] * D) / lmsW[i]) + (1.0 - D));
	}
	return InverseChromaticAdaption(lms);
}
double* CIECAM02::GetLMSFromAchromaticResponse(double p2, double a, double b)
{
	double* lms = new double[3];
	lms[0] = (0.32787 * p2) + (0.32145 * a) + (0.20527 * b);
	lms[1] = (0.32787 * p2) - (0.63507 * a) - (0.18603 * b);
	lms[2] = (0.32787 * p2) - (0.15681 * a) - (4.49038 * b);
	return lms;
}
double CIECAM02::InverseNonlinearCompression(double c)
{

	return  (100.0 / FL) * pow((27.13 * fabs(c - 0.1)) / (400.0 - fabs(c - 0.1)), 1.0 / 0.42);
}
double CIECAM02::CalculateHueAngle(const double Hc)
{
	return 0;
	//vector<double>::iterator it = std::find_if(H.rbegin(), H.rend(), )
}
double * CIECAM02::ChromaticAdaption(const double * input)
{
	return (Multiply3x3WithVector(&Mcat02[0][0], input));
}

double * CIECAM02::InverseChromaticAdaption(const double * input)
{
	return (Multiply3x3WithVector(&Mcat02Inv[0][0], input));;
}

double * CIECAM02::HPETransform(const double * input)
{
	return Multiply3x3WithVector(&Mhpe[0][0], input);
}

double * CIECAM02::HPEInverseTransform(const double * input)
{
	return Multiply3x3WithVector(&MhpeInv[0][0], input);
}

void CIECAM02::Initialize()
{
	k = 1 / (5 * La + 1);
	FL = 0.2*pow(k, 4) * 5 * La + 0.1*(1 - pow(k, 4))*(1 - pow(k, 4))*pow(5 * La, 1 / 3);
	n = Yb / xyz_white[1];
	Ncb = Nbb = 0.725*pow(1 / n, 0.2);
	D = F * (1 - (1 / 3.6)*exp((-La - 42) / 92));
	z = 1.48 + sqrt(n);
}
