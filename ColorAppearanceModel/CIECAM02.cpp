#include "stdafx.h"
#include "CIECAM02.h"
#include "CAM.h"
#include <math.h>
#include <vector>
#include <algorithm>
#define IsLargerThan(x1, x2) x1 > x2? 1:0; 

//Reference: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.89.7364&rep=rep1&type=pdf


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
	xyz_white = enviroment->XYZ_white;
	Initialize();
}

CIECAM02::~CIECAM02()
{
}

CAMOuptut CIECAM02::GetForwardValue(const double * xyz)
{
	//XYZ to Cone Response Space LMS
	double* lmsW = ChromaticAdaption(xyz_white);//RwGwBw
	double* lms = ChromaticAdaption(xyz);//RGB

	//Adapted Cone Response
	double* lmsCorrsp = new double[3];//RcGcBc
	double* lmsCorrspW = new double[3]; //RcwGcwBcw
	for (size_t i = 0; i < 3 ;i++)
	{
		lmsCorrsp[i] = (D*(xyz_white[1] / lmsW[i]) + (1 - D))*lms[i];
		lmsCorrspW[i] = (D*(xyz_white[1] / lmsW[i]) + (1 - D))*lmsW[i];
	}
	//Adapted Cone Response to HPE space
	double* lmsPrime = HPETransform(InverseChromaticAdaption (lmsCorrsp));//R'G'B'
	double* lmsWhitePrime = HPETransform(InverseChromaticAdaption(lmsCorrspW));//Rw'Gw'Bw'

	//Post adaption nonlinear response compression
	double* lmsPostAdaption = new double[3];
	double* lmsWhitePostAdaption = new double[3];
	for (size_t i = 0; i < 3; i++)
	{
		lmsPostAdaption[i] = NonlinearCompression(lmsPrime[i]);
		lmsWhitePostAdaption[i] = NonlinearCompression(lmsWhitePrime[i]);
	}

	//a and b
	double a = lmsPostAdaption[0] - 12 * lmsPostAdaption[1] / 11 + lmsPostAdaption[2] / 11;
	double b = (lmsPostAdaption[0] + lmsPostAdaption[1] - 2 * lmsPostAdaption[2]) / 9;
	//Hue angle
	double hueAngle = Cartesian2PolarAngle(a, b);
	//Eccentricity factor
	double et = (12500.0 * Nc*Ncb / 13.0) * (cos(hueAngle*PI/180 + 2) + 3.8);
	//Temporal magnitude
	double t = et * sqrt(a*a + b * b) / (lmsPostAdaption[0] + lmsPostAdaption[1] + 1.05*lmsPostAdaption[2]);
	//Achromatic response
	double A = GetAchromaticResponse(lmsPostAdaption);
	double Aw = GetAchromaticResponse(lmsWhitePostAdaption);

	//Delete used variables
	delete lmsW;
	delete lms;
	delete lmsCorrsp;
	delete lmsCorrspW;
	delete lmsPrime;
	delete lmsWhitePrime;
	delete lmsPostAdaption;
	delete lmsWhitePostAdaption;

	//Calculate 
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
	hueAngle = DegreeToRadian(hueAngle);
	output->am = output->M * cos(hueAngle);
	output->bm = output->M * sin(hueAngle);
	output->ac = output->C * cos(hueAngle);
	output->bc = output->C * sin(hueAngle);
	output->as = output->s * cos(hueAngle);
	output->bs = output->s * sin(hueAngle);



	//Return results
	return *output;
}
double CIECAM02::CalculateHueComposition(const double hueAngle)
{
	double hp = hueAngle < h[0] ? hueAngle + 360 : hueAngle;
	
	
	vector<double>::iterator it = std::find_if(h.begin(), h.end(), [&hp](double hi)->bool {return  hi >= hp; });
	int i = abs(std::distance(it, h.begin())) - 1;
	
	return H[i] + 100 * (hp - h[i]) / e[i] / ((hp - h[i]) / e[i] + (h[i + 1]-hp ) / e[i + 1]);
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
	double* lmsCorrspW = new double[3]; 
	for (size_t i = 0; i < 3; i++)
	{
		lmsCorrspW[i] = (D*(xyz_white[1] / lmsW[i]) + (1 - D))*lmsW[i];
	}
	double* lmsWhitePrime = HPETransform(InverseChromaticAdaption(lmsCorrspW));
	double* lmsWhitePostAdaption = new double[3];
	for (size_t i = 0; i < 3; i++)
	{
		lmsWhitePostAdaption[i] = NonlinearCompression(lmsWhitePrime[i]);
	}
	double Aw = GetAchromaticResponse(lmsWhitePostAdaption);

	double hueAngle = CalculateHueAngle(output->H);
	double t = pow(output->C / (sqrt(output->J / 100.0) * pow(1.64 - pow(0.29, n), 0.73)), 10.0 / 9.0);
	double et = ((12500.0 / 13.0) * Nc * Ncb) * (cos(DegreeToRadian(output->h) + 2.0) + 3.8);
	double A = pow(output->J / 100.0, 1.0 / (c * z)) * Aw;

	double p1 =  et / t;
	double p2 = (A / Nbb) + 0.305;
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
	
	vector<double>::iterator it = std::find_if(H.begin(), H.end(), [&Hc](double Hi) {return Hi >= Hc; });
	int i = abs(distance(H.begin(), it)) - 1;
	double hue = ((Hc - H[i])*(e[i + 1] * h[i] - e[i] * h[i + 1]) - 100 * h[i] * e[i + 1]) / ((Hc - H[i])*(e[i + 1] - e[i]) - 100 * e[i + 1]);
	return hue > 360 ? hue - 360 : hue;
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

double* CIECAM02::CalculateColorDifference(const double* xyz1, const double* xyz2)
{
	CAMOuptut output1 = GetForwardValue(xyz1);
	CAMOuptut output2 = GetForwardValue(xyz2);
	CAMOuptut diff = output1 - output2;
	double deltaE_JMH = sqrt(diff.J *diff.J + diff.am*diff.am + diff.bm*diff.bm);
	double deltaE_JCH = sqrt(diff.J *diff.J + diff.ac*diff.ac + diff.bc*diff.bc);
	double deltaE_JSH = sqrt(diff.J *diff.J + diff.ac*diff.as + diff.bc*diff.bs);
	return new double[3]{deltaE_JCH, deltaE_JMH, deltaE_JSH};
}

void CIECAM02::Initialize()
{
	k = 1 / (5 * La + 1);
	FL = 0.2*pow(k, 4) * 5 * La + 0.1*(1 - pow(k, 4))*(1 - pow(k, 4))*pow(5 * La, 0.3333);
	n = Yb / xyz_white[1];
	Ncb = Nbb = 0.725*pow(1 / n, 0.2);
	D = F * (1 - (1 / 3.6)*exp((-La - 42) / 92));
	z = 1.48 + sqrt(n);
}
