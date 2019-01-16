#pragma once
typedef enum ViewConditionParam
{
	Average,
	Dim,
	Dark
};
extern struct CAMEnviroment
{
public:
	ViewConditionParam Condition;
	double* XYZ_white;
	double Yb;
	double La;

};
extern struct CAMOuptut
{
public:
	CAMOuptut operator -(const CAMOuptut& x){
		CAMOuptut diff;
		diff.J = x.J - J;
		diff.as = x.as - as;
		diff.bs = x.bs - bs;
		diff.ac = x.ac - ac;
		diff.bc = x.bc - bc;
		diff.am = x.am - am;
		diff.bm = x.bm - bm;
		return diff;
	};
public:
	double Q;//Brightness
	double J;//Lightness;
	double M;//Colorfulness;
	double C;// Chroma;
	double s;// Saturation;
	double h;// HueAngle;
	double H;// HueComposition;
	double a;
	double b;
	double ac;
	double bc;
	double am;
	double bm;
	double as;
	double bs;
};
