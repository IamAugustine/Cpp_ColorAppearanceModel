#pragma once
enum ViewConditionParam
{
	Average,
	Dim,
	Dark
};
struct CAMEnviroment
{
public:
	ViewConditionParam Condition;
	double XYZ_white[3];
	double Yb;
	double La;

};
struct CAMOuptut
{
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
};
