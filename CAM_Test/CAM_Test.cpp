// CAM_Test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "..\ColorAppearanceModel\CIECAM02.h"
void PrintCiecam02ForwardSolution(double* xyz, CAMEnviroment* envo, CAMOuptut* v)
{
	cout << "Forward Solution of CIECAM02  "<< endl;
	cout << "----------------------------  " << endl;
	cout << "Input XYZ Value: " << xyz[0] << "," << xyz[1] << "," << xyz[2] << endl;
	cout << "Enviroment settings" << endl;
	cout << "XYZ Value of White Point: " << envo->XYZ_white[0] << "," << envo->XYZ_white[1] << "," << envo->XYZ_white[2] << endl;
	cout << "La = " << envo->La << endl;
	cout << "Yb = " << envo->Yb << endl;
	cout << "----------------------------  " << endl;
	cout << "Output: " << endl;
	cout << "Q = " << v->Q << endl;
	cout << "J = " << v->J << endl;
	cout << "M = " << v->M << endl;
	cout << "C = " << v->C << endl;
	cout << "S = " << v->s << endl;
	cout << "h = " << v->h << endl;
	cout << "H = " << v->H << endl;
	cout << "a = " << v->a << endl;
	cout << "b = " << v->b << endl;

}
int main()
{
	CAMEnviroment* envrmnt = new CAMEnviroment;
	envrmnt->Condition = ViewConditionParam::Average;
	envrmnt->XYZ_white = new double[3]{ 95.05,100,108.88 };
	envrmnt->La = 31.83;
	envrmnt->Yb = 20;
	CIECAM02* cam02 = new CIECAM02(envrmnt);
	double* testXyz = new double[3]{ 57.06, 43.06, 31.96 };
	CAMOuptut result = cam02->GetForwardValue(testXyz);
	PrintCiecam02ForwardSolution(testXyz, envrmnt, &result);
	double* reserveXyz = cam02->GetInverseValue(&result);
	
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
