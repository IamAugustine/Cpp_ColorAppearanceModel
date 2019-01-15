#pragma once
#include "CAM.h"
class CAM_API CIECAM97 :
	public CAM
{
public:
	CIECAM97();
	~CIECAM97();
public:
	CAMOuptut GetForwardValue(const double* xyz);
	double* GetInverseValue(const CAMOuptut* output);
};

