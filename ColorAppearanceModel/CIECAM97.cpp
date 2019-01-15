#include "stdafx.h"
#include "CIECAM97.h"


CIECAM97::CIECAM97()
{
}


CIECAM97::~CIECAM97()
{
}

CAMOuptut CIECAM97::GetForwardValue(const double * xyz)
{
	return CAMOuptut();
}

double * CIECAM97::GetInverseValue(const CAMOuptut * output)
{
	return nullptr;
}
