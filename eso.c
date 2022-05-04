#include "eso.h"
#include "nonlinearfunction.h"
#include "math.h"

void ESOCalculateNewSpaceState(ESO_2_StateSpace* spaceState, ESO_2_Gains* gains, const float u, const float y, const float b0, const float h)
{
	float e = spaceState->z1 - y;
	spaceState->z1 = spaceState->z1 + h*spaceState->z2 - gains->l1*e;
	spaceState->z2 = spaceState->z2 + h*(spaceState->z3+b0*u) - gains->l2 *  NN_Fal(e, 0.5, h);
	spaceState->z3 = spaceState->z3 - gains->l3*NN_Fal(e, 0.25, h);
}

ESO_2_Gains ESOCalculateGainsBaseOnBandwidth(float bandwidth)
{
	ESO_2_Gains gains;
	gains.l1 = 3*bandwidth;
	gains.l2 = 3*pow(bandwidth, 2);
	gains.l3 = pow(bandwidth, 3);
	return gains;
}

ESO_2_Gains ESOCalculateGainsBaseOnSampleTime(float sampleTime)
{
	ESO_2_Gains gains;
	gains.l1 = 1;
	gains.l2 = 1/(2*pow(sampleTime,0.5));
	gains.l3 = 2/(25*pow(sampleTime, 1.2));
	return gains;
}
