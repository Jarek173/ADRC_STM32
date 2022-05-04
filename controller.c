#include "controller.h"
#include "nonlinearfunction.h"

#include <math.h>

float PDCalculate(float e1, float e2, PD_Gains* pdGains)
{
	return e1*pdGains->kp + e2*pdGains->kd;
}

float PDNonlinearCalculate(float e1, float e2, PD_Gains* pdGains, float alpha1, float alpha2, float delta)
{
	return NN_Fal(e1, alpha1,delta)*pdGains->kp + NN_Fal(e2, alpha2,delta)*pdGains->kd;
}

PD_Gains PDCalculateGainsBaseOnBandwith(float bandwidth)
{
	PD_Gains gains;
	gains.kp = pow(bandwidth, 2);
	gains.kd = 2*bandwidth;
	return gains;
}
