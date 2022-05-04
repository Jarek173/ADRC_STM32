#include "nonlinearfunction.h"

#include "stdlib.h"
#include "math.h"

static float sign(float value)
{
	if (value < 0)
		return -1;
	if(value == 0)
		return 0;
	return 1;
}

float NN_Fal(float r, float alpha, float delta)
{
	if(abs(r) > delta)
		return sign(r) * pow(abs(r), alpha);
	return r/pow(delta, 1-alpha);
}

float NN_Fhan(float x1, float x2, float r, float h)
{
	    float d = h*pow(r,2);
	    float a0= h*x2;
		float y=x1+a0;
		float a1=sqrt(d*(d+8*abs(y)));
		float a2=a0+sign(y)*(a1-d)/2.0;
		float sy = (sign(y+d) - sign(y-d))/2.0;
		float a = (a0+y-a2)*sy+a2;
	    float sa = (sign(a+d) - sign(a-d))/2.0;
	    return -r*(a/d-sign(a))*sa-r*sign(a);
}
