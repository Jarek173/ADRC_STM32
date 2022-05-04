#include "trackingdiff.h"
#include "nonlinearfunction.h"

void TDCalculate(TD_2* td, float v, float h1)
{
	td->v1 = td->v1 + h1*td->v2;
	float x1 = td->v1-v;
	td->v2 = td->v2 + h1*NN_Fhan(x1, td->v2, 2, h1);
}
