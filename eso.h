#ifndef ESO_H
#define ESO_H

#include <math.h>

/**
 * @brief Extended state observer -> gains for second-order Plant
 */
typedef struct{
	float l1;
	float l2;
	float l3;
} ESO_2_Gains;


/**
 * @brief Extended state observer -> state-space for second order Plant
 */
typedef struct{
	float z1;
	float z2;
	float z3;
} ESO_2_StateSpace;



/**
 *@brief Function for calculating new ESO state space
 *@param spaceState Pointer to structure with state-space. This variable should contain last state-space,
 *        also new estimated state-space will be wrote to this variable after calculation
 *@param gains ESO gains defined by user
 *@param u Control input
 *@param y Output
 *@param b0 Approximation of b in plant
 *@param h Sampling time
 */
void ESOCalculateNewSpaceState(ESO_2_StateSpace* spaceState, ESO_2_Gains* gains, const float u, const float y, const float b0, const float h);

/**
 * @brief Function for calculating ESO gains depends on ESO bandwidth
 * @param bandwidth
 * @return Structure with ESO Gains
 */
ESO_2_Gains ESOCalculateGainsBaseOnBandwidth(float bandwidth);

/**
 * @brief Function for calculating ESO gains depends on plan sample time
 * @param sampleTime
 * @return Structure with ESO Gains
 */
ESO_2_Gains ESOCalculateGainsBaseOnSampleTime(float sampleTime);
#endif
