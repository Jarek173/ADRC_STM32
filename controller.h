#ifndef CONTROLLER_H
#define CONTROLLER_H

/**
 * @brief Structure with PD gains
 */
typedef struct
{
	float kp;
	float kd;
} PD_Gains;

/**
 * @brief Calculate controller output depends on PD gains
 * @param e1 Proportional error
 * @param e2 Derivative error
 * @return Control input for Plant
 */
float PDCalculate(float e1, float e2, PD_Gains* pdGains);

/**
 * @brief Calculate nonlinear controller output depends on PD gains
 * @param e1 Proportional error
 * @param e2 Derivative error
 * @return Control input for Plant
 */
float PDNonlinearCalculate(float e1, float e2, PD_Gains* pdGains, float alpha1, float alpha2, float delta);


/**
 * @brief Calculate PD gains depends on bandwidth
 * @param bandwith
 * @return Structure with calculated PD gains
 */
PD_Gains PDCalculateGainsBaseOnBandwith(float bandwidth);

#endif
