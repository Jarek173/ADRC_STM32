#ifndef TRACKING_DIFF_H
#define TRACKING_DIFF_H


/**
 * @brief Structure for desired trajectory and its derivative
 */
typedef struct
{
  float v1;
  float v2;
}TD_2;

/**
 * @brief Function for calculating TD for second-order system
 * @param td structure with desired trajectory and its derivative
 * @param v Reference trajectory
 * @param h1 Value which determine speed up or slow down the transient profile
 */
void TDCalculate(TD_2* td, float v, float h1);

#endif
