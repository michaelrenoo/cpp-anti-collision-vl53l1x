/*
 * filters.cpp
 *
 *  Created on: 1 Mar 2019
 *      Author: Juri Bieler
 */

#ifndef MAIN_UTILS_FILTERS_CPP_
#define MAIN_UTILS_FILTERS_CPP_

#include <math.h>

#include "driver/driver.hpp"

int32_t filter_iir(int32_t in_now, int32_t out_prev, float coeff);
float filter_iir_f(float in_now, float out_prev, float coeff);

#endif /* MAIN_UTILS_FILTERS_CPP_ */
