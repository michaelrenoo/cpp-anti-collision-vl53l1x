/*
 * filters.h
 *
 *  Created on: 1 Mar 2019
 *      Author: Juri Bieler
 */

#include "filters.hpp"

/**
 * @brief iir filter using int32 aka exponential vilter aka Exponentially
 * Weighted Moving Average (EWMA)
 *
 * @param in_now current new value
 * @param out_prev last output value of iir filter
 * @param coeff how much effect the in_now has, 0. = none, 1. = 100 percent
 * @return int32_t new filtered value
 */
int32_t filter_iir(int32_t in_now, int32_t out_prev, float coeff) {
  return (int32_t)round((1. - coeff) * out_prev + coeff * in_now);
}

/**
 * @brief iir filter using floats aka exponential vilter aka Exponentially
 * Weighted Moving Average (EWMA)
 *
 * @param in_now current new value
 * @param out_prev last output value of iir filter
 * @param coeff how much effect the in_now has, 0. = none, 1. = 100 percent
 * @return float new filtered value
 */
float filter_iir_f(float in_now, float out_prev, float coeff) {
  return ((1. - coeff) * out_prev + coeff * in_now);
}
