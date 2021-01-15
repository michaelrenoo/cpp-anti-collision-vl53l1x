/*
 * converterUtils.h
 *
 *  Created on: 11 Nov 2017
 *      Author: Juri Bieler
 */

#ifndef LIBRARIES_UTILS_CONVERTERUTILS_H_
#define LIBRARIES_UTILS_CONVERTERUTILS_H_

#include <stdio.h>
#include <stdlib.h>

#include <cstdint>
#include <cstring>
#include <string>

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

typedef union {
  uint8_t b[4];
  long l;
} longByteConv;

typedef union {
  float f;
  unsigned char b[4];
} floatByteConv;

typedef union {
  double d;
  unsigned char b[sizeof(double)];
} doubleByteConv;

int16_t bytes_to_int16(uint8_t* buff);
void int16_to_bytes(int16_t num, uint8_t* buff);

uint16_t bytes_to_uint16(uint8_t* buff);
void uint16_to_bytes(uint16_t num, uint8_t* buff);

int32_t bytes_to_int32(uint8_t* buff);
void int32_to_bytes(int32_t num, uint8_t* buff);

uint32_t bytes_to_uint32(uint8_t* buff);
void uint32_to_bytes(uint32_t num, uint8_t* buff);

int64_t bytes_to_int64(uint8_t* buff);
void int64_to_bytes(int64_t num, uint8_t* buff);

uint64_t bytes_to_uint64(uint8_t* buff);
void uint64_to_bytes(uint64_t num, uint8_t* buff);

long bytes_to_long(uint8_t* buf);
void long_to_bytes(long longInt, uint8_t* buf);

float bytes_to_float(uint8_t* buf);
void float_to_bytes(float val, uint8_t* buf);

double bytes_to_double(uint8_t* buf);
void double_to_bytes(double val, uint8_t* buf);

void str_parse_double(uint8_t* str, uint8_t len, double* output);
void str_parse_float(uint8_t* str, uint8_t len, float* output);
void str_parse_uint8(uint8_t* str, uint8_t len, uint8_t* output);
void str_parse_uint32(uint8_t* str, uint8_t len, uint32_t* output);
void str_parse_uint64(uint8_t* str, uint8_t len, uint64_t* output);

int hex_str_parse_uint8_buff(uint8_t* hex_str, uint8_t len, uint8_t* buff);
uint8_t hex_to_uint8(uint8_t* hex);
uint8_t char_to_uint8(uint8_t ch);

#endif /* LIBRARIES_UTILS_CONVERTERUTILS_H_ */
