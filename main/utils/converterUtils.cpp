/*
 * converterUtils.cpp
 *
 *  Created on: 11 Nov 2017
 *      Author: Juri Bieler
 */

#include "converterUtils.hpp"

#include <stdio.h>

#include <string>

int64_t bytes_to_int64(uint8_t* buff) {
  return buff[0] + ((uint64_t)buff[1] << 8) + ((uint64_t)buff[2] << 16) +
         ((uint64_t)buff[3] << 24) + ((uint64_t)buff[4] << 32) +
         ((uint64_t)buff[5] << 40) + ((uint64_t)buff[6] << 48) +
         ((uint64_t)buff[7] << 56);
}

void int64_to_bytes(int64_t num, uint8_t* buff) {
  buff[0] = (uint8_t)(num >> 0);
  buff[1] = (uint8_t)(num >> 8);
  buff[2] = (uint8_t)(num >> 16);
  buff[3] = (uint8_t)(num >> 24);
  buff[4] = (uint8_t)(num >> 32);
  buff[5] = (uint8_t)(num >> 40);
  buff[6] = (uint8_t)(num >> 48);
  buff[7] = (uint8_t)(num >> 56);
}

uint64_t bytes_to_uint64(uint8_t* buff) {
  return buff[0] + ((uint64_t)buff[1] << 8) + ((uint64_t)buff[2] << 16) +
         ((uint64_t)buff[3] << 24) + ((uint64_t)buff[4] << 32) +
         ((uint64_t)buff[5] << 40) + ((uint64_t)buff[6] << 48) +
         ((uint64_t)buff[7] << 56);
}

void uint64_to_bytes(uint64_t num, uint8_t* buff) {
  buff[0] = (uint8_t)(num >> 0);
  buff[1] = (uint8_t)(num >> 8);
  buff[2] = (uint8_t)(num >> 16);
  buff[3] = (uint8_t)(num >> 24);
  buff[4] = (uint8_t)(num >> 32);
  buff[5] = (uint8_t)(num >> 40);
  buff[6] = (uint8_t)(num >> 48);
  buff[7] = (uint8_t)(num >> 56);
}

int32_t bytes_to_int32(uint8_t* buff) {
  return buff[0] + (buff[1] << 8) + (buff[2] << 16) + (buff[3] << 24);
}

void int32_to_bytes(int32_t num, uint8_t* buff) {
  buff[0] = (uint8_t)(num >> 0);
  buff[1] = (uint8_t)(num >> 8);
  buff[2] = (uint8_t)(num >> 16);
  buff[3] = (uint8_t)(num >> 24);
}

uint32_t bytes_to_uint32(uint8_t* buff) {
  return buff[0] + (buff[1] << 8) + (buff[2] << 16) + (buff[3] << 24);
}

void uint32_to_bytes(uint32_t num, uint8_t* buff) {
  buff[0] = (uint8_t)(num >> 0);
  buff[1] = (uint8_t)(num >> 8);
  buff[2] = (uint8_t)(num >> 16);
  buff[3] = (uint8_t)(num >> 24);
}

int16_t bytes_to_int16(uint8_t* buff) { return buff[0] + (buff[1] << 8); }

void int16_to_bytes(int16_t num, uint8_t* buff) {
  buff[0] = (uint8_t)(num >> 0);
  buff[1] = (uint8_t)(num >> 8);
}

uint16_t bytes_to_uint16(uint8_t* buff) { return buff[0] | (buff[1] << 8); }

void uint16_to_bytes(uint16_t num, uint8_t* buff) {
  buff[0] = (uint8_t)(num >> 0);
  buff[1] = (uint8_t)(num >> 8);
}

long bytes_to_long(uint8_t* buf) {
  longByteConv b2l;
  memcpy(b2l.b, buf, 4);
  return b2l.l;
}

void long_to_bytes(long longInt, uint8_t* buf) {
  longByteConv l2b;
  l2b.l = longInt;
  // printf("The address of s is: %p\n", &myUnion.myByte);
  memcpy(buf, l2b.b, 4);
}

float bytes_to_float(uint8_t* buf) {
  floatByteConv b2f;
  memcpy(b2f.b, buf, 4);
  return b2f.f;
}

void float_to_bytes(float val, uint8_t* buf) {
  floatByteConv f2b;
  f2b.f = val;
  memcpy(buf, f2b.b, 4);
}

double bytes_to_double(uint8_t* buf) {
  doubleByteConv b2d;
  memcpy(b2d.b, buf, 8);
  return b2d.d;
}

void double_to_bytes(double val, uint8_t* buf) {
  doubleByteConv d2b;
  d2b.d = val;
  memcpy(buf, d2b.b, 8);
}

void str_parse_double(uint8_t* str, uint8_t len, double* output) {
  uint8_t parseStr[len];
  memcpy(parseStr, str, len);
  *output = atof((const char*)parseStr);
}

void str_parse_float(uint8_t* str, uint8_t len, float* output) {
  uint8_t parseStr[len];
  memcpy(parseStr, str, len);
  *output = atof((const char*)parseStr);
}

void str_parse_uint8(uint8_t* str, uint8_t len, uint8_t* output) {
  // max int size plus sign -> digits of 2^(4*8-1) and + or -
  uint8_t parseStr[11];
  // copy it to fresh buffer
  memcpy(parseStr, str, len);
  // fill rest of buffer with 0x00
  for (uint8_t i = len; i < sizeof(int); i++) {
    parseStr[i] = 0x00;
  }
  *output = (uint8_t)atoi((const char*)parseStr);
}

// note: untested
void str_parse_uint32(uint8_t* str, uint8_t len, uint32_t* output) {
  uint8_t parseStr[11];
  memcpy(parseStr, str, len);
  for (uint8_t i = len; i < sizeof(int); i++) {
    parseStr[i] = 0x00;
  }
  *output = (uint32_t)atol((const char*)parseStr);
}

// note: untested
void str_parse_uint64(uint8_t* str, uint8_t len, uint64_t* output) {
  uint8_t parseStr[20];
  memcpy(parseStr, str, len);
  for (uint8_t i = len; i < sizeof(int); i++) {
    parseStr[i] = 0x00;
  }
  *output = (uint64_t)atoll((const char*)parseStr);
}

int hex_str_parse_uint8_buff(uint8_t* hex_str, uint8_t len, uint8_t* buff) {
  uint8_t i = 0;
  for (i = 0; i < len / 2; i++) {
    buff[i] = hex_to_uint8(hex_str + (2 * i));
  }
  return i;
}

uint8_t hex_to_uint8(uint8_t* hex) {
  uint8_t val0 = char_to_uint8(hex[0]);
  uint8_t val1 = char_to_uint8(hex[1]);
  uint8_t val = (val0 << 4) | (val1 & 0xF);
  return val;
}

uint8_t char_to_uint8(uint8_t ch) {
  if (ch >= '0' && ch <= '9')
    return ch - '0';
  else if (ch >= 'a' && ch <= 'f')
    return ch - 'a' + 10;
  else if (ch >= 'A' && ch <= 'F')
    return ch - 'A' + 10;
  return 0;
}
