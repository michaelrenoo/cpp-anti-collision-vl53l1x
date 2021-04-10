/*
 *      Written by Nathan Seidle @ SparkFun Electronics, April 12th, 2018
 *      Modified by Juri Bieler and Michael Reno to work with with Raspberry Pi
 *      4, January 2021
 *      Email: juribieler@gmail.com, michaelreno19@gmail.com
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../interface/IscMaster.hpp"
#include "vl53l1_register_map.h"

class VL53L1X {
 public:
  bool init(IscMaster *iscm);
  void softReset();  // Reset the sensor via software
  void startMeasurement(
      uint8_t newAddress,
      uint8_t offset = 0);   // Write a block of bytes to the sensor to
                             // configure it to take a measurement
  bool newDataReady();       // Polls the measurement completion bit
  uint16_t getDistance();    // Returns the results from the last measurement,
                             // distance in mm
  uint16_t getSignalRate();  // Returns the results from the last measurement,
                             // signal rate
  void setDistanceMode(uint8_t mode = 2);  // Defaults to long range
  uint8_t getDistanceMode();
  uint8_t getRangeStatus();  // Returns the results from the last measurement,
                             // 0 = valid
  void setAddress(uint8_t newAddress);  // Set new I2C address to the sensor
  uint8_t getAddress() { return deviceAddress; };

 private:
  IscMaster *iscm;        // i2c master
  uint8_t deviceAddress;  // current device address
  static const uint8_t defaultAddress_VL53L1X =
      0x29;                                         // default device addresss
  static constexpr uint8_t I2C_BUFFER_LENGTH = 32;  // default I2C buffer length

  // Variables
  uint8_t _i2cport;  // The generic connection to user's chosen I2C hardware
  uint8_t _distanceMode = 0;  // Mode 0 = Standard
};
