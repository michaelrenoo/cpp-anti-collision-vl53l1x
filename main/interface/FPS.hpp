/*
 * GPSParser.h
 *
 *  Created on: Jan 28, 2018
 *      Author: juri
 */

#ifndef MAIN_FPS_H_
#define MAIN_FPS_H_

#include <stdio.h>

#include <string>

#include "driver/driver.hpp"
#include "interface/MavlinkCom.hpp"

#define FPS_IN_BUF_SIZE 1024
#define FPS_OUT_BUF_SIZE 512

class FPS {
 private:
  uint8_t isInitialized;
  uart_port_t uartPort;
  int send_ack_msg(uint8_t msgClass, uint8_t msgID);

 public:
  FPS();
  void init(uart_port_t uartPort);
  int parse(uint8_t* str, uint16_t len);
  int send_bytes(uint8_t* buff, uint16_t len);
  uart_port_t get_uart_port();
  bool is_initialized();
};

#endif /* MAIN_FPS_H_ */
