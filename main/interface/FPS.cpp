/*
 * FPS.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: juri
 */

#include "FPS.hpp"

static const char* TAG = "FPS";
#define PATTERN_CHR_NUM 1

extern FPS fps;

extern MavlinkCom mav;

Semaphore fpsSemaphore;

#if BUILD_TARGET == TARGET_ESP32
static void fps_event_task(void* pvParameters) {
#else
static void fps_event_task() {
#endif
  uint8_t* in_buff = (uint8_t*)malloc(FPS_IN_BUF_SIZE);
  for (;;) {
    int len =
        uart_read(fps.get_uart_port(), in_buff, FPS_IN_BUF_SIZE, 0xffffffff);
    if (len > 0) {
      fps.parse(in_buff, len);
    }
  }
  free(in_buff);
  in_buff = NULL;
  end_task();
}

FPS::FPS() {
  // do nothing else here
  this->isInitialized = false;
  this->uartPort = UART_NUM_0;
}

void FPS::init(uart_port_t uartPort) {
  this->uartPort = uartPort;

  // fpsSemaphore = Semaphore();
  uart_init_driver(this->uartPort, FPS_IN_BUF_SIZE, FPS_OUT_BUF_SIZE);
  start_task(fps_event_task, "fps_event", 4 * 1024, 12);

  this->isInitialized = true;
}

int FPS::parse(uint8_t* buff, uint16_t len) {
  mav.parse_in_msg(buff, len);
  return (0);
}

int FPS::send_bytes(uint8_t* buff, uint16_t len) {
  if (this->isInitialized) {
    if (fpsSemaphore.take(200) == true) {
      uart_wait_for_tx_done(this->get_uart_port(), 40);
      int sendLen = uart_write(this->get_uart_port(), buff, len);
      fpsSemaphore.give();
      return sendLen;
    } else {
      log_w(TAG, "Warning: could not obtain semaphore (fps)");
    }
  } else {
    // log_w(TAG, "Warning: fps is not initialized");
  }
  return 0;
}

uart_port_t FPS::get_uart_port() { return (this->uartPort); }

bool FPS::is_initialized() { return this->isInitialized; }
