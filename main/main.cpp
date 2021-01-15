/*
 *  Created on: 16 Dec 2017
 *      Author: Juri Bieler
 */

// my libs
#include "./def.hpp"
#include "driver/driver.hpp"
#include "interface/BMP280.hpp"
#include "interface/FPS.hpp"
#include "interface/IscMaster.hpp"
#include "interface/MavlinkCom.hpp"
#include "interface/VL53L1X.h"

static const char *TAG = "Main";

// #define DATA_LENGTH 512

#define BUTTON_PRESS_FOR_SHUTDOWN_TIME_MS 3000

#if BUILD_TARGET == TARGET_ESP32
extern "C" {
void app_main(void);
}
#endif

IscMaster iscm;

MavlinkCom mav;
FPS fps;
VL53L1X vl53;
BMP280 bme280;

uint8_t printBuff[256];

int log_handle(const char *format, va_list arg) {
  // print to DiAvEnCom if requested...
  uint8_t buff[128];
  int cmdLen = vsprintf(reinterpret_cast<char *>(buff), format, arg);
  // com.send_debug_str(buff, cmdLen);
  // if not already sent
#if BUILD_TARGET == TARGET_PI3
  // since for pi Debug is only for logging we want to print it as well
  vprintf(format, arg);
#endif
  return cmdLen;
}

void assign_uart_usage(uart_port_t port_nr, uint32_t baud_bits_sec,
                       uart_name_t tx_pin, gpio_num_t rx_pin) {
  // init UART
  uart_init(port_nr, tx_pin, rx_pin, baud_bits_sec);
  fps.init(port_nr);
  log_i(TAG, "init uart%d as fps", port_nr);
}

#if BUILD_TARGET == TARGET_ESP32
static void dist_task(void *pvParameters) {
#else
static void dist_task() {
#endif
  DelayUntil delay = DelayUntil();
  while (1) {
    uint8_t mavBuff[MAV_OUT_MAX_LEN];
    int msgLen = 0;
    msgLen = mav_generate_distance_sensor(mavBuff, MAV_OUT_MAX_LEN, 1,
                                          MAV_SENSOR_ROTATION_YAW_90, 42);
    fps.send_bytes(mavBuff, msgLen);
    delay.wait_for(200);
  }
  end_task();
}

#if BUILD_TARGET == TARGET_ESP32
void app_main(void) {
#else
int main() {
#endif

  //********************************
  //* init printouts               *
  //********************************

  log_i(TAG, "this chip uid: %llu", get_efuse_mac());
  log_i(TAG, "software version: %s", FW_VERSION_NAME);
  log_i(TAG, "compile time: %s %s", __DATE__, __TIME__);

  //********************************
  //* init hw pins & driver        *
  //********************************

  init_gpio_isr_service();

  // Initialize NVS
  // init_nvs();

  // init other hardware
  init_onboard_led();
#if LED2_GPIO >= 0
  init_gpio_out(LED2_GPIO);
  set_gpio_out(LED2_GPIO, false);
#endif

  set_led(true);
  init_onboard_button();

  // init_gpio_out(ENG_OFF_TRIGGER_GPIO);
  // init_gpio_out(PARASHUTE_TRIGGER_GPIO);

  //********************************
  //* init UARTs                   *
  //********************************

#if BUILD_TARGET == TARGET_ESP32
  // init UART 0
  uart_init(UART_NUM_0, UART0_TX_PIN, UART0_RX_PIN, UART0_BAUD_RATE);
  log_i(TAG, "init uart0 as debug port");
#endif

  log_set_vprintf(log_handle);

  // init UART 1
  assign_uart_usage(UART_NUM_2, 115200, UART2_TX_PIN, UART2_RX_PIN);

  //********************************
  //* init i2c                     *
  //********************************

  // init I2C master on I2C_NUM_0
  isc_master_init(I2C_NUM_1, PIN_NUM_I2C0_SDA, PIN_NUM_I2C0_SCL);
  iscm.init(I2C_NUM_1);
  task_delay_ms(2000);

  vl53.init(&iscm);
  // bme280.init(&iscm);

  //********************************
  //* init DataHandler             *
  //********************************

  if (fps.is_initialized()) {
    mav.init();
  }

  // start dist send task
  start_task(dist_task, "dist_task", 8 * 1024, 1);

  //********************************
  //* complete boot up             *
  //********************************

  log_i(TAG, "init complete!");
  set_led(false);

  //********************************
  //* run user but function loop   *
  //********************************
#if BUILD_VERSION == VERSION_CC
  bool gearUp = false;
#endif
  while (1) {
    if (button_is_pressed()) {
      while (button_is_pressed()) {
        task_delay_ms(100);
      }
      uint16_t dist = vl53.getDistance();
      log_i(TAG, "test, %d", dist);
    }
    task_delay_ms(200);
  }

#if BUILD_TARGET == TARGET_ESP32
  return;
#else
  return 0;
#endif
}
