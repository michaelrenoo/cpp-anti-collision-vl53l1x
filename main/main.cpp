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
VL53L1X vl53l0;
VL53L1X vl53l1;
VL53L1X vl53l2;
VL53L1X vl53l3;
FPS fps;
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
    uint8_t mavBuff0[MAV_OUT_MAX_LEN];
    uint8_t mavBuff1[MAV_OUT_MAX_LEN];
    uint8_t mavBuff2[MAV_OUT_MAX_LEN];
    uint8_t mavBuff3[MAV_OUT_MAX_LEN];
    int msgLen0 = 0;
    int msgLen1 = 0;
    int msgLen2 = 0;
    int msgLen3 = 0;

    msgLen0 = mav_generate_distance_sensor(
        mavBuff0, MAV_OUT_MAX_LEN, 1,
        MAV_SENSOR_ROTATION_NONE,  // 1st sensor - towards USB in Pi
        (vl53l0.getDistance() / 10));
    msgLen1 = mav_generate_distance_sensor(
        mavBuff1, MAV_OUT_MAX_LEN, 2,
        MAV_SENSOR_ROTATION_YAW_90,  // 2nd sensor - towards power connector in
                                     // Pi
        (vl53l1.getDistance() / 10));
    msgLen2 = mav_generate_distance_sensor(
        mavBuff2, MAV_OUT_MAX_LEN, 3,
        MAV_SENSOR_ROTATION_YAW_180,  // 3rd sensor - backwards from USB in Pi
        (vl53l2.getDistance() / 10));
    msgLen3 = mav_generate_distance_sensor(
        mavBuff3, MAV_OUT_MAX_LEN, 4,
        MAV_SENSOR_ROTATION_YAW_270,  // 4th sensor - backwards from power
                                      // connector in Pi
        (vl53l3.getDistance() / 10));

    fps.send_bytes(mavBuff0, msgLen0);
    fps.send_bytes(mavBuff1, msgLen1);
    fps.send_bytes(mavBuff2, msgLen2);
    fps.send_bytes(mavBuff3, msgLen3);
    delay.wait_for(200);
  }
  end_task();
}

#if BUILD_TARGET == TARGET_ESP32
static void i2c_vl53l1x_read_task(void *pvParameters) {
#else
static void i2c_vl53l1x_read_task() {
#endif
  static const char *TAG = "Sensor Task";
  log_i(TAG, "start task");
  task_delay_ms(1000);

  // while (1) {
  //   vl53l0.startMeasurement();
  //   while (vl53l0.newDataReady() == false) {
  //     task_delay_ms(10);
  //   }
  //   log_i(TAG, "old address: %d", vl53l0.getAddress());
  //   log_i(TAG, "dist: %d", vl53l0.getDistance());
  //   task_delay_ms(200);
  //   vl53l0.setAddress(vl53l0.getAddress() / 2);
  //   task_delay_ms(200);
  //   log_i(TAG, "new address: %d", vl53l0.getAddress());
  //   task_delay_ms(2000);
  //   vl53l0.softReset();
  //   log_i(TAG, "dist: %d", vl53l0.getDistance());
  //   task_delay_ms(2000);
  //   break;
  // }

  log_i(TAG, "Address Sensor 1: %d", vl53l0.getAddress());
  log_i(TAG, "Address Sensor 2: %d", vl53l1.getAddress());
  log_i(TAG, "Address Sensor 3: %d", vl53l2.getAddress());
  log_i(TAG, "Address Sensor 4: %d", vl53l3.getAddress());
  // vl53l0.softReset();
  // vl53l1.softReset();
  // vl53l2.softReset();
  // vl53l3.softReset();

  // vl53l0.startMeasurement(vl53l0.getAddress());
  // vl53l1.startMeasurement();
  // vl53l2.startMeasurement();
  // vl53l3.startMeasurement();

  while (1) {
    while (vl53l0.newDataReady() == false && vl53l1.newDataReady() == false &&
           vl53l2.newDataReady() == false && vl53l3.newDataReady() == false) {
      task_delay_ms(10);
    }

    // TODO: Add counter to count how many times per second the distance is
    // measured. and log it with another task. -> count is then a global var
    log_i(TAG, "Dist Sensor 1: %d", vl53l0.getDistance());
    log_i(TAG, "Dist Sensor 2: %d", vl53l1.getDistance());
    log_i(TAG, "Dist Sensor 3: %d", vl53l2.getDistance());
    log_i(TAG, "Dist Sensor 4: %d", vl53l3.getDistance());
    log_i(TAG, "---------------------------------------");
    task_delay_ms(200);
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

// Set all XSHUT as low
#if XSHUT0 >= 0
  init_gpio_out(XSHUT0);
  set_gpio_out(XSHUT0, false);
#endif
#if XSHUT1 >= 0
  init_gpio_out(XSHUT1);
  set_gpio_out(XSHUT1, false);
#endif
#if XSHUT2 >= 0
  init_gpio_out(XSHUT2);
  set_gpio_out(XSHUT2, false);
#endif
#if XSHUT3 >= 0
  init_gpio_out(XSHUT3);
  set_gpio_out(XSHUT3, false);
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

  //********************************
  //* init i2c                     *
  //********************************

  // init I2C master on I2C_NUM_0
  isc_master_init(I2C_NUM_1, PIN_NUM_I2C0_SDA, PIN_NUM_I2C0_SCL);
  iscm.init(I2C_NUM_1);
  task_delay_ms(2000);

  for (int ToF = 0; ToF <= 3; ToF++) {
    uint8_t newAddress = 21 + ToF;
    switch (ToF) {
      case 0:
        // set_gpio_out(XSHUT0, false);
        set_gpio_out(XSHUT0, true);
        task_delay_ms(200);
        vl53l0.init(&iscm);
        log_i(TAG, "VL53L1X 1 Address: %u", vl53l0.getAddress());
        vl53l0.setAddress(newAddress);
        vl53l0.softReset();
        vl53l0.startMeasurement(newAddress);
        log_i(TAG, "VL53L1X 1 Address: %u", vl53l0.getAddress());
        // vl53l0.softReset();
        break;
      case 1:
        // set_gpio_out(XSHUT1, false);
        set_gpio_out(XSHUT1, true);
        task_delay_ms(200);
        vl53l1.init(&iscm);
        log_i(TAG, "VL53L1X 2 Address: %u", vl53l1.getAddress());
        vl53l1.setAddress(newAddress);
        vl53l1.softReset();
        vl53l1.startMeasurement(newAddress);
        log_i(TAG, "VL53L1X 2 Address: %u", vl53l1.getAddress());
        // vl53l1.softReset();
        break;
      case 2:
        // set_gpio_out(XSHUT2, false);
        set_gpio_out(XSHUT2, true);
        task_delay_ms(200);
        vl53l2.init(&iscm);
        log_i(TAG, "VL53L1X 3 Address: %u", vl53l2.getAddress());
        vl53l2.setAddress(newAddress);
        vl53l2.softReset();
        vl53l2.startMeasurement(newAddress);
        log_i(TAG, "VL53L1X 3 Address: %u", vl53l2.getAddress());
        // vl53l2.softReset();
        break;
      case 3:
        // set_gpio_out(XSHUT3, false);
        set_gpio_out(XSHUT3, true);
        task_delay_ms(200);
        vl53l3.init(&iscm);
        log_i(TAG, "VL53L1X 4 Address: %u", vl53l3.getAddress());
        vl53l3.setAddress(newAddress);
        vl53l3.softReset();
        vl53l3.startMeasurement(newAddress);
        log_i(TAG, "VL53L1X 4 Address: %u", vl53l3.getAddress());
        // vl53l3.softReset();
        break;
    }
  }

  //********************************
  //* init FPS                     *
  //********************************

  // init UART 1
  assign_uart_usage(UART_NUM_2, 115200, UART2_TX_PIN, UART2_RX_PIN);

  // bme280.init(&iscm);

  //********************************
  //* init DataHandler             *
  //********************************

  if (fps.is_initialized()) {
    mav.init();
  }

  // start vl53l1x read task
  start_task(i2c_vl53l1x_read_task, "vl53l1x", 4 * 1024, 1);
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
      uint16_t dist = vl53l0.getDistance();
      // start_task(i2c_vl53l1x_read_task, "vl53l1x", 4 * 1024, 1);
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
