/*
 *  Created on: 15.01.2021
 *      Author: Juri Bieler, Michael Reno
 *      Email: michaelreno19@gmail.com
 */

// my libs
// To write csv file
// #include <fstream>
// #include <iostream>
// #include <string>
// using namespace std;

#include "./def.hpp"
#include "driver/driver.hpp"
#include "interface/FPS.hpp"
#include "interface/IscMaster.hpp"
#include "interface/MavlinkCom.hpp"
#include "interface/VL53L1X.h"

static const char *TAG = "Main";

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

uint16_t count = 0;
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

/*
  Send distance_sensor MAVLink message
*/
#if BUILD_TARGET == TARGET_ESP32
static void dist_task(void *pvParameters) {
#else
static void dist_task() {
#endif
  DelayUntil delay = DelayUntil();
  while (1) {
    // Declare buffers and message lengths
    uint8_t mavBuff0[MAV_OUT_MAX_LEN];
    uint8_t mavBuff1[MAV_OUT_MAX_LEN];
    uint8_t mavBuff2[MAV_OUT_MAX_LEN];
    uint8_t mavBuff3[MAV_OUT_MAX_LEN];
    int msgLen0 = 0;
    int msgLen1 = 0;
    int msgLen2 = 0;
    int msgLen3 = 0;

    // Initialise lengths as MAVLink messages for each sensors
    msgLen0 = mav_generate_distance_sensor(
        mavBuff0, MAV_OUT_MAX_LEN, 1,
        MAV_SENSOR_ROTATION_NONE,  // 1st sensor - towards USB in Pi
        (vl53l0.getDistance() / 10));
    msgLen1 = mav_generate_distance_sensor(
        mavBuff1, MAV_OUT_MAX_LEN, 2,
        MAV_SENSOR_ROTATION_YAW_270,  // 2nd sensor - towards power connector in
                                      // Pi (or backwards from power if Pi is
                                      // flipped)
        (vl53l1.getDistance() / 10));
    msgLen2 = mav_generate_distance_sensor(
        mavBuff2, MAV_OUT_MAX_LEN, 3,
        MAV_SENSOR_ROTATION_YAW_180,  // 3rd sensor - backwards from USB in Pi
        (vl53l2.getDistance() / 10));
    msgLen3 = mav_generate_distance_sensor(
        mavBuff3, MAV_OUT_MAX_LEN, 4,
        MAV_SENSOR_ROTATION_YAW_90,  // 4th sensor - backwards from power
                                     // connector in Pi (or towards from power
                                     // if Pi is flipped)
        (vl53l3.getDistance() / 10));

    // Send messages to the flight controller
    fps.send_bytes(mavBuff0, msgLen0);
    fps.send_bytes(mavBuff1, msgLen1);
    fps.send_bytes(mavBuff2, msgLen2);
    fps.send_bytes(mavBuff3, msgLen3);
    delay.wait_for(200);  // Wait to prevent thread blocking
  }
  end_task();
}

/*
  Task to read distances from all 4 sensors
*/
#if BUILD_TARGET == TARGET_ESP32
static void i2c_vl53l1x_read_task(void *pvParameters) {
#else
static void i2c_vl53l1x_read_task() {
#endif
  static const char *TAG = "Sensor Task";
  log_i(TAG, "start task");  // Log to start message
  task_delay_ms(1000);

  // Print out sensor addresses after setAddress()
  log_i(TAG, "Address Sensor 1: %d", vl53l0.getAddress());
  log_i(TAG, "Address Sensor 2: %d", vl53l1.getAddress());
  log_i(TAG, "Address Sensor 3: %d", vl53l2.getAddress());
  log_i(TAG, "Address Sensor 4: %d", vl53l3.getAddress());

  // Export csv according to the given file name:
  // string name;
  // cin >> name;
  // name.append(".csv");
  // ofstream csvFile(name);

  // Check if all sensors have new data ready
  while (1) {
    while (vl53l0.newDataReady() == false && vl53l1.newDataReady() == false &&
           vl53l2.newDataReady() == false && vl53l3.newDataReady() == false) {
      task_delay_ms(10);  // If not, wait for 10 ms before testing again
    }

    // If yes, get distances from all sensors and print it out
    log_i(TAG, "Dist Sensor 1: %d", vl53l0.getDistance());
    log_i(TAG, "Dist Sensor 2: %d", vl53l1.getDistance());
    log_i(TAG, "Dist Sensor 3: %d", vl53l2.getDistance());
    log_i(TAG, "Dist Sensor 4: %d", vl53l3.getDistance());
    // csvFile << vl53l3.getDistance() << endl;  // Export distance from one
    // sensor to the .csv file
    log_i(TAG, "---------------------------------------");
    count++;            // Increase global counter
    task_delay_ms(20);  // Give time so the thread is not blocked
  }

  end_task();
}

/*
  Counter task
*/
#if BUILD_TARGET == TARGET_ESP32
static void count_task(void *pvParameters) {
#else
static void count_task() {
#endif
  // ofstream csvFile("EntfernungProSekunde1.csv");  // Create .csv file to
  // print out counter
  while (1) {
    static const char *TAG = "Count Task";
    task_delay_ms(1000);
    log_i(TAG, "Counter per second: %d",
          count);  // Print out counter per second (= 1000 ms)
    // csvFile << count << endl;
    log_i(TAG, "---------------------------------------");
    count = 0;  // Reset global counter each second
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

  // init other hardware
  init_onboard_led();
#if LED2_GPIO >= 0
  init_gpio_out(LED2_GPIO);
  set_gpio_out(LED2_GPIO, false);
#endif

// Set all XSHUT Pins as low
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

  // Set new address for each sensors
  for (int ToF = 0; ToF <= 3; ToF++) {
    uint8_t newAddress = 21 + ToF;  // Set newAddress to a range of uint8 from
                                    // 21 to the amount of sensors
    switch (ToF) {
      case 0:
        set_gpio_out(XSHUT0, true);  // Set XSHUT back to HIGH
        task_delay_ms(200);
        vl53l0.init(&iscm);  // Initialise the sensor
        log_i(TAG, "VL53L1X 1 Address: %u",
              vl53l0.getAddress());     // Print out old address
        vl53l0.setAddress(newAddress);  // Set new address
        vl53l0.softReset();  // Reset the sensor to save the applied change
        vl53l0.startMeasurement(
            newAddress);  // Start measurement with the sensor
        log_i(TAG, "VL53L1X 1 Address: %u",
              vl53l0.getAddress());  // Print out new address
        break;
      case 1:
        set_gpio_out(XSHUT1, true);
        task_delay_ms(200);
        vl53l1.init(&iscm);
        log_i(TAG, "VL53L1X 2 Address: %u", vl53l1.getAddress());
        vl53l1.setAddress(newAddress);
        vl53l1.softReset();
        vl53l1.startMeasurement(newAddress);
        log_i(TAG, "VL53L1X 2 Address: %u", vl53l1.getAddress());
        break;
      case 2:
        set_gpio_out(XSHUT2, true);
        task_delay_ms(200);
        vl53l2.init(&iscm);
        log_i(TAG, "VL53L1X 3 Address: %u", vl53l2.getAddress());
        vl53l2.setAddress(newAddress);
        vl53l2.softReset();
        vl53l2.startMeasurement(newAddress);
        log_i(TAG, "VL53L1X 3 Address: %u", vl53l2.getAddress());
        break;
      case 3:
        set_gpio_out(XSHUT3, true);
        task_delay_ms(200);
        vl53l3.init(&iscm);
        log_i(TAG, "VL53L1X 4 Address: %u", vl53l3.getAddress());
        vl53l3.setAddress(newAddress);
        vl53l3.softReset();
        vl53l3.startMeasurement(newAddress);
        log_i(TAG, "VL53L1X 4 Address: %u", vl53l3.getAddress());
        break;
    }
  }

  //********************************
  //* init FPS                     *
  //********************************

  // init UART 1
  assign_uart_usage(UART_NUM_2, 115200, UART2_TX_PIN, UART2_RX_PIN);

  //********************************
  //* init DataHandler             *
  //********************************

  if (fps.is_initialized()) {
    mav.init();  // Init MAVLink only when the UART connection has been
                 // established
  }

  // start vl53l1x read task
  start_task(i2c_vl53l1x_read_task, "vl53l1x", 4 * 1024, 1);
  // start dist send task
  start_task(dist_task, "dist_task", 8 * 1024, 1);
  // start count task
  start_task(count_task, "count_task", 8 * 1024, 1);

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
