/**
 * @file bmp280.cpp
 *
 * ESP-IDF driver for BMP280/BME280 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016 sheinz <https://github.com/sheinz>\n - as bmp280.c
 * Copyright (C) 2018 Ruslan V. Uss <https://github.com/UncleRus> - as bmp280.c
 * Copyright (C) 2020 Michael Reno <michaelreno19@gmail.com> - to fix
 * integration with Diaven driver
 *
 * GPL Licensed as described in the file LICENSE - to match Diaven driver
 */

#include "BMP280.hpp"

#define I2C_FREQ_HZ 1000000  // Max 1MHz for esp-idf

static const char *TAG = "BMP280";

extern BMP280 bme280;

BMP280::BMP280()  // Constructor
    : iscm(),
      dig_t1(0),
      dig_p1(0),
      dig_t2(0),
      dig_t3(0),
      dig_p2(0),
      dig_p3(0),
      dig_p4(0),
      dig_p5(0),
      dig_p6(0),
      dig_p7(0),
      dig_p8(0),
      dig_p9(0),
      dig_h2(0),
      dig_h4(0),
      dig_h5(0),
      dig_h1(0),
      dig_h3(0),
      dig_h6(0),
      id(BME280_CHIP_ID) {}

/**
 * BMP280 registers
 */
#define BMP280_REG_TEMP_XLSB 0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB 0xFB
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_TEMP (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB 0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB 0xF8
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_PRESSURE (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG 0xF5   /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL 0xF4     /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS 0xF3   /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM 0xF2 /* bits: 2-0 osrs_h; */
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_ID 0xD0
#define BMP280_REG_CALIB 0x88
#define BMP280_REG_HUM_CALIB 0x88

#define BMP280_RESET_VALUE 0xB6

bmp280_params_t params;

#if BUILD_TARGET == TARGET_ESP32
static void i2c_bme280_read_task(void *pvParameters) {
#else
static void i2c_bme280_read_task() {
#endif
  log_i("INIT-2", "read task before setup()");
  log_i(TAG, "start task");
  uint16_t dev = 0x76;                    // Sensor address
  bool bme280p = 0x60 == BME280_CHIP_ID;  // always true
  log_i("ESP32:", "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

  log_i(TAG, "calibration...");
  bme280.setup();
  task_delay_ms(1000);

  while (1) {
    float pressure, temperature, humidity;
    if (bme280.read_as_float(dev, &temperature, &pressure, &humidity) !=
        ERROR_OK) {
      log_i("BME280", "Temperature/pressure reading failed\n");
    } else {
      log_i("BME280", "Pressure: %.2f Pa, Temperature: %.2f C", pressure,
            temperature);
    }

    if (bme280p) {
      log_i("BME280", "Humidity: %.2f\n", humidity);
    } else {
      log_i("BME280", "\n");
    }
    task_delay_ms(1000);
  }
  end_task();
}

void BMP280::init(IscMaster *iscm) {
  this->iscm = iscm;
  log_i("INIT-1", "BMP280::init before read task");
  start_task(i2c_bme280_read_task, "bme280 read", 4 * 1024, 1);
}

error_t BMP280::read_register16(uint16_t dev, uint8_t reg, int16_t *r) {
  uint8_t d[] = {0, 0};

  this->iscm->read_register(dev, reg, d, 2);
  *r = d[0] | (d[1] << 8);

  return ERROR_OK;
}

error_t BMP280::read_registerU16(uint16_t dev, uint8_t reg, uint16_t *r) {
  uint8_t d[] = {0, 0};

  this->iscm->read_register(dev, reg, d, 2);
  *r = d[0] | (d[1] << 8);

  return ERROR_OK;
}

error_t BMP280::write_register8(uint16_t dev, uint8_t addr, uint8_t value) {
  return this->iscm->write_register(dev, addr, &value, 1);
}

// static error_t BMP280::read_calibration_data()
error_t BMP280::read_calibration_data() {
  read_registerU16(BMP280_I2C_ADDRESS_0, 0x88, &dig_t1);
  read_register16(BMP280_I2C_ADDRESS_0, 0x8a, &dig_t2);
  read_register16(BMP280_I2C_ADDRESS_0, 0x8c, &dig_t3);
  read_registerU16(BMP280_I2C_ADDRESS_0, 0x8e, &dig_p1);
  read_register16(BMP280_I2C_ADDRESS_0, 0x90, &dig_p2);
  read_register16(BMP280_I2C_ADDRESS_0, 0x92, &dig_p3);
  read_register16(BMP280_I2C_ADDRESS_0, 0x94, &dig_p4);
  read_register16(BMP280_I2C_ADDRESS_0, 0x96, &dig_p5);
  read_register16(BMP280_I2C_ADDRESS_0, 0x98, &dig_p6);
  read_register16(BMP280_I2C_ADDRESS_0, 0x9a, &dig_p7);
  read_register16(BMP280_I2C_ADDRESS_0, 0x9c, &dig_p8);
  read_register16(BMP280_I2C_ADDRESS_0, 0x9e, &dig_p9);

  log_d(TAG, "Calibration data received:");
  log_d(TAG, "dig_T1=%d", dig_t1);
  log_d(TAG, "dig_T2=%d", dig_t2);
  log_d(TAG, "dig_T3=%d", dig_t3);
  log_d(TAG, "dig_P1=%d", dig_p1);
  log_d(TAG, "dig_P2=%d", dig_p2);
  log_d(TAG, "dig_P3=%d", dig_p3);
  log_d(TAG, "dig_P4=%d", dig_p4);
  log_d(TAG, "dig_P5=%d", dig_p5);
  log_d(TAG, "dig_P6=%d", dig_p6);
  log_d(TAG, "dig_P7=%d", dig_p7);
  log_d(TAG, "dig_P8=%d", dig_p8);
  log_d(TAG, "dig_P9=%d", dig_p9);

  return ERROR_OK;
}

error_t BMP280::read_hum_calibration_data() {
  uint16_t h4, h5;

  this->iscm->read_register(BMP280_I2C_ADDRESS_0, 0xa1, &dig_h1, 1);
  read_register16(BMP280_I2C_ADDRESS_0, 0xe1, &dig_h2);
  this->iscm->read_register(BMP280_I2C_ADDRESS_0, 0xe3, &dig_h3, 1);
  read_registerU16(BMP280_I2C_ADDRESS_0, 0xe4, &h4);
  read_registerU16(BMP280_I2C_ADDRESS_0, 0xe5, &h5);
  this->iscm->read_register(BMP280_I2C_ADDRESS_0, 0xe7, (uint8_t *)&dig_h6, 1);

  dig_h4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
  dig_h5 = h5 >> 4;
  log_d(TAG, "Calibration data received:");
  log_d(TAG, "dig_H1=%d", dig_h1);
  log_d(TAG, "dig_H2=%d", dig_h2);
  log_d(TAG, "dig_H3=%d", dig_h3);
  log_d(TAG, "dig_H4=%d", dig_h4);
  log_d(TAG, "dig_H5=%d", dig_h5);
  log_d(TAG, "dig_H6=%d", dig_h6);

  return ERROR_OK;
}

error_t BMP280::init_desc(uint16_t dev, uint8_t addr, i2c_port_t port) {
  if (addr != BMP280_I2C_ADDRESS_0 && addr != BMP280_I2C_ADDRESS_1) {
    log_e(TAG, "Invalid I2C address");
    return ERROR_INVALID_ARG;
  }

  iscm->init(port);

  return ERROR_OK;
}

error_t BMP280::free_desc(uint16_t dev) { return ERROR_OK; }

void BMP280::setup() {
  bmp280_params_t params = {};
  params.mode = BMP280_MODE_NORMAL;
  params.filter = BMP280_FILTER_OFF;
  params.oversampling_pressure = BMP280_STANDARD;
  params.oversampling_temperature = BMP280_STANDARD;
  params.oversampling_humidity = BMP280_STANDARD;
  params.standby = BMP280_STANDBY_250;

  this->iscm->read_register(BMP280_I2C_ADDRESS_0, BMP280_REG_ID, &id, 1);

  if (id != BMP280_CHIP_ID && id != BME280_CHIP_ID) {
    log_e(TAG,
          "Invalid chip ID: expected: 0x%x (BME280) or 0x%x (BMP280) got: 0x%x",
          BME280_CHIP_ID, BMP280_CHIP_ID, id);
  }

  log_i("INIT-3", "setup after check chip id, before soft reset");

  // Soft reset.
  uint8_t reset[1];
  reset[0] = BMP280_RESET_VALUE;

  this->iscm->write_register(BMP280_I2C_ADDRESS_0, BMP280_REG_RESET, reset, 1);

  log_i("INIT-3",
        "setup after soft reset and write register, before while loop");

  // Wait until finished copying over the NVP data.
  while (1) {
    uint8_t status;
    if (!this->iscm->read_register(BMP280_I2C_ADDRESS_0, BMP280_REG_STATUS,
                                   &status, 1) &&
        (status & 1) == 0)
      break;
    task_delay_ms(100);
  }

  log_i("INIT-3", "setup after while loop, before read calibration data");

  read_calibration_data();

  if (id == BME280_CHIP_ID) {
    read_hum_calibration_data();
  }

  log_i("INIT-3", "setup after read calibration data before writing config.");

  uint8_t config[1];
  config[0] = (params.standby << 5) | (params.filter << 2);
  log_d(TAG, "Writing config reg=%x", config[0]);

  log_i("INIT-3", "setup after writing config, before write register");

  this->iscm->write_register(BMP280_I2C_ADDRESS_0, BMP280_REG_CONFIG, config,
                             1);

  if (params.mode == BMP280_MODE_FORCED) {
    params.mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
  }

  uint8_t ctrl[1];
  ctrl[0] = (params.oversampling_temperature << 5) |
            (params.oversampling_pressure << 2) | (params.mode);

  if (id == BME280_CHIP_ID) {
    // Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
    uint8_t ctrl_hum = params.oversampling_humidity;
    log_d(TAG, "Writing ctrl hum reg=%x", ctrl_hum);
    this->write_register8(BMP280_I2C_ADDRESS_0, BMP280_REG_CTRL_HUM, ctrl_hum);
  }

  log_d(TAG, "Writing ctrl reg=%x", ctrl[0]);
  this->iscm->write_register(BMP280_I2C_ADDRESS_0, BMP280_REG_CTRL, ctrl, 1);
}

error_t BMP280::force_measurement(uint16_t dev) {
  uint8_t ctrl[1];
  this->iscm->read_register(BMP280_I2C_ADDRESS_0, BMP280_REG_CTRL, &ctrl[0], 1);
  ctrl[0] &= ~0b11;  // clear two lower bits
  ctrl[0] |= BMP280_MODE_FORCED;
  log_d(TAG, "Writing ctrl reg=%x", ctrl[0]);
  this->iscm->write_register(BMP280_I2C_ADDRESS_0, BMP280_REG_CTRL, ctrl, 1);

  return ERROR_OK;
}

error_t BMP280::check_if_measuring(uint16_t dev, bool *busy) {
  const uint8_t regs[2] = {BMP280_REG_STATUS, BMP280_REG_CTRL};
  uint8_t status[2];
  this->iscm->write_register(dev, regs[0], status, 1);
  this->iscm->write_register(dev, regs[1], status, 1);
  // this->iscm->read_bytes(dev, status, 2);

  // Check mode - FORCED means BM280 is busy (it switches to SLEEP mode when
  // finished) Additionally, check 'measuring' bit in status register
  *busy = ((status[1] & 0b11) == BMP280_MODE_FORCED) || (status[0] & (1 << 3));

  return ERROR_OK;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
int32_t BMP280::compensate_temperature(uint16_t dev, int32_t adc_temp,
                                       int32_t *fine_temp) {
  int32_t var1, var2;

  var1 = ((((adc_temp >> 3) - ((int32_t)dig_t1 << 1))) * (int32_t)dig_t2) >> 11;
  var2 = (((((adc_temp >> 4) - (int32_t)dig_t1) *
            ((adc_temp >> 4) - (int32_t)dig_t1)) >>
           12) *
          (int32_t)dig_t3) >>
         14;

  *fine_temp = var1 + var2;
  return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
uint32_t BMP280::compensate_pressure(uint16_t dev, int32_t adc_press,
                                     int32_t fine_temp) {
  int64_t var1, var2, p;

  var1 = (int64_t)fine_temp - 128000;
  var2 = var1 * var1 * (int64_t)dig_p6;
  var2 = var2 + ((var1 * (int64_t)dig_p5) << 17);
  var2 = var2 + (((int64_t)dig_p4) << 35);
  var1 =
      ((var1 * var1 * (int64_t)dig_p3) >> 8) + ((var1 * (int64_t)dig_p2) << 12);
  var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dig_p1) >> 33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }

  p = 1048576 - adc_press;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)dig_p9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)dig_p8 * p) >> 19;

  p = ((p + var1 + var2) >> 8) + ((int64_t)dig_p7 << 4);
  return p;
}

/**
 * Compensation algorithm is taken from BME280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
uint32_t BMP280::compensate_humidity(uint16_t dev, int32_t adc_hum,
                                     int32_t fine_temp) {
  int32_t v_x1_u32r;

  v_x1_u32r = fine_temp - (int32_t)76800;
  v_x1_u32r = ((((adc_hum << 14) - ((int32_t)dig_h4 << 20) -
                 ((int32_t)dig_h5 * v_x1_u32r)) +
                (int32_t)16384) >>
               15) *
              (((((((v_x1_u32r * (int32_t)dig_h6) >> 10) *
                   (((v_x1_u32r * (int32_t)dig_h3) >> 11) + (int32_t)32768)) >>
                  10) +
                 (int32_t)2097152) *
                    (int32_t)dig_h2 +
                8192) >>
               14);
  v_x1_u32r =
      v_x1_u32r -
      (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (int32_t)dig_h1) >> 4);
  v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
  v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
  return v_x1_u32r >> 12;
}

error_t BMP280::read_fixed_data(uint16_t dev, int32_t *temperature,
                                uint32_t *pressure, uint32_t *humidity) {
  int32_t adc_pressure;
  int32_t adc_temp;
  uint8_t data[8];

  // Only the BME280 supports reading the humidity.
  if (this->readID() != BME280_CHIP_ID) {
    if (humidity) *humidity = 0;
    humidity = NULL;
  }

  // Need to read in one sequence to ensure they match.
  // size_t size = humidity ? 8 : 6;
  uint16_t size = humidity ? 8 : 6;
  this->iscm->read_register(BMP280_I2C_ADDRESS_0, 0xf7, data, size);

  adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
  adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;
  log_d(TAG, "ADC temperature: %d", adc_temp);
  log_d(TAG, "ADC pressure: %d", adc_pressure);

  int32_t fine_temp;
  *temperature = compensate_temperature(dev, adc_temp, &fine_temp);
  *pressure = compensate_pressure(dev, adc_pressure, fine_temp);

  if (humidity) {
    int32_t adc_humidity = data[6] << 8 | data[7];
    log_d(TAG, "ADC humidity: %d", adc_humidity);
    *humidity = compensate_humidity(dev, adc_humidity, fine_temp);
  }

  return ERROR_OK;
}

error_t BMP280::read_as_float(uint16_t dev, float *temperature, float *pressure,
                              float *humidity) {
  int32_t fixed_temperature;
  uint32_t fixed_pressure;
  uint32_t fixed_humidity;
  read_fixed_data(dev, &fixed_temperature, &fixed_pressure,
                  humidity ? &fixed_humidity : NULL);
  *temperature = (float)fixed_temperature / 100;
  *pressure = (float)fixed_pressure / 256;
  if (humidity) *humidity = (float)fixed_humidity / 1024;

  return ERROR_OK;
}
