

#include "VL53L1X.h"

static const char *TAG = "VL53L1X";

extern IscMaster iscm;
extern VL53L1X vl53l0;

// This is 135 bytes to be written every time to the VL53L1X to initiate a
// measurement 0x29 is written to memory location 0x01, which is the register
// for the I2C address which is indeed 0x29. So this makes sense. We could
// evaluate the default register settings of a given device against this set of
// bytes and write only the ones that are different but it's faster, easier, and
// perhaps fewer code words to write the config block as is. The block was
// obtained via inspection of the ST P-NUCLEO-53L1A1
uint8_t configBlock[] = {
    0x29, 0x02, 0x10, 0x00, 0x28, 0xBC, 0x7A, 0x81,  // 8
    0x80, 0x07, 0x95, 0x00, 0xED, 0xFF, 0xF7, 0xFD,  // 16
    0x9E, 0x0E, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00,  // 24
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00,  // 32
    0x28, 0x00, 0x0D, 0x0A, 0x00, 0x00, 0x00, 0x00,  // 40
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,  // 48
    0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,  // 56
    0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x02,  // 64
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00,  // 72
    0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x02, 0x00,  // 80
    0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF,  // 88
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91, 0x0F,  // 96
    0x00, 0xA5, 0x0D, 0x00, 0x80, 0x00, 0x0C, 0x08,  // 104
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x10, 0x00,  // 112
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F,  // 120
    0x0D, 0x0E, 0x0E, 0x01, 0x00, 0x02, 0xC7, 0xFF,  // 128
    0x8B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40         // 129 - 135 (0x81 - 0x87)
};

// Check to see if sensor is responding
// Set sensor up for 2.8/3.3V I2C
// Return true if succesful
bool VL53L1X::init(IscMaster *iscm) {
  if (this->deviceAddress == NULL) {
    this->deviceAddress = defaultAddress_VL53L1X;
  }
  // this->deviceAddress = 0x29;
  this->iscm = iscm;

  // Check the device ID
  uint8_t buff[2] = {0, 0};
  iscm->read_register16(deviceAddress, VL53L1_IDENTIFICATION__MODEL_ID, buff,
                        2);
  // log_i(TAG, "test: %x %x", buff[0], buff[1]);
  uint16_t modelID = buff[0] << 8 | buff[1];
  uint8_t count = 0;
  while (modelID != 0xEACC) {
    iscm->read_register16(deviceAddress, VL53L1_IDENTIFICATION__MODEL_ID, buff,
                          2);
    modelID = buff[0] << 8 | buff[1];
    log_i(TAG, "test: %x %x = %d", buff[0], buff[1], modelID);
    task_delay_ms(1000);
    if (count > 10) {
      log_e(TAG, "could not connect to senor");
      return false;
    }
    count++;
  }

  softReset();

  // Polls the bit 0 of the FIRMWARE__SYSTEM_STATUS register to see if the
  // firmware is ready
  int counter = 0;
  uint8_t res[2] = {0};
  iscm->read_register16(deviceAddress, VL53L1_FIRMWARE__SYSTEM_STATUS, res, 2);
  uint16_t res16 = res[0] << 8 | res[1];
  while ((res[0] & 0x01) == 0) {
    if (counter++ == 100) {
      return false;  // Sensor timed out
    }
    iscm->read_register16(deviceAddress, VL53L1_FIRMWARE__SYSTEM_STATUS, res,
                          2);
    res16 = res[0] << 8 | res[1];
    task_delay_ms(10);
  }

  // Set I2C to 2.8V mode. In this mode 3.3V I2C is allowed.
  uint16_t result = 0;
  uint8_t buffer1[2] = {0};
  iscm->read_register16(deviceAddress, VL53L1_PAD_I2C_HV__EXTSUP_CONFIG,
                        buffer1, 2);
  result = buffer1[0] << 8 | buffer1[1];

  result = (result & 0xFE) | 0x01;
  uint8_t writeBuff[2] = {0};
  writeBuff[2] = result >> 8;
  writeBuff[3] = result & 0x00FF;
  iscm->write_register16(deviceAddress, VL53L1_PAD_I2C_HV__EXTSUP_CONFIG,
                         writeBuff, 2);

  // start_task(i2c_vl53l1x_read_task, "vl53l1x", 4 * 1024, 1);
  return true;  // Sensor online!
}

// Write the configuration block with a max of I2C_BUFFER_LENGTH bytes at a time
// Offset allows us to start at a location within the configBlock array
// This is the main function that setups up the VL53L1X to take measurements
// This was obtained by inspecting the example software from ST and by
// capturing I2C trace on ST Nucleo demo board
void VL53L1X::startMeasurement(uint8_t newAddress, uint8_t offset) {
  unsigned int result;
  uint8_t address = 1 + offset;  // Start at memory location 0x01, add offset
  int length = 137;
  uint8_t buffer[length] = {0};
  buffer[0] = 0x00;
  buffer[1] = 0x01;
  for (int x = 2; x < 135; x++) {
    buffer[x] = configBlock[x - 2];
  }
  buffer[2] = newAddress;
  deviceAddress = newAddress;
  buffer[135] = 0x01;
  buffer[136] = 0x40;
  for (uint8_t x = 0; x < 137; x++) {
    result = buffer[x];
  }
  iscm->send_bytes(this->deviceAddress, buffer, length);
}

// Polls the measurement completion bit
bool VL53L1X::newDataReady() {
  uint8_t result[1] = {0};
  iscm->read_register16(this->deviceAddress, VL53L1_GPIO__TIO_HV_STATUS, result,
                        1);
  return result[0] != 03;
}

// Reset sensor via software
void VL53L1X::softReset() {
  uint8_t val = 0;
  iscm->write_register16(this->deviceAddress, VL53L1_SOFT_RESET, &val, 1);
  delay(1);  // Driver uses 100us
  // Exit reset
  val = 0x01;
  iscm->write_register16(this->deviceAddress, VL53L1_SOFT_RESET, &val, 1);
}

// The sensor has 44 bytes of various datums.
// See VL53L1_i2c_decode_system_results() in vl53l1_register_funcs.c for the
// decoder ringer Start from memory address VL53L1_RESULT__INTERRUPT_STATUS 0:
// result__interrupt_status, 1: result__range_status,
// VL53L1_RESULT__RANGE_STATUS 2: result__report_status,
// VL53L1_RESULT__REPORT_STATUS 3: result__stream_count 4:
// result__dss_actual_effective_spads_sd0 6:
// result__peak_signal_count_rate_mcps_sd0 8:
// result__ambient_count_rate_mcps_sd0 10: result__sigma_sd0 12:
// result__phase_sd0 14: result__final_crosstalk_corrected_range_mm_sd0 16:
// result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 18:
// result__mm_inner_actual_effective_spads_sd0 20:
// result__mm_outer_actual_effective_spads_sd0 22:
// result__avg_signal_count_rate_mcps_sd0 24:
// result__dss_actual_effective_spads_sd1 26:
// result__peak_signal_count_rate_mcps_sd1 28:
// result__ambient_count_rate_mcps_sd1 30: result__sigma_sd1 32:
// result__phase_sd1 34: result__final_crosstalk_corrected_range_mm_sd1 36:
// result__spare_0_sd1 38: result__spare_1_sd1 40: result__spare_2_sd1 42:
// result__spare_3_sd1 44: result__thresh_info

// Get the 'final' results from measurement
uint16_t VL53L1X::getDistance() {
  uint16_t val = 0;
  uint8_t buff[2] = {0};
  iscm->read_register16(deviceAddress,
                        VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                        buff, 2);
  val = buff[0] << 8 | buff[1];
  return val;
}

// Get signal rate
// This seems to be a number representing the quality of the measurement, the
// number of SPADs used perhaps
uint16_t VL53L1X::getSignalRate() {
  uint16_t val = 0;
  uint8_t buff[2] = {0};
  iscm->read_register16(
      this->deviceAddress,
      VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, buff,
      2);
  val = buff[0] << 8 | buff[1];
  // float signalRate = (float)reading/65536.0;
  return val;
}

// Adapted from ST API
// 0 is short range, 1 is mid range, 2 is long range.
void VL53L1X::setDistanceMode(uint8_t mode) {
  uint8_t periodA;
  uint8_t periodB;
  uint8_t phaseHigh;
  uint8_t phaseInit;
  switch (mode) {
    case 0:
      periodA = 0x07;
      periodB = 0x05;
      phaseHigh = 0x38;
      phaseInit = 6;
      break;
    case 1:
      periodA = 0x0B;
      periodB = 0x09;
      phaseHigh = 0x78;
      phaseInit = 10;
      break;
    case 2:
      periodA = 0x0F;
      periodB = 0x0D;
      phaseHigh = 0xB8;
      phaseInit = 14;
      break;
    // If user inputs wrong range, we default to long range
    default:
      periodA = 0x0F;
      periodB = 0x0D;
      phaseHigh = 0xB8;
      phaseInit = 14;
      break;
  }
  // timing
  iscm->write_register16(this->deviceAddress,
                         VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, &periodA, 1);
  iscm->write_register16(this->deviceAddress,
                         VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, &periodB, 1);
  iscm->write_register16(this->deviceAddress,
                         VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, &phaseHigh, 1);

  // dynamic
  iscm->write_register16(this->deviceAddress, VL53L1_SD_CONFIG__WOI_SD0,
                         &periodA, 1);
  iscm->write_register16(this->deviceAddress, VL53L1_SD_CONFIG__WOI_SD1,
                         &periodB, 1);
  iscm->write_register16(this->deviceAddress,
                         VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, &phaseInit, 1);
  iscm->write_register16(this->deviceAddress,
                         VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, &phaseInit, 1);
  _distanceMode = mode;
}

uint8_t VL53L1X::getDistanceMode() { return _distanceMode; }

// The sensor returns a range status that needs to be re-mapped to one of 9
// different statuses This does that.
uint8_t VL53L1X::getRangeStatus() {
#define VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE (1)
#define VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE (2)
#define VL53L1_DEVICEERROR_NOVHVVALUEFOUND (3)
#define VL53L1_DEVICEERROR_MSRCNOTARGET (4)
#define VL53L1_DEVICEERROR_RANGEPHASECHECK (5)
#define VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK (6)
#define VL53L1_DEVICEERROR_PHASECONSISTENCY (7)
#define VL53L1_DEVICEERROR_MINCLIP (8)
#define VL53L1_DEVICEERROR_RANGECOMPLETE (9)
#define VL53L1_DEVICEERROR_ALGOUNDERFLOW (10)
#define VL53L1_DEVICEERROR_ALGOOVERFLOW (11)
#define VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD (12)
#define VL53L1_DEVICEERROR_USERROICLIP (13)
#define VL53L1_DEVICEERROR_REFSPADCHARNOTENOUGHDPADS (14)
#define VL53L1_DEVICEERROR_REFSPADCHARMORETHANTARGET (15)
#define VL53L1_DEVICEERROR_REFSPADCHARLESSTHANTARGET (16)
#define VL53L1_DEVICEERROR_MULTCLIPFAIL (17)
#define VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY (18)
#define VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK (19)
#define VL53L1_DEVICEERROR_EVENTCONSISTENCY (20)
#define VL53L1_DEVICEERROR_MINSIGNALEVENTCHECK (21)
#define VL53L1_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE (22)

#define VL53L1_RANGESTATUS_RANGE_VALID 0 /*!<The Range is valid. */
#define VL53L1_RANGESTATUS_SIGMA_FAIL 1  /*!<Sigma Fail. */
#define VL53L1_RANGESTATUS_SIGNAL_FAIL 2 /*!<Signal fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED \
  3 /*!<Target is below minimum detection threshold. */
#define VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL \
  4 /*!<Phase out of valid limits -  different to a wrap exit. */
#define VL53L1_RANGESTATUS_HARDWARE_FAIL 5 /*!<Hardware fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL \
  6 /*!<The Range is valid but the wraparound check has not been done. */
#define VL53L1_RANGESTATUS_WRAP_TARGET_FAIL \
  7 /*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define VL53L1_RANGESTATUS_PROCESSING_FAIL \
  8 /*!<Internal algo underflow or overflow in lite ranging. */
#define VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL \
  9 /*!<Specific to lite ranging.            \
     */
#define VL53L1_RANGESTATUS_SYNCRONISATION_INT                             \
  10 /*!<1st interrupt when starting ranging in back to back mode. Ignore \
        data. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE                   \
  11 /*!<All Range ok but object is result of multiple pulses merging \
        together.*/
#define VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL \
  12 /*!<Used  by RQL  as different to phase fail. */
#define VL53L1_RANGESTATUS_MIN_RANGE_FAIL \
  13 /*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define VL53L1_RANGESTATUS_RANGE_INVALID \
  14 /*!<lld returned valid range but negative value ! */
#define VL53L1_RANGESTATUS_NONE 255 /*!<No Update. */

  // Read status
  uint8_t buff[1] = {0};
  // cpi2c_readRegister(_i2cport, VL53L1_RESULT__RANGE_STATUS) & 0x1F;
  iscm->read_register(this->deviceAddress, VL53L1_RESULT__RANGE_STATUS, buff,
                      1);
  uint8_t measurementStatus = buff[0];

  // Convert status from one to another - From vl53l1_api.c
  switch (measurementStatus) {
    case VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY:
      measurementStatus = VL53L1_RANGESTATUS_SYNCRONISATION_INT;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGEPHASECHECK:
      measurementStatus = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
      break;
    case VL53L1_DEVICEERROR_MSRCNOTARGET:
      measurementStatus = VL53L1_RANGESTATUS_SIGNAL_FAIL;
      break;
    case VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK:
      measurementStatus = VL53L1_RANGESTATUS_SIGMA_FAIL;
      break;
    case VL53L1_DEVICEERROR_PHASECONSISTENCY:
      measurementStatus = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD:
      measurementStatus = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
      break;
    case VL53L1_DEVICEERROR_MINCLIP:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID;
      break;
    default:
      measurementStatus = VL53L1_RANGESTATUS_NONE;
  }

  return measurementStatus;
}

void VL53L1X::setAddress(uint8_t newAddress) {
  this->iscm->write_register16(deviceAddress, VL53L1_I2C_SLAVE__DEVICE_ADDRESS,
                               &newAddress, 1);
  deviceAddress = newAddress;
}