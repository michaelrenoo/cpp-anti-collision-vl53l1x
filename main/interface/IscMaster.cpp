/**
 * @file I2CMaster.cpp
 * @author Juri Bieler (juribieler@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-05-17
 * Modified by Michael Reno (michaelreno19@gmail.com)
 * in January 2021
 * @copyright Copyright (c) 2021
 *
 */

#include "IscMaster.hpp"

static const char* TAG = "IscMaster";

extern IscMaster iscm;
extern TaskEventVar user_cmd_event_group;

Semaphore i2cmSemaphore;

#if BUILD_TARGET == TARGET_ESP32
static void i2c_master_read_task(void* pvParameters) {
#else
static void i2c_master_read_task() {
#endif
  // task_delay_ms(2000);
  isc_scan(iscm.get_i2c_port());
  while (1) {
    iscm.parse();
    task_delay_ms(I2CMASTER_READ_PAUSE_MS);
  }
  end_task();
}

IscMaster::IscMaster()
    : userComIsConneced(false), i2cPort(I2C_NUM_0), cmd_count(0) {}

void IscMaster::init(i2c_port_t i2cPort) {
  this->i2cPort = i2cPort;
  // i2cmSemaphore = Semaphore();
  start_task(i2c_master_read_task, "i2c read", 4 * 1024, 1);
}

i2c_port_t IscMaster::get_i2c_port() { return this->i2cPort; }

void IscMaster::parse() {}

int IscMaster::read_register(uint16_t address, uint8_t regAdd,
                             uint8_t* readBuff, uint16_t readBuffLen) {
  if (i2cmSemaphore.take(200)) {
    int ret =
        // this->read_register_locked(address, regAdd, readBuff, readBuffLen);
        isc_master_read_register(this->get_i2c_port(), address, regAdd,
                                 readBuff, readBuffLen);
    i2cmSemaphore.give();
    return ret;
  } else
    log_w(TAG, "read_register: could not obtain semaphore");
  return ERROR_FAIL;
}

int IscMaster::read_register16(uint16_t address, uint16_t regAdd,
                               uint8_t* readBuff, uint16_t readBuffLen) {
  if (i2cmSemaphore.take(200)) {
    int ret =
        // this->read_register_locked(address, regAdd, readBuff, readBuffLen);
        isc_master_read_register16(this->get_i2c_port(), address, regAdd,
                                   readBuff, readBuffLen);
    i2cmSemaphore.give();
    return ret;
  } else
    log_w(TAG, "read_register: could not obtain semaphore");
  return ERROR_FAIL;
}

int IscMaster::write_register(uint16_t address, uint8_t regAdd,
                              uint8_t* writeBuff, uint16_t writeBuffLen) {
  if (i2cmSemaphore.take(200)) {
    // int ret =
    //     this->write_register_locked(address, regAdd, writeBuff,
    //     writeBuffLen);
    int ret = isc_master_write_register(this->get_i2c_port(), address, regAdd,
                                        writeBuff, writeBuffLen);
    i2cmSemaphore.give();
    return ret;
  } else
    log_w(TAG, "write_register: could not obtain semaphore");
  return ERROR_FAIL;
}

int IscMaster::write_register16(uint16_t address, uint16_t regAdd,
                                uint8_t* writeBuff, uint16_t writeBuffLen) {
  if (i2cmSemaphore.take(200)) {
    int ret = isc_master_write_register16(this->get_i2c_port(), address, regAdd,
                                          writeBuff, writeBuffLen);
    i2cmSemaphore.give();
    return ret;
  } else
    log_w(TAG, "write_register: could not obtain semaphore");
  return ERROR_FAIL;
}

int IscMaster::send_bytes(uint16_t address, uint8_t* buff, uint16_t len) {
  if (i2cmSemaphore.take(200)) {
    // int ret = this->send_bytes_locked(address, buff, len);
    int ret = isc_master_write(this->get_i2c_port(), address, buff, len);
    // int ret = i2c_master_write_slave(this->get_i2c_port(),
    //                                  this->userComAddress, buff, len);
    i2cmSemaphore.give();
    return ret;
  } else
    log_w(TAG, "send_bytes: could not obtain semaphore");
  return ERROR_FAIL;
}
