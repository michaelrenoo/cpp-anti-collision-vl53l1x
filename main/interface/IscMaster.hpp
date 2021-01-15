/**
 * @file I2CMaster.h
 * @author Juri Bieler (juribieler@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-05-17
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef MAIN_INTERFACE_ISCMASTER_H_
#define MAIN_INTERFACE_ISCMASTER_H_

#include <stdio.h>

#include <cstdint>
#include <cstring>
#include <string>

#include "driver/driver.hpp"
#include "utils/converterUtils.hpp"

// #define I2CMASTER_ACK_CHECK_ENABLE 0x01
#define I2CMASTER_READ_PAUSE_MS 500

#define I2CMASTER_USR_CMD_OUT_BUFF_LEN 2 * USR_CMD_MAX_TOTAL_LENGTH

class IscMaster {
 private:
  bool userComIsConneced;
  i2c_port_t i2cPort;
  uint8_t cmd_count;

 public:
  IscMaster();
  void init(i2c_port_t i2cPort);
  void set_user_com_address(uint16_t address);
  i2c_port_t get_i2c_port();
  void parse();
  int read_bytes(uint16_t address, uint8_t* buff, uint16_t len);

  int read_register(uint16_t address, uint8_t regAdd, uint8_t* readBuff,
                    uint16_t readBuffLen);
  int read_register16(uint16_t address, uint16_t regAdd, uint8_t* readBuff,
                      uint16_t readBuffLen);
  int write_register(uint16_t address, uint8_t regAdd, uint8_t* writeBuff,
                     uint16_t writeBuffLen);
  int write_register16(uint16_t address, uint16_t regAdd, uint8_t* writeBuff,
                       uint16_t writeBuffLen);
  int send_bytes(uint16_t address, uint8_t* buff, uint16_t len);
};

#endif /* MAIN_INTERFACE_ISCMASTER_H_ */