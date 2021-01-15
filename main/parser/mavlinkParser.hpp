/*
 * UBXParser.h
 *
 *  Created on: Mar 16, 2018
 *      Author: juri
 */

#ifndef MAIN_PARSER_MAVLINKPARSER_H_
#define MAIN_PARSER_MAVLINKPARSER_H_

#include <stdio.h>

#include <string>

#include "utils/converterUtils.hpp"
//
#include "driver/driver.hpp"

//#include "mavlinkChecksum.hpp"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "parser/mavlink/v2/ardupilotmega/mavlink.h"

#define MAV_V1_HEADER_LEN 6
#define MAV_V2_HEADER_LEN 10
#define MAV_V1_SYNC_CHAR 0xfe
#define MAV_V2_SYNC_CHAR 0xfd
#define MAV_CHECKSUM_LEN 2

#define SYSTEM_ID 0x01
#define COMP_ID 82

#define TARGET_SYSTEM_ID 0x01
#define TARGET_COMP_ID 0x01

int mav_generate_HEARTBEAT(uint8_t* str, uint16_t maxLen);

int mav_generate_set_message_interval(uint8_t* str, uint16_t maxLen,
                                      uint16_t msg_id, int32_t intervall_us);

int mav_generate_distance_sensor(uint8_t* str, uint16_t max_len, uint8_t id,
                                 uint8_t orientation, uint16_t dist_cm);

#endif /* MAIN_PARSER_MAVLINKPARSER_H_ */
