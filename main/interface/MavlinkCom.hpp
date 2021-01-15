/*
 * MavlinkCom.h
 *
 *  Created on: 25 Mar 2019
 *      Author: Juri Bieler
 */

#ifndef MAIN_INTERFACE_MAVLINKCOM_H_
#define MAIN_INTERFACE_MAVLINKCOM_H_

#include "cppDatatypes/Queue.hpp"
#include "driver/driver.hpp"
#include "parser/mavlink/v2/ardupilotmega/mavlink.h"
#include "parser/mavlinkParser.hpp"

#define FPS_OUT_MAX_LEN 256

#define MAV_IN_MSG_QUE_LENGTH 8

#define HEARTBEAT_PAUSE_MS 1000
#define IDLE_TASK_PAUSE_MS 1000

#define MAV_COM_AQUIRE_SEMAPHORE_TIMEOUT 1000
#define MAV_CONNECTION_LOST_TIMEOUT_MS 1200
#define MAV_HEARTBEAT_TIMEOUT_MS 2050
#define MAV_POSiTION_TIMEOUT_MS 5000
#define MAV_WAIT_FOR_ACK_TIMEOUT_MS 1000
#define MAV_RESEND_TRY_COUNT 3
#define MAV_OUT_MAX_LEN 256

const int MavConnectionOk = 1;
const int MavConnectionConfigRunning = 2;
const int MavConnectionDead = 3;

class MavlinkCom {
 public:
  uint8_t is_configured;
  uint8_t fcFrqCounter;
  uint8_t hbCounter;

  MavlinkCom();
  void init();
  void send_heartbeat();
  void do_configuration();
  void process_cmds();
  void parse_in_msg(uint8_t *buff, uint16_t len);
  uint8_t get_connection_status();
  bool has_active_connection();
  bool wait_for_mission_item_request(uint8_t seq);
  bool wait_for_mission_ack();
  bool send_msg(uint8_t *mavBuff, uint16_t msgLen);
  bool send_msg_secure(uint8_t *mavBuff, uint16_t msgLen, uint32_t msg_id);
  bool set_msg_interval(uint16_t msg_id, int32_t intervall_us);
  void setup_msg_intervalls();
  uint16_t get_current_fc_mis_seq();

  void request_log_file_list(uint16_t form_index, uint16_t to_index);
  void request_log_data(uint16_t file_id, uint32_t offst, uint32_t count);
  void request_log_file_delete();

 private:
  uint16_t current_fc_mis_seq;
  uint32_t last_hb_received_ms;
  uint32_t last_pos_received_ms;
  uart_port_t uartPort;
  uint8_t config_in_progress;
  mavlink_command_ack_t last_ack;
  mavlink_mission_request_t last_mission_request;

  void set_receive_bit(const event_bit_t bit);
  void process_new_in_msg(mavlink_message_t in_msg);
  bool wait_for_bit(uint16_t timeout_ms, event_bit_t bits);
};

#endif /* MAIN_INTERFACE_MAVLINKCOM_H_ */
