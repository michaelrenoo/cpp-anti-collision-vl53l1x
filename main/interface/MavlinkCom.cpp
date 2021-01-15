/*
 * MavlinkCom.cpp
 *
 *  Created on: 25 Mar 2019
 *      Author: Juri Bieler
 */

#include "MavlinkCom.hpp"

static const char* TAG = "MavlinkCom";

const event_bit_t MAV_BIT_MSG_IN = BIT1;
const event_bit_t MAV_BIT_ACK_REC = BIT2;
const event_bit_t MAV_BIT_PARAM_REC = BIT3;
const event_bit_t MAV_BIT_MISSION_ACK_REC = BIT4;
const event_bit_t MAV_BIT_MISSION_REQ_REC = BIT5;
const event_bit_t MAV_BIT_LOG_ENTRY_REC = BIT6;
const event_bit_t MAV_BIT_LOG_DATA_REC = BIT6;
static TaskEventVar mav_com_event_group;

mavlink_message_t in_msg_ary[MAV_IN_MSG_QUE_LENGTH];
Queue<mavlink_message_t> mavInMsqQue;

mavlink_message_t in_msg;
mavlink_status_t in_stat;

#include "FPS.hpp"
extern FPS fps;
extern MavlinkCom mav;

Semaphore mavSemaphore;

#if BUILD_TARGET == TARGET_ESP32
static void mavlink_in_process_task(void* pvParameters) {
#else
static void mavlink_in_process_task() {
#endif
  // event_bit_t uxBits;
  while (1) {
    if (mav_com_event_group.wait_for(MAV_BIT_MSG_IN,
                                     MAV_CONNECTION_LOST_TIMEOUT_MS)) {
      mav.process_cmds();
    }
    if (!mav.has_active_connection()) {
      // mav.is_configured = 0;
      mav.do_configuration();
    }
  }
  end_task();
}

#if BUILD_TARGET == TARGET_ESP32
static void mavlink_configure_task(void* pvParameters) {
#else
static void mavlink_configure_task() {
#endif
  // log_i(TAG, "RUN CONFIGURATION...");
  mav.setup_msg_intervalls();
  // log_i(TAG, "CONFIGURATION DONE!");
  mav.is_configured = 1;
  end_task();
}

#if BUILD_TARGET == TARGET_ESP32
static void heartbeat_task(void* pvParameters) {
#else
static void heartbeat_task() {
#endif
  DelayUntil delay = DelayUntil();
  while (1) {
    if (true) {
      mav.send_heartbeat();
      delay.wait_for(HEARTBEAT_PAUSE_MS);
    } else {
      task_delay_ms(IDLE_TASK_PAUSE_MS);
    }
  }
}

MavlinkCom::MavlinkCom()
    : is_configured(0),
      fcFrqCounter(0),
      hbCounter(0),
      current_fc_mis_seq(0),
      last_hb_received_ms(0),
      last_pos_received_ms(0),
      uartPort(UART_NUM_1),
      config_in_progress(false),
      last_ack(),
      last_mission_request() {
  this->last_mission_request.seq = 255;
}

void MavlinkCom::init() {
  mavInMsqQue.init(in_msg_ary, MAV_IN_MSG_QUE_LENGTH);
  task_delay_ms(100);
  start_task(mavlink_in_process_task, "mavlink_in_process", 4 * 1024, 12);
  start_task(heartbeat_task, "heartbeat_task", 2 * 1024, 1);
}

void MavlinkCom::send_heartbeat() {
  uint8_t heartbeatBuff[32];
  int heartbeatLen = mav_generate_HEARTBEAT(heartbeatBuff, FPS_OUT_MAX_LEN);
  fps.send_bytes(heartbeatBuff, heartbeatLen);
}

void MavlinkCom::do_configuration() {
  if (this->config_in_progress) {
    return;
  }
  mav.is_configured = 0;
  start_task(mavlink_configure_task, "mavlink_configure", 4 * 1024, 12);
}

void MavlinkCom::process_cmds() {
  while (!mavInMsqQue.is_empty()) {
    // mavlink_message_t in_msg = mavInMsqQue.top();
    this->process_new_in_msg(mavInMsqQue.top());
  }
}

void MavlinkCom::parse_in_msg(uint8_t* buff, uint16_t len) {
  uint8_t ret = 0;
  for (uint16_t i = 0; i < len; i++) {
    ret = mavlink_parse_char(0, buff[i], &in_msg, &in_stat);
    if (ret == 1) {
      mavInMsqQue.push(&in_msg);
      mav_com_event_group.set(MAV_BIT_MSG_IN);
    }
    if (ret == 2) {
      log_w(TAG, "bad CRC in incomming mavlink msg (@i%d)", i);
    }
  }
}

void MavlinkCom::process_new_in_msg(mavlink_message_t in_msg) {
  if (in_msg.sysid == 255) {
    // this comes from groundstation
    return;
  }
  if (in_msg.sysid != 1 || in_msg.compid != 1) {
    if (in_msg.sysid == 252 || in_msg.compid == 1) {
      // msg is from groundstation
    } else {
      log_i(TAG,
            "msg (%d) not from FC, wrong ID(s): sysID: %d(exp. %d), "
            "compID: %d(exp. %d)",
            in_msg.msgid, in_msg.sysid, 1, in_msg.compid, 1);
    }
    return;
  }
  switch (in_msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      this->last_hb_received_ms = get_time_system_ms();
      mavlink_heartbeat_t hb = {};
      mavlink_msg_heartbeat_decode((const mavlink_message_t*)&in_msg, &hb);
      this->hbCounter++;
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK:
      mavlink_msg_command_ack_decode((const mavlink_message_t*)&in_msg,
                                     &last_ack);
      this->set_receive_bit(MAV_BIT_ACK_REC);
      break;
    case MAVLINK_MSG_ID_HOME_POSITION:
      break;
    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
      break;
    case MAVLINK_MSG_ID_SYS_STATUS:
      break;
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
      break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      break;
    case MAVLINK_MSG_ID_GPS2_RAW:
      break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
      break;
    case MAVLINK_MSG_ID_MISSION_CURRENT:
      break;
    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
      break;
    case MAVLINK_MSG_ID_PARAM_VALUE:
      break;
    case MAVLINK_MSG_ID_MISSION_ACK:
      break;
    case MAVLINK_MSG_ID_LOG_ENTRY:
      break;
    case MAVLINK_MSG_ID_LOG_DATA:
      break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
      break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
      break;
    case MAVLINK_MSG_ID_MISSION_ITEM:
      break;
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
      break;
    case MAVLINK_MSG_ID_ATTITUDE:
      break;
    case MAVLINK_MSG_ID_POWER_STATUS:
      break;
    case MAVLINK_MSG_ID_RC_CHANNELS:
      break;
    case MAVLINK_MSG_ID_RAW_IMU:
      break;
    case MAVLINK_MSG_ID_SCALED_IMU2:
      break;
    case MAVLINK_MSG_ID_SCALED_IMU3:
      break;
    case MAVLINK_MSG_ID_SYSTEM_TIME:
      break;
    case MAVLINK_MSG_ID_TIMESYNC:
      break;
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
      break;
    case MAVLINK_MSG_ID_SIMSTATE:
      break;
    case MAVLINK_MSG_ID_HWSTATUS:
      break;
    case MAVLINK_MSG_ID_VIBRATION:
      break;
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
      break;
    case MAVLINK_MSG_ID_VFR_HUD:
      break;
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
      break;
    case MAVLINK_MSG_ID_AHRS:
      break;
    case MAVLINK_MSG_ID_AHRS2:
      break;
    case MAVLINK_MSG_ID_AHRS3:
      break;
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
      break;
    case MAVLINK_MSG_ID_MEMINFO:
      break;
    case MAVLINK_MSG_ID_TERRAIN_REPORT:
      // read this for alt above ground
      break;
    case MAVLINK_MSG_ID_RANGEFINDER:
      break;
    case MAVLINK_MSG_ID_SENSOR_OFFSETS:
      break;
    default:
      log_i(TAG, "unhandeled message: %d", in_msg.msgid);
      break;
  }
}

void MavlinkCom::set_receive_bit(const event_bit_t bit) {
  mav_com_event_group.set(bit);
}

bool MavlinkCom::wait_for_bit(uint16_t timeout_ms, const event_bit_t bits) {
  return mav_com_event_group.wait_for(bits, timeout_ms);
}

uint8_t MavlinkCom::get_connection_status() {
  if (this->config_in_progress) return MavConnectionConfigRunning;
  if (!has_active_connection()) return MavConnectionDead;
  return MavConnectionOk;
}

bool MavlinkCom::has_active_connection() {
  return get_time_system_ms() - this->last_hb_received_ms <=
             MAV_HEARTBEAT_TIMEOUT_MS and
         get_time_system_ms() - this->last_pos_received_ms <=
             MAV_POSiTION_TIMEOUT_MS;
}

bool MavlinkCom::wait_for_mission_item_request(uint8_t seq) {
  uint32_t start = get_time_system_ms();
  // xEventGroupClearBits(mav_com_event_group, MAV_BIT_MISSION_REQ_REC);
  mav_com_event_group.clear(MAV_BIT_MISSION_REQ_REC);
  while (get_time_system_ms() - start <= MAV_WAIT_FOR_ACK_TIMEOUT_MS) {
    if (this->wait_for_bit(MAV_WAIT_FOR_ACK_TIMEOUT_MS,
                           MAV_BIT_MISSION_REQ_REC)) {
      if (this->last_mission_request.seq == seq) {
        // log_i(TAG, "got mission item request after %d msec",
        //          get_time_system_ms() - start);
        return true;
      }
    }
  }
  log_i(TAG, "failed to wait for mission item request nr. %d after %d msec",
        seq, get_time_system_ms() - start);
  return false;
}

bool MavlinkCom::wait_for_mission_ack() {
  // xEventGroupClearBits(mav_com_event_group, MAV_BIT_MISSION_ACK_REC);
  mav_com_event_group.clear(MAV_BIT_MISSION_ACK_REC);
  if (this->wait_for_bit(MAV_WAIT_FOR_ACK_TIMEOUT_MS,
                         MAV_BIT_MISSION_ACK_REC)) {
    return true;
  }
  return false;
}

bool MavlinkCom::send_msg(uint8_t* mavBuff, uint16_t msgLen) {
  return fps.send_bytes(mavBuff, msgLen) > 0;
}

bool MavlinkCom::send_msg_secure(uint8_t* mavBuff, uint16_t msgLen,
                                 uint32_t msg_id) {
  if (!this->has_active_connection() && mav.is_configured > 0 &&
      !this->config_in_progress) {
    log_w(TAG, "Warning: could not send secure, no active connection");
    // return false;
    // we should try to send anyway
  }
  if (mavSemaphore.take(MAV_COM_AQUIRE_SEMAPHORE_TIMEOUT)) {
    // clear event bit first, in case there is a old, unhandled ack
    mav_com_event_group.clear(MAV_BIT_ACK_REC);
    mav_com_event_group.clear(MAV_BIT_PARAM_REC);
    mav_com_event_group.clear(MAV_BIT_MISSION_ACK_REC);
    mav_com_event_group.clear(MAV_BIT_MISSION_REQ_REC);
    mav_com_event_group.clear(MAV_BIT_LOG_ENTRY_REC);
    mav_com_event_group.clear(MAV_BIT_LOG_DATA_REC);
    uint32_t start = get_time_system_ms();
    uint8_t i = 0;
    for (i = 0; i < MAV_RESEND_TRY_COUNT; i++) {
      // log_i(TAG, "send msg: %d, retries: %d", msg_id, i);
      fps.send_bytes(mavBuff, msgLen);
      switch (msg_id) {
        case MAV_CMD_DO_SET_PARAMETER: {
          // TODO: incosistend use of MAV_CMD_x and MAVLINK_MSG_ID_x
          if (this->wait_for_bit(MAV_WAIT_FOR_ACK_TIMEOUT_MS,
                                 MAV_BIT_PARAM_REC)) {
            mavSemaphore.give();
            return true;
          }
          break;
        }
        case MAVLINK_MSG_ID_PARAM_SET: {
          if (this->wait_for_bit(MAV_WAIT_FOR_ACK_TIMEOUT_MS,
                                 MAV_BIT_PARAM_REC)) {
            mavSemaphore.give();
            return true;
          }
          break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
          if (this->wait_for_bit(MAV_WAIT_FOR_ACK_TIMEOUT_MS,
                                 MAV_BIT_PARAM_REC)) {
            mavSemaphore.give();
            return true;
          }
          break;
        }
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
          if (this->wait_for_mission_ack()) {
            mavSemaphore.give();
            return true;
          }
          break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
          // only use this for goto mission item, otherwise the response is
          // request next sequence
          if (this->wait_for_mission_ack()) {
            mavSemaphore.give();
            return true;
          }
          break;
        }
        case MAVLINK_MSG_ID_LOG_ENTRY: {
          if (this->wait_for_bit(MAV_WAIT_FOR_ACK_TIMEOUT_MS,
                                 MAV_BIT_LOG_ENTRY_REC)) {
            mavSemaphore.give();
            return true;
          }
          break;
        }
        case MAVLINK_MSG_ID_LOG_DATA: {
          if (this->wait_for_bit(MAV_WAIT_FOR_ACK_TIMEOUT_MS,
                                 MAV_BIT_LOG_DATA_REC)) {
            mavSemaphore.give();
            return true;
          }
          break;
        }
        default: {
          if (this->wait_for_bit(MAV_WAIT_FOR_ACK_TIMEOUT_MS,
                                 MAV_BIT_ACK_REC)) {
            if (this->last_ack.command == msg_id) {
              if (this->last_ack.result == MAV_RESULT_ACCEPTED) {
                // log_i(
                //     TAG,
                //     "success send secure mav message (%d) after: %d ms and
                //     "
                //     "%d tries",
                //     msg_id, get_time_system_ms() - start, i + 1);
                mavSemaphore.give();
                return true;
              } else {
                log_e(TAG,
                      "mav message (%d) ack returned an error_code: %d, for "
                      "msg: %d, prgress: "
                      "%d, res2: %d",
                      msg_id, this->last_ack.result, this->last_ack.command,
                      this->last_ack.progress, this->last_ack.result_param2);
                task_delay_ms(500);
              }
            } else {
              log_w(TAG, "cought ack for different msg_id: %d, but wanted: %d",
                    this->last_ack.command, msg_id);
            }
          }
          break;
        }
      }
    }
    log_e(TAG, "failed to send mav message (%d) timeout: %d ms and %d tries",
          msg_id, get_time_system_ms() - start, i + 1);
    mavSemaphore.give();
  } else {
    log_w(TAG, "Warning: could not obtain semaphore (mav message (%d))",
          msg_id);
  }
  return false;
}

/*
 * PRESETS
 */

bool MavlinkCom::set_msg_interval(uint16_t msg_id, int32_t intervall_us) {
  uint8_t mavBuff[MAV_OUT_MAX_LEN];
  int msgLen = 0;
  msgLen = mav_generate_set_message_interval(mavBuff, MAV_OUT_MAX_LEN, msg_id,
                                             intervall_us);
  return this->send_msg_secure(mavBuff, msgLen, MAV_CMD_SET_MESSAGE_INTERVAL);
}

void MavlinkCom::setup_msg_intervalls() {
  // check so this process get only started once
  if (this->config_in_progress) {
    return;
  }
  this->config_in_progress = true;

  if (!this->set_msg_interval(MAVLINK_MSG_ID_SYS_STATUS, 1000000)) {
    log_w(TAG,
          "could not config mavlink, no response, wait for 10 sec. and "
          "retry...");
    task_delay_ms(10000);
    this->config_in_progress = false;
    return;
  }
  this->set_msg_interval(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1000000);  // 1Hz
  this->set_msg_interval(MAVLINK_MSG_ID_BATTERY_STATUS, 1000000);      // 1Hz
  this->set_msg_interval(MAVLINK_MSG_ID_GPS_RAW_INT, 200000);          // 5Hz
  this->set_msg_interval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000);  // 5Hz
  this->set_msg_interval(MAVLINK_MSG_ID_RANGEFINDER, 200000);          // 5Hz
  this->set_msg_interval(MAVLINK_MSG_ID_MISSION_CURRENT, 200000);      // 5Hz
  this->set_msg_interval(MAVLINK_MSG_ID_SCALED_PRESSURE, 200000);      // 5Hz
  this->set_msg_interval(MAVLINK_MSG_ID_SYSTEM_TIME, 5000000);         // 2Hz

  this->config_in_progress = false;
}
