/*
 * UBXParser.cpp
 *
 *  Created on: Mar 16, 2018
 *      Author: juri
 */

#include "mavlinkParser.hpp"

/*
 ******************************************************
 * basic commands                                     *
 ******************************************************
 */

/**
 * @param: str buffer to write on
 * @param: maxLen of buffer
 * @return: len of HEARTBEAT message in buffer
 */
int mav_generate_HEARTBEAT(uint8_t* str, uint16_t maxLen) {
  mavlink_message_t msg = {};
  uint16_t len = mavlink_msg_heartbeat_pack(
      SYSTEM_ID,  // system id
      COMP_ID,    // component id
      &msg,
      MAV_TYPE_ONBOARD_CONTROLLER,  // type - Type of the system (quadrotor,
                                    // helicopter, etc.). Components use the
                                    // same type as their associated system.
      MAV_AUTOPILOT_INVALID,        // autopilot - Autopilot type / class.
      0,                            // base_mode - System mode bitmap
      0,   // custom_mode - A bitfield for use for autopilot-specific flags
      0);  // system_status - System status flag.
  len = mavlink_msg_to_send_buffer(str, &msg);
  return len;
}

int mav_generate_set_message_interval(uint8_t* str, uint16_t max_len,
                                      uint16_t msg_id, int32_t intervall_us) {
  mavlink_message_t msg = {};
  uint16_t len = mavlink_msg_command_long_pack(
      SYSTEM_ID,         // system_id ID of this system
      COMP_ID,           // component_id ID of this component (e.g.
                         // 200 for IMU)
      &msg,              // msg The MAVLink message to compress the data into
      TARGET_SYSTEM_ID,  // target_system  System which should
                         // execute the command
      TARGET_COMP_ID,    // target_component  Component which
                         // should execute the command, 0 for
                         // all components
      MAV_CMD_SET_MESSAGE_INTERVAL,  // command  Command ID (of command to
                                     // send).
      0,       // confirmation  0: First transmission of this command. 1-255:
               // Confirmation transmissions (e.g. for kill command)
      msg_id,  // param1 - msg id
      intervall_us,    // param2 - intevall in us
      0, 0, 0, 0, 0);  // no use for param 2 to 7
  len = mavlink_msg_to_send_buffer(str, &msg);
  return len;
}

int mav_generate_distance_sensor(uint8_t* str, uint16_t max_len, uint8_t id,
                                 uint8_t orientation, uint16_t dist_cm) {
  const float q[4] = {1., 0., 0., 0.};
  mavlink_message_t msg = {};
  uint16_t len = mavlink_msg_distance_sensor_pack(
      SYSTEM_ID, COMP_ID, &msg, get_time_system_ms(),
      0,    // min dist
      400,  // max dist
      dist_cm, MAV_DISTANCE_SENSOR_LASER, id, orientation, 255,
      0.471239,  // Horizontal FOV in rad. 27 deg = 0.471239 rad
      0, q);
  len = mavlink_msg_to_send_buffer(str, &msg);
  return len;
}