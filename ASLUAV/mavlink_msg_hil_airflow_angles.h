#pragma once
// MESSAGE HIL_AIRFLOW_ANGLES PACKING

#define MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES 215

MAVPACKED(
typedef struct __mavlink_hil_airflow_angles_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float angleofattack; /*< [deg] Angle of attack*/
 float sideslip; /*< [deg] Sideslip angle*/
}) mavlink_hil_airflow_angles_t;

#define MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN 16
#define MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_MIN_LEN 16
#define MAVLINK_MSG_ID_215_LEN 16
#define MAVLINK_MSG_ID_215_MIN_LEN 16

#define MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_CRC 208
#define MAVLINK_MSG_ID_215_CRC 208



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_AIRFLOW_ANGLES { \
    215, \
    "HIL_AIRFLOW_ANGLES", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_airflow_angles_t, timestamp) }, \
         { "angleofattack", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_airflow_angles_t, angleofattack) }, \
         { "sideslip", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_airflow_angles_t, sideslip) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_AIRFLOW_ANGLES { \
    "HIL_AIRFLOW_ANGLES", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_airflow_angles_t, timestamp) }, \
         { "angleofattack", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_airflow_angles_t, angleofattack) }, \
         { "sideslip", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_airflow_angles_t, sideslip) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_airflow_angles message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param angleofattack [deg] Angle of attack
 * @param sideslip [deg] Sideslip angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_airflow_angles_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float angleofattack, float sideslip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN);
#else
    mavlink_hil_airflow_angles_t packet;
    packet.timestamp = timestamp;
    packet.angleofattack = angleofattack;
    packet.sideslip = sideslip;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_CRC);
}

/**
 * @brief Pack a hil_airflow_angles message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param angleofattack [deg] Angle of attack
 * @param sideslip [deg] Sideslip angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_airflow_angles_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float angleofattack,float sideslip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN);
#else
    mavlink_hil_airflow_angles_t packet;
    packet.timestamp = timestamp;
    packet.angleofattack = angleofattack;
    packet.sideslip = sideslip;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_CRC);
}

/**
 * @brief Encode a hil_airflow_angles struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_airflow_angles C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_airflow_angles_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_airflow_angles_t* hil_airflow_angles)
{
    return mavlink_msg_hil_airflow_angles_pack(system_id, component_id, msg, hil_airflow_angles->timestamp, hil_airflow_angles->angleofattack, hil_airflow_angles->sideslip);
}

/**
 * @brief Encode a hil_airflow_angles struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_airflow_angles C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_airflow_angles_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_airflow_angles_t* hil_airflow_angles)
{
    return mavlink_msg_hil_airflow_angles_pack_chan(system_id, component_id, chan, msg, hil_airflow_angles->timestamp, hil_airflow_angles->angleofattack, hil_airflow_angles->sideslip);
}

/**
 * @brief Send a hil_airflow_angles message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param angleofattack [deg] Angle of attack
 * @param sideslip [deg] Sideslip angle
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_airflow_angles_send(mavlink_channel_t chan, uint64_t timestamp, float angleofattack, float sideslip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES, buf, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_CRC);
#else
    mavlink_hil_airflow_angles_t packet;
    packet.timestamp = timestamp;
    packet.angleofattack = angleofattack;
    packet.sideslip = sideslip;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES, (const char *)&packet, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_CRC);
#endif
}

/**
 * @brief Send a hil_airflow_angles message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_airflow_angles_send_struct(mavlink_channel_t chan, const mavlink_hil_airflow_angles_t* hil_airflow_angles)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_airflow_angles_send(chan, hil_airflow_angles->timestamp, hil_airflow_angles->angleofattack, hil_airflow_angles->sideslip);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES, (const char *)hil_airflow_angles, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_airflow_angles_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float angleofattack, float sideslip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES, buf, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_CRC);
#else
    mavlink_hil_airflow_angles_t *packet = (mavlink_hil_airflow_angles_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->angleofattack = angleofattack;
    packet->sideslip = sideslip;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES, (const char *)packet, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_AIRFLOW_ANGLES UNPACKING


/**
 * @brief Get field timestamp from hil_airflow_angles message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_hil_airflow_angles_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field angleofattack from hil_airflow_angles message
 *
 * @return [deg] Angle of attack
 */
static inline float mavlink_msg_hil_airflow_angles_get_angleofattack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field sideslip from hil_airflow_angles message
 *
 * @return [deg] Sideslip angle
 */
static inline float mavlink_msg_hil_airflow_angles_get_sideslip(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a hil_airflow_angles message into a struct
 *
 * @param msg The message to decode
 * @param hil_airflow_angles C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_airflow_angles_decode(const mavlink_message_t* msg, mavlink_hil_airflow_angles_t* hil_airflow_angles)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_airflow_angles->timestamp = mavlink_msg_hil_airflow_angles_get_timestamp(msg);
    hil_airflow_angles->angleofattack = mavlink_msg_hil_airflow_angles_get_angleofattack(msg);
    hil_airflow_angles->sideslip = mavlink_msg_hil_airflow_angles_get_sideslip(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN? msg->len : MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN;
        memset(hil_airflow_angles, 0, MAVLINK_MSG_ID_HIL_AIRFLOW_ANGLES_LEN);
    memcpy(hil_airflow_angles, _MAV_PAYLOAD(msg), len);
#endif
}
