#pragma once
// MESSAGE SENS_HALL PACKING

#define MAVLINK_MSG_ID_SENS_HALL 213

MAVPACKED(
typedef struct __mavlink_sens_hall_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float mag_T; /*< [mT] Magnetic force [mT]*/
 float temp_C; /*< [C] Temperature [C]*/
 float aoa; /*< [deg] Angle of Attack [deg]*/
}) mavlink_sens_hall_t;

#define MAVLINK_MSG_ID_SENS_HALL_LEN 20
#define MAVLINK_MSG_ID_SENS_HALL_MIN_LEN 20
#define MAVLINK_MSG_ID_213_LEN 20
#define MAVLINK_MSG_ID_213_MIN_LEN 20

#define MAVLINK_MSG_ID_SENS_HALL_CRC 202
#define MAVLINK_MSG_ID_213_CRC 202



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENS_HALL { \
    213, \
    "SENS_HALL", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_hall_t, timestamp) }, \
         { "mag_T", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_hall_t, mag_T) }, \
         { "temp_C", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_hall_t, temp_C) }, \
         { "aoa", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sens_hall_t, aoa) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENS_HALL { \
    "SENS_HALL", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_hall_t, timestamp) }, \
         { "mag_T", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_hall_t, mag_T) }, \
         { "temp_C", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_hall_t, temp_C) }, \
         { "aoa", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sens_hall_t, aoa) }, \
         } \
}
#endif

/**
 * @brief Pack a sens_hall message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param mag_T [mT] Magnetic force [mT]
 * @param temp_C [C] Temperature [C]
 * @param aoa [deg] Angle of Attack [deg]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_hall_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float mag_T, float temp_C, float aoa)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_HALL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, mag_T);
    _mav_put_float(buf, 12, temp_C);
    _mav_put_float(buf, 16, aoa);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_HALL_LEN);
#else
    mavlink_sens_hall_t packet;
    packet.timestamp = timestamp;
    packet.mag_T = mag_T;
    packet.temp_C = temp_C;
    packet.aoa = aoa;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_HALL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_HALL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_HALL_MIN_LEN, MAVLINK_MSG_ID_SENS_HALL_LEN, MAVLINK_MSG_ID_SENS_HALL_CRC);
}

/**
 * @brief Pack a sens_hall message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param mag_T [mT] Magnetic force [mT]
 * @param temp_C [C] Temperature [C]
 * @param aoa [deg] Angle of Attack [deg]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_hall_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float mag_T,float temp_C,float aoa)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_HALL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, mag_T);
    _mav_put_float(buf, 12, temp_C);
    _mav_put_float(buf, 16, aoa);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_HALL_LEN);
#else
    mavlink_sens_hall_t packet;
    packet.timestamp = timestamp;
    packet.mag_T = mag_T;
    packet.temp_C = temp_C;
    packet.aoa = aoa;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_HALL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_HALL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_HALL_MIN_LEN, MAVLINK_MSG_ID_SENS_HALL_LEN, MAVLINK_MSG_ID_SENS_HALL_CRC);
}

/**
 * @brief Encode a sens_hall struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sens_hall C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_hall_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sens_hall_t* sens_hall)
{
    return mavlink_msg_sens_hall_pack(system_id, component_id, msg, sens_hall->timestamp, sens_hall->mag_T, sens_hall->temp_C, sens_hall->aoa);
}

/**
 * @brief Encode a sens_hall struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sens_hall C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_hall_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sens_hall_t* sens_hall)
{
    return mavlink_msg_sens_hall_pack_chan(system_id, component_id, chan, msg, sens_hall->timestamp, sens_hall->mag_T, sens_hall->temp_C, sens_hall->aoa);
}

/**
 * @brief Send a sens_hall message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param mag_T [mT] Magnetic force [mT]
 * @param temp_C [C] Temperature [C]
 * @param aoa [deg] Angle of Attack [deg]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sens_hall_send(mavlink_channel_t chan, uint64_t timestamp, float mag_T, float temp_C, float aoa)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_HALL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, mag_T);
    _mav_put_float(buf, 12, temp_C);
    _mav_put_float(buf, 16, aoa);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_HALL, buf, MAVLINK_MSG_ID_SENS_HALL_MIN_LEN, MAVLINK_MSG_ID_SENS_HALL_LEN, MAVLINK_MSG_ID_SENS_HALL_CRC);
#else
    mavlink_sens_hall_t packet;
    packet.timestamp = timestamp;
    packet.mag_T = mag_T;
    packet.temp_C = temp_C;
    packet.aoa = aoa;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_HALL, (const char *)&packet, MAVLINK_MSG_ID_SENS_HALL_MIN_LEN, MAVLINK_MSG_ID_SENS_HALL_LEN, MAVLINK_MSG_ID_SENS_HALL_CRC);
#endif
}

/**
 * @brief Send a sens_hall message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sens_hall_send_struct(mavlink_channel_t chan, const mavlink_sens_hall_t* sens_hall)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sens_hall_send(chan, sens_hall->timestamp, sens_hall->mag_T, sens_hall->temp_C, sens_hall->aoa);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_HALL, (const char *)sens_hall, MAVLINK_MSG_ID_SENS_HALL_MIN_LEN, MAVLINK_MSG_ID_SENS_HALL_LEN, MAVLINK_MSG_ID_SENS_HALL_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENS_HALL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sens_hall_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float mag_T, float temp_C, float aoa)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, mag_T);
    _mav_put_float(buf, 12, temp_C);
    _mav_put_float(buf, 16, aoa);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_HALL, buf, MAVLINK_MSG_ID_SENS_HALL_MIN_LEN, MAVLINK_MSG_ID_SENS_HALL_LEN, MAVLINK_MSG_ID_SENS_HALL_CRC);
#else
    mavlink_sens_hall_t *packet = (mavlink_sens_hall_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->mag_T = mag_T;
    packet->temp_C = temp_C;
    packet->aoa = aoa;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_HALL, (const char *)packet, MAVLINK_MSG_ID_SENS_HALL_MIN_LEN, MAVLINK_MSG_ID_SENS_HALL_LEN, MAVLINK_MSG_ID_SENS_HALL_CRC);
#endif
}
#endif

#endif

// MESSAGE SENS_HALL UNPACKING


/**
 * @brief Get field timestamp from sens_hall message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_sens_hall_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mag_T from sens_hall message
 *
 * @return [mT] Magnetic force [mT]
 */
static inline float mavlink_msg_sens_hall_get_mag_T(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temp_C from sens_hall message
 *
 * @return [C] Temperature [C]
 */
static inline float mavlink_msg_sens_hall_get_temp_C(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field aoa from sens_hall message
 *
 * @return [deg] Angle of Attack [deg]
 */
static inline float mavlink_msg_sens_hall_get_aoa(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a sens_hall message into a struct
 *
 * @param msg The message to decode
 * @param sens_hall C-struct to decode the message contents into
 */
static inline void mavlink_msg_sens_hall_decode(const mavlink_message_t* msg, mavlink_sens_hall_t* sens_hall)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sens_hall->timestamp = mavlink_msg_sens_hall_get_timestamp(msg);
    sens_hall->mag_T = mavlink_msg_sens_hall_get_mag_T(msg);
    sens_hall->temp_C = mavlink_msg_sens_hall_get_temp_C(msg);
    sens_hall->aoa = mavlink_msg_sens_hall_get_aoa(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENS_HALL_LEN? msg->len : MAVLINK_MSG_ID_SENS_HALL_LEN;
        memset(sens_hall, 0, MAVLINK_MSG_ID_SENS_HALL_LEN);
    memcpy(sens_hall, _MAV_PAYLOAD(msg), len);
#endif
}
