#pragma once
// MESSAGE HIL_GROUND_TRUTH PACKING

#define MAVLINK_MSG_ID_HIL_GROUND_TRUTH 214

MAVPACKED(
typedef struct __mavlink_hil_ground_truth_t {
 uint64_t timestamp; /*< Timestamp*/
 float roll; /*< Roll angle*/
 float pitch; /*< Pitch angle*/
 float yaw; /*< Yaw angle*/
 float roll_rate; /*< Roll rate*/
 float pitch_rate; /*< Pitch rate*/
 float yaw_rate; /*< Yaw rate*/
 int32_t lat; /*< Latitude*/
 int32_t lon; /*< Longitude*/
 float alt; /*< Altitude*/
 float vn; /*< Northing ground speed*/
 float ve; /*< Easting ground speed*/
 float vd; /*< Down ground speed*/
 float ias; /*< Indicated airspeed*/
 float tas; /*< True airspeed*/
 float wind_speed; /*< Wind speed (2D)*/
 float wind_dir; /*< Wind direction (2D)*/
}) mavlink_hil_ground_truth_t;

#define MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN 72
#define MAVLINK_MSG_ID_HIL_GROUND_TRUTH_MIN_LEN 72
#define MAVLINK_MSG_ID_214_LEN 72
#define MAVLINK_MSG_ID_214_MIN_LEN 72

#define MAVLINK_MSG_ID_HIL_GROUND_TRUTH_CRC 56
#define MAVLINK_MSG_ID_214_CRC 56



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_GROUND_TRUTH { \
    214, \
    "HIL_GROUND_TRUTH", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_ground_truth_t, timestamp) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_ground_truth_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_ground_truth_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_ground_truth_t, yaw) }, \
         { "roll_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_ground_truth_t, roll_rate) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_ground_truth_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_ground_truth_t, yaw_rate) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_hil_ground_truth_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_hil_ground_truth_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_hil_ground_truth_t, alt) }, \
         { "vn", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_hil_ground_truth_t, vn) }, \
         { "ve", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_hil_ground_truth_t, ve) }, \
         { "vd", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_hil_ground_truth_t, vd) }, \
         { "ias", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_hil_ground_truth_t, ias) }, \
         { "tas", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_hil_ground_truth_t, tas) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_hil_ground_truth_t, wind_speed) }, \
         { "wind_dir", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_hil_ground_truth_t, wind_dir) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_GROUND_TRUTH { \
    "HIL_GROUND_TRUTH", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_ground_truth_t, timestamp) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_ground_truth_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_ground_truth_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_ground_truth_t, yaw) }, \
         { "roll_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_ground_truth_t, roll_rate) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_ground_truth_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_ground_truth_t, yaw_rate) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_hil_ground_truth_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_hil_ground_truth_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_hil_ground_truth_t, alt) }, \
         { "vn", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_hil_ground_truth_t, vn) }, \
         { "ve", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_hil_ground_truth_t, ve) }, \
         { "vd", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_hil_ground_truth_t, vd) }, \
         { "ias", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_hil_ground_truth_t, ias) }, \
         { "tas", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_hil_ground_truth_t, tas) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_hil_ground_truth_t, wind_speed) }, \
         { "wind_dir", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_hil_ground_truth_t, wind_dir) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_ground_truth message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp
 * @param roll Roll angle
 * @param pitch Pitch angle
 * @param yaw Yaw angle
 * @param roll_rate Roll rate
 * @param pitch_rate Pitch rate
 * @param yaw_rate Yaw rate
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude
 * @param vn Northing ground speed
 * @param ve Easting ground speed
 * @param vd Down ground speed
 * @param ias Indicated airspeed
 * @param tas True airspeed
 * @param wind_speed Wind speed (2D)
 * @param wind_dir Wind direction (2D)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_ground_truth_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float roll, float pitch, float yaw, float roll_rate, float pitch_rate, float yaw_rate, int32_t lat, int32_t lon, float alt, float vn, float ve, float vd, float ias, float tas, float wind_speed, float wind_dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, roll_rate);
    _mav_put_float(buf, 24, pitch_rate);
    _mav_put_float(buf, 28, yaw_rate);
    _mav_put_int32_t(buf, 32, lat);
    _mav_put_int32_t(buf, 36, lon);
    _mav_put_float(buf, 40, alt);
    _mav_put_float(buf, 44, vn);
    _mav_put_float(buf, 48, ve);
    _mav_put_float(buf, 52, vd);
    _mav_put_float(buf, 56, ias);
    _mav_put_float(buf, 60, tas);
    _mav_put_float(buf, 64, wind_speed);
    _mav_put_float(buf, 68, wind_dir);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN);
#else
    mavlink_hil_ground_truth_t packet;
    packet.timestamp = timestamp;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.roll_rate = roll_rate;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.ias = ias;
    packet.tas = tas;
    packet.wind_speed = wind_speed;
    packet.wind_dir = wind_dir;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_GROUND_TRUTH;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_MIN_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_CRC);
}

/**
 * @brief Pack a hil_ground_truth message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param roll Roll angle
 * @param pitch Pitch angle
 * @param yaw Yaw angle
 * @param roll_rate Roll rate
 * @param pitch_rate Pitch rate
 * @param yaw_rate Yaw rate
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude
 * @param vn Northing ground speed
 * @param ve Easting ground speed
 * @param vd Down ground speed
 * @param ias Indicated airspeed
 * @param tas True airspeed
 * @param wind_speed Wind speed (2D)
 * @param wind_dir Wind direction (2D)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_ground_truth_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float roll,float pitch,float yaw,float roll_rate,float pitch_rate,float yaw_rate,int32_t lat,int32_t lon,float alt,float vn,float ve,float vd,float ias,float tas,float wind_speed,float wind_dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, roll_rate);
    _mav_put_float(buf, 24, pitch_rate);
    _mav_put_float(buf, 28, yaw_rate);
    _mav_put_int32_t(buf, 32, lat);
    _mav_put_int32_t(buf, 36, lon);
    _mav_put_float(buf, 40, alt);
    _mav_put_float(buf, 44, vn);
    _mav_put_float(buf, 48, ve);
    _mav_put_float(buf, 52, vd);
    _mav_put_float(buf, 56, ias);
    _mav_put_float(buf, 60, tas);
    _mav_put_float(buf, 64, wind_speed);
    _mav_put_float(buf, 68, wind_dir);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN);
#else
    mavlink_hil_ground_truth_t packet;
    packet.timestamp = timestamp;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.roll_rate = roll_rate;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.ias = ias;
    packet.tas = tas;
    packet.wind_speed = wind_speed;
    packet.wind_dir = wind_dir;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_GROUND_TRUTH;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_MIN_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_CRC);
}

/**
 * @brief Encode a hil_ground_truth struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_ground_truth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_ground_truth_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_ground_truth_t* hil_ground_truth)
{
    return mavlink_msg_hil_ground_truth_pack(system_id, component_id, msg, hil_ground_truth->timestamp, hil_ground_truth->roll, hil_ground_truth->pitch, hil_ground_truth->yaw, hil_ground_truth->roll_rate, hil_ground_truth->pitch_rate, hil_ground_truth->yaw_rate, hil_ground_truth->lat, hil_ground_truth->lon, hil_ground_truth->alt, hil_ground_truth->vn, hil_ground_truth->ve, hil_ground_truth->vd, hil_ground_truth->ias, hil_ground_truth->tas, hil_ground_truth->wind_speed, hil_ground_truth->wind_dir);
}

/**
 * @brief Encode a hil_ground_truth struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_ground_truth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_ground_truth_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_ground_truth_t* hil_ground_truth)
{
    return mavlink_msg_hil_ground_truth_pack_chan(system_id, component_id, chan, msg, hil_ground_truth->timestamp, hil_ground_truth->roll, hil_ground_truth->pitch, hil_ground_truth->yaw, hil_ground_truth->roll_rate, hil_ground_truth->pitch_rate, hil_ground_truth->yaw_rate, hil_ground_truth->lat, hil_ground_truth->lon, hil_ground_truth->alt, hil_ground_truth->vn, hil_ground_truth->ve, hil_ground_truth->vd, hil_ground_truth->ias, hil_ground_truth->tas, hil_ground_truth->wind_speed, hil_ground_truth->wind_dir);
}

/**
 * @brief Send a hil_ground_truth message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param roll Roll angle
 * @param pitch Pitch angle
 * @param yaw Yaw angle
 * @param roll_rate Roll rate
 * @param pitch_rate Pitch rate
 * @param yaw_rate Yaw rate
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude
 * @param vn Northing ground speed
 * @param ve Easting ground speed
 * @param vd Down ground speed
 * @param ias Indicated airspeed
 * @param tas True airspeed
 * @param wind_speed Wind speed (2D)
 * @param wind_dir Wind direction (2D)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_ground_truth_send(mavlink_channel_t chan, uint64_t timestamp, float roll, float pitch, float yaw, float roll_rate, float pitch_rate, float yaw_rate, int32_t lat, int32_t lon, float alt, float vn, float ve, float vd, float ias, float tas, float wind_speed, float wind_dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, roll_rate);
    _mav_put_float(buf, 24, pitch_rate);
    _mav_put_float(buf, 28, yaw_rate);
    _mav_put_int32_t(buf, 32, lat);
    _mav_put_int32_t(buf, 36, lon);
    _mav_put_float(buf, 40, alt);
    _mav_put_float(buf, 44, vn);
    _mav_put_float(buf, 48, ve);
    _mav_put_float(buf, 52, vd);
    _mav_put_float(buf, 56, ias);
    _mav_put_float(buf, 60, tas);
    _mav_put_float(buf, 64, wind_speed);
    _mav_put_float(buf, 68, wind_dir);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GROUND_TRUTH, buf, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_MIN_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_CRC);
#else
    mavlink_hil_ground_truth_t packet;
    packet.timestamp = timestamp;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.roll_rate = roll_rate;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.ias = ias;
    packet.tas = tas;
    packet.wind_speed = wind_speed;
    packet.wind_dir = wind_dir;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GROUND_TRUTH, (const char *)&packet, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_MIN_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_CRC);
#endif
}

/**
 * @brief Send a hil_ground_truth message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_ground_truth_send_struct(mavlink_channel_t chan, const mavlink_hil_ground_truth_t* hil_ground_truth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_ground_truth_send(chan, hil_ground_truth->timestamp, hil_ground_truth->roll, hil_ground_truth->pitch, hil_ground_truth->yaw, hil_ground_truth->roll_rate, hil_ground_truth->pitch_rate, hil_ground_truth->yaw_rate, hil_ground_truth->lat, hil_ground_truth->lon, hil_ground_truth->alt, hil_ground_truth->vn, hil_ground_truth->ve, hil_ground_truth->vd, hil_ground_truth->ias, hil_ground_truth->tas, hil_ground_truth->wind_speed, hil_ground_truth->wind_dir);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GROUND_TRUTH, (const char *)hil_ground_truth, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_MIN_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_ground_truth_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float roll, float pitch, float yaw, float roll_rate, float pitch_rate, float yaw_rate, int32_t lat, int32_t lon, float alt, float vn, float ve, float vd, float ias, float tas, float wind_speed, float wind_dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, roll_rate);
    _mav_put_float(buf, 24, pitch_rate);
    _mav_put_float(buf, 28, yaw_rate);
    _mav_put_int32_t(buf, 32, lat);
    _mav_put_int32_t(buf, 36, lon);
    _mav_put_float(buf, 40, alt);
    _mav_put_float(buf, 44, vn);
    _mav_put_float(buf, 48, ve);
    _mav_put_float(buf, 52, vd);
    _mav_put_float(buf, 56, ias);
    _mav_put_float(buf, 60, tas);
    _mav_put_float(buf, 64, wind_speed);
    _mav_put_float(buf, 68, wind_dir);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GROUND_TRUTH, buf, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_MIN_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_CRC);
#else
    mavlink_hil_ground_truth_t *packet = (mavlink_hil_ground_truth_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->roll_rate = roll_rate;
    packet->pitch_rate = pitch_rate;
    packet->yaw_rate = yaw_rate;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->vn = vn;
    packet->ve = ve;
    packet->vd = vd;
    packet->ias = ias;
    packet->tas = tas;
    packet->wind_speed = wind_speed;
    packet->wind_dir = wind_dir;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GROUND_TRUTH, (const char *)packet, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_MIN_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_GROUND_TRUTH UNPACKING


/**
 * @brief Get field timestamp from hil_ground_truth message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_hil_ground_truth_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field roll from hil_ground_truth message
 *
 * @return Roll angle
 */
static inline float mavlink_msg_hil_ground_truth_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch from hil_ground_truth message
 *
 * @return Pitch angle
 */
static inline float mavlink_msg_hil_ground_truth_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw from hil_ground_truth message
 *
 * @return Yaw angle
 */
static inline float mavlink_msg_hil_ground_truth_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field roll_rate from hil_ground_truth message
 *
 * @return Roll rate
 */
static inline float mavlink_msg_hil_ground_truth_get_roll_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch_rate from hil_ground_truth message
 *
 * @return Pitch rate
 */
static inline float mavlink_msg_hil_ground_truth_get_pitch_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw_rate from hil_ground_truth message
 *
 * @return Yaw rate
 */
static inline float mavlink_msg_hil_ground_truth_get_yaw_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field lat from hil_ground_truth message
 *
 * @return Latitude
 */
static inline int32_t mavlink_msg_hil_ground_truth_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field lon from hil_ground_truth message
 *
 * @return Longitude
 */
static inline int32_t mavlink_msg_hil_ground_truth_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field alt from hil_ground_truth message
 *
 * @return Altitude
 */
static inline float mavlink_msg_hil_ground_truth_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field vn from hil_ground_truth message
 *
 * @return Northing ground speed
 */
static inline float mavlink_msg_hil_ground_truth_get_vn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field ve from hil_ground_truth message
 *
 * @return Easting ground speed
 */
static inline float mavlink_msg_hil_ground_truth_get_ve(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field vd from hil_ground_truth message
 *
 * @return Down ground speed
 */
static inline float mavlink_msg_hil_ground_truth_get_vd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field ias from hil_ground_truth message
 *
 * @return Indicated airspeed
 */
static inline float mavlink_msg_hil_ground_truth_get_ias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field tas from hil_ground_truth message
 *
 * @return True airspeed
 */
static inline float mavlink_msg_hil_ground_truth_get_tas(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field wind_speed from hil_ground_truth message
 *
 * @return Wind speed (2D)
 */
static inline float mavlink_msg_hil_ground_truth_get_wind_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field wind_dir from hil_ground_truth message
 *
 * @return Wind direction (2D)
 */
static inline float mavlink_msg_hil_ground_truth_get_wind_dir(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Decode a hil_ground_truth message into a struct
 *
 * @param msg The message to decode
 * @param hil_ground_truth C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_ground_truth_decode(const mavlink_message_t* msg, mavlink_hil_ground_truth_t* hil_ground_truth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_ground_truth->timestamp = mavlink_msg_hil_ground_truth_get_timestamp(msg);
    hil_ground_truth->roll = mavlink_msg_hil_ground_truth_get_roll(msg);
    hil_ground_truth->pitch = mavlink_msg_hil_ground_truth_get_pitch(msg);
    hil_ground_truth->yaw = mavlink_msg_hil_ground_truth_get_yaw(msg);
    hil_ground_truth->roll_rate = mavlink_msg_hil_ground_truth_get_roll_rate(msg);
    hil_ground_truth->pitch_rate = mavlink_msg_hil_ground_truth_get_pitch_rate(msg);
    hil_ground_truth->yaw_rate = mavlink_msg_hil_ground_truth_get_yaw_rate(msg);
    hil_ground_truth->lat = mavlink_msg_hil_ground_truth_get_lat(msg);
    hil_ground_truth->lon = mavlink_msg_hil_ground_truth_get_lon(msg);
    hil_ground_truth->alt = mavlink_msg_hil_ground_truth_get_alt(msg);
    hil_ground_truth->vn = mavlink_msg_hil_ground_truth_get_vn(msg);
    hil_ground_truth->ve = mavlink_msg_hil_ground_truth_get_ve(msg);
    hil_ground_truth->vd = mavlink_msg_hil_ground_truth_get_vd(msg);
    hil_ground_truth->ias = mavlink_msg_hil_ground_truth_get_ias(msg);
    hil_ground_truth->tas = mavlink_msg_hil_ground_truth_get_tas(msg);
    hil_ground_truth->wind_speed = mavlink_msg_hil_ground_truth_get_wind_speed(msg);
    hil_ground_truth->wind_dir = mavlink_msg_hil_ground_truth_get_wind_dir(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN? msg->len : MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN;
        memset(hil_ground_truth, 0, MAVLINK_MSG_ID_HIL_GROUND_TRUTH_LEN);
    memcpy(hil_ground_truth, _MAV_PAYLOAD(msg), len);
#endif
}
