#pragma once
// MESSAGE NPFG_STATUS PACKING

#define MAVLINK_MSG_ID_NPFG_STATUS 213

MAVPACKED(
typedef struct __mavlink_npfg_status_t {
 uint64_t timestamp; /*< Timestamp*/
 float lateral_acceleration_demand; /*< Lateral acceleration demand*/
 float heading_ref; /*< Heading reference*/
 float heading_error; /*< Heading error*/
 float nav_bearing; /*< Look-ahead bearing*/
 float bearing_feas; /*< Bearing feasibility*/
 float track_error; /*< Track error*/
 float track_error_bound; /*< Track error boundary*/
 float p_gain; /*< Proportional gain*/
 float airsp_incr_w; /*< Airspeed increment due to wind speed*/
 float airsp_incr_e; /*< Airspeed increment due to track error*/
 uint8_t wind_condition; /*< Wind condition*/
}) mavlink_npfg_status_t;

#define MAVLINK_MSG_ID_NPFG_STATUS_LEN 49
#define MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN 49
#define MAVLINK_MSG_ID_213_LEN 49
#define MAVLINK_MSG_ID_213_MIN_LEN 49

#define MAVLINK_MSG_ID_NPFG_STATUS_CRC 13
#define MAVLINK_MSG_ID_213_CRC 13



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NPFG_STATUS { \
    213, \
    "NPFG_STATUS", \
    12, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_npfg_status_t, timestamp) }, \
         { "lateral_acceleration_demand", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_npfg_status_t, lateral_acceleration_demand) }, \
         { "heading_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_npfg_status_t, heading_ref) }, \
         { "heading_error", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_npfg_status_t, heading_error) }, \
         { "nav_bearing", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_npfg_status_t, nav_bearing) }, \
         { "bearing_feas", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_npfg_status_t, bearing_feas) }, \
         { "track_error", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_npfg_status_t, track_error) }, \
         { "track_error_bound", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_npfg_status_t, track_error_bound) }, \
         { "p_gain", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_npfg_status_t, p_gain) }, \
         { "airsp_incr_w", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_npfg_status_t, airsp_incr_w) }, \
         { "airsp_incr_e", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_npfg_status_t, airsp_incr_e) }, \
         { "wind_condition", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_npfg_status_t, wind_condition) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NPFG_STATUS { \
    "NPFG_STATUS", \
    12, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_npfg_status_t, timestamp) }, \
         { "lateral_acceleration_demand", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_npfg_status_t, lateral_acceleration_demand) }, \
         { "heading_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_npfg_status_t, heading_ref) }, \
         { "heading_error", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_npfg_status_t, heading_error) }, \
         { "nav_bearing", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_npfg_status_t, nav_bearing) }, \
         { "bearing_feas", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_npfg_status_t, bearing_feas) }, \
         { "track_error", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_npfg_status_t, track_error) }, \
         { "track_error_bound", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_npfg_status_t, track_error_bound) }, \
         { "p_gain", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_npfg_status_t, p_gain) }, \
         { "airsp_incr_w", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_npfg_status_t, airsp_incr_w) }, \
         { "airsp_incr_e", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_npfg_status_t, airsp_incr_e) }, \
         { "wind_condition", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_npfg_status_t, wind_condition) }, \
         } \
}
#endif

/**
 * @brief Pack a npfg_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp
 * @param lateral_acceleration_demand Lateral acceleration demand
 * @param heading_ref Heading reference
 * @param heading_error Heading error
 * @param nav_bearing Look-ahead bearing
 * @param bearing_feas Bearing feasibility
 * @param track_error Track error
 * @param track_error_bound Track error boundary
 * @param p_gain Proportional gain
 * @param airsp_incr_w Airspeed increment due to wind speed
 * @param airsp_incr_e Airspeed increment due to track error
 * @param wind_condition Wind condition
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_npfg_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float lateral_acceleration_demand, float heading_ref, float heading_error, float nav_bearing, float bearing_feas, float track_error, float track_error_bound, float p_gain, float airsp_incr_w, float airsp_incr_e, uint8_t wind_condition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NPFG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, lateral_acceleration_demand);
    _mav_put_float(buf, 12, heading_ref);
    _mav_put_float(buf, 16, heading_error);
    _mav_put_float(buf, 20, nav_bearing);
    _mav_put_float(buf, 24, bearing_feas);
    _mav_put_float(buf, 28, track_error);
    _mav_put_float(buf, 32, track_error_bound);
    _mav_put_float(buf, 36, p_gain);
    _mav_put_float(buf, 40, airsp_incr_w);
    _mav_put_float(buf, 44, airsp_incr_e);
    _mav_put_uint8_t(buf, 48, wind_condition);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
#else
    mavlink_npfg_status_t packet;
    packet.timestamp = timestamp;
    packet.lateral_acceleration_demand = lateral_acceleration_demand;
    packet.heading_ref = heading_ref;
    packet.heading_error = heading_error;
    packet.nav_bearing = nav_bearing;
    packet.bearing_feas = bearing_feas;
    packet.track_error = track_error;
    packet.track_error_bound = track_error_bound;
    packet.p_gain = p_gain;
    packet.airsp_incr_w = airsp_incr_w;
    packet.airsp_incr_e = airsp_incr_e;
    packet.wind_condition = wind_condition;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NPFG_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
}

/**
 * @brief Pack a npfg_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param lateral_acceleration_demand Lateral acceleration demand
 * @param heading_ref Heading reference
 * @param heading_error Heading error
 * @param nav_bearing Look-ahead bearing
 * @param bearing_feas Bearing feasibility
 * @param track_error Track error
 * @param track_error_bound Track error boundary
 * @param p_gain Proportional gain
 * @param airsp_incr_w Airspeed increment due to wind speed
 * @param airsp_incr_e Airspeed increment due to track error
 * @param wind_condition Wind condition
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_npfg_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float lateral_acceleration_demand,float heading_ref,float heading_error,float nav_bearing,float bearing_feas,float track_error,float track_error_bound,float p_gain,float airsp_incr_w,float airsp_incr_e,uint8_t wind_condition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NPFG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, lateral_acceleration_demand);
    _mav_put_float(buf, 12, heading_ref);
    _mav_put_float(buf, 16, heading_error);
    _mav_put_float(buf, 20, nav_bearing);
    _mav_put_float(buf, 24, bearing_feas);
    _mav_put_float(buf, 28, track_error);
    _mav_put_float(buf, 32, track_error_bound);
    _mav_put_float(buf, 36, p_gain);
    _mav_put_float(buf, 40, airsp_incr_w);
    _mav_put_float(buf, 44, airsp_incr_e);
    _mav_put_uint8_t(buf, 48, wind_condition);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
#else
    mavlink_npfg_status_t packet;
    packet.timestamp = timestamp;
    packet.lateral_acceleration_demand = lateral_acceleration_demand;
    packet.heading_ref = heading_ref;
    packet.heading_error = heading_error;
    packet.nav_bearing = nav_bearing;
    packet.bearing_feas = bearing_feas;
    packet.track_error = track_error;
    packet.track_error_bound = track_error_bound;
    packet.p_gain = p_gain;
    packet.airsp_incr_w = airsp_incr_w;
    packet.airsp_incr_e = airsp_incr_e;
    packet.wind_condition = wind_condition;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NPFG_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
}

/**
 * @brief Encode a npfg_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param npfg_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_npfg_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_npfg_status_t* npfg_status)
{
    return mavlink_msg_npfg_status_pack(system_id, component_id, msg, npfg_status->timestamp, npfg_status->lateral_acceleration_demand, npfg_status->heading_ref, npfg_status->heading_error, npfg_status->nav_bearing, npfg_status->bearing_feas, npfg_status->track_error, npfg_status->track_error_bound, npfg_status->p_gain, npfg_status->airsp_incr_w, npfg_status->airsp_incr_e, npfg_status->wind_condition);
}

/**
 * @brief Encode a npfg_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param npfg_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_npfg_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_npfg_status_t* npfg_status)
{
    return mavlink_msg_npfg_status_pack_chan(system_id, component_id, chan, msg, npfg_status->timestamp, npfg_status->lateral_acceleration_demand, npfg_status->heading_ref, npfg_status->heading_error, npfg_status->nav_bearing, npfg_status->bearing_feas, npfg_status->track_error, npfg_status->track_error_bound, npfg_status->p_gain, npfg_status->airsp_incr_w, npfg_status->airsp_incr_e, npfg_status->wind_condition);
}

/**
 * @brief Send a npfg_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param lateral_acceleration_demand Lateral acceleration demand
 * @param heading_ref Heading reference
 * @param heading_error Heading error
 * @param nav_bearing Look-ahead bearing
 * @param bearing_feas Bearing feasibility
 * @param track_error Track error
 * @param track_error_bound Track error boundary
 * @param p_gain Proportional gain
 * @param airsp_incr_w Airspeed increment due to wind speed
 * @param airsp_incr_e Airspeed increment due to track error
 * @param wind_condition Wind condition
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_npfg_status_send(mavlink_channel_t chan, uint64_t timestamp, float lateral_acceleration_demand, float heading_ref, float heading_error, float nav_bearing, float bearing_feas, float track_error, float track_error_bound, float p_gain, float airsp_incr_w, float airsp_incr_e, uint8_t wind_condition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NPFG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, lateral_acceleration_demand);
    _mav_put_float(buf, 12, heading_ref);
    _mav_put_float(buf, 16, heading_error);
    _mav_put_float(buf, 20, nav_bearing);
    _mav_put_float(buf, 24, bearing_feas);
    _mav_put_float(buf, 28, track_error);
    _mav_put_float(buf, 32, track_error_bound);
    _mav_put_float(buf, 36, p_gain);
    _mav_put_float(buf, 40, airsp_incr_w);
    _mav_put_float(buf, 44, airsp_incr_e);
    _mav_put_uint8_t(buf, 48, wind_condition);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, buf, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#else
    mavlink_npfg_status_t packet;
    packet.timestamp = timestamp;
    packet.lateral_acceleration_demand = lateral_acceleration_demand;
    packet.heading_ref = heading_ref;
    packet.heading_error = heading_error;
    packet.nav_bearing = nav_bearing;
    packet.bearing_feas = bearing_feas;
    packet.track_error = track_error;
    packet.track_error_bound = track_error_bound;
    packet.p_gain = p_gain;
    packet.airsp_incr_w = airsp_incr_w;
    packet.airsp_incr_e = airsp_incr_e;
    packet.wind_condition = wind_condition;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, (const char *)&packet, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#endif
}

/**
 * @brief Send a npfg_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_npfg_status_send_struct(mavlink_channel_t chan, const mavlink_npfg_status_t* npfg_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_npfg_status_send(chan, npfg_status->timestamp, npfg_status->lateral_acceleration_demand, npfg_status->heading_ref, npfg_status->heading_error, npfg_status->nav_bearing, npfg_status->bearing_feas, npfg_status->track_error, npfg_status->track_error_bound, npfg_status->p_gain, npfg_status->airsp_incr_w, npfg_status->airsp_incr_e, npfg_status->wind_condition);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, (const char *)npfg_status, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_NPFG_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_npfg_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float lateral_acceleration_demand, float heading_ref, float heading_error, float nav_bearing, float bearing_feas, float track_error, float track_error_bound, float p_gain, float airsp_incr_w, float airsp_incr_e, uint8_t wind_condition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, lateral_acceleration_demand);
    _mav_put_float(buf, 12, heading_ref);
    _mav_put_float(buf, 16, heading_error);
    _mav_put_float(buf, 20, nav_bearing);
    _mav_put_float(buf, 24, bearing_feas);
    _mav_put_float(buf, 28, track_error);
    _mav_put_float(buf, 32, track_error_bound);
    _mav_put_float(buf, 36, p_gain);
    _mav_put_float(buf, 40, airsp_incr_w);
    _mav_put_float(buf, 44, airsp_incr_e);
    _mav_put_uint8_t(buf, 48, wind_condition);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, buf, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#else
    mavlink_npfg_status_t *packet = (mavlink_npfg_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->lateral_acceleration_demand = lateral_acceleration_demand;
    packet->heading_ref = heading_ref;
    packet->heading_error = heading_error;
    packet->nav_bearing = nav_bearing;
    packet->bearing_feas = bearing_feas;
    packet->track_error = track_error;
    packet->track_error_bound = track_error_bound;
    packet->p_gain = p_gain;
    packet->airsp_incr_w = airsp_incr_w;
    packet->airsp_incr_e = airsp_incr_e;
    packet->wind_condition = wind_condition;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, (const char *)packet, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE NPFG_STATUS UNPACKING


/**
 * @brief Get field timestamp from npfg_status message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_npfg_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lateral_acceleration_demand from npfg_status message
 *
 * @return Lateral acceleration demand
 */
static inline float mavlink_msg_npfg_status_get_lateral_acceleration_demand(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field heading_ref from npfg_status message
 *
 * @return Heading reference
 */
static inline float mavlink_msg_npfg_status_get_heading_ref(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field heading_error from npfg_status message
 *
 * @return Heading error
 */
static inline float mavlink_msg_npfg_status_get_heading_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field nav_bearing from npfg_status message
 *
 * @return Look-ahead bearing
 */
static inline float mavlink_msg_npfg_status_get_nav_bearing(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field bearing_feas from npfg_status message
 *
 * @return Bearing feasibility
 */
static inline float mavlink_msg_npfg_status_get_bearing_feas(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field track_error from npfg_status message
 *
 * @return Track error
 */
static inline float mavlink_msg_npfg_status_get_track_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field track_error_bound from npfg_status message
 *
 * @return Track error boundary
 */
static inline float mavlink_msg_npfg_status_get_track_error_bound(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field p_gain from npfg_status message
 *
 * @return Proportional gain
 */
static inline float mavlink_msg_npfg_status_get_p_gain(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field airsp_incr_w from npfg_status message
 *
 * @return Airspeed increment due to wind speed
 */
static inline float mavlink_msg_npfg_status_get_airsp_incr_w(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field airsp_incr_e from npfg_status message
 *
 * @return Airspeed increment due to track error
 */
static inline float mavlink_msg_npfg_status_get_airsp_incr_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field wind_condition from npfg_status message
 *
 * @return Wind condition
 */
static inline uint8_t mavlink_msg_npfg_status_get_wind_condition(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Decode a npfg_status message into a struct
 *
 * @param msg The message to decode
 * @param npfg_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_npfg_status_decode(const mavlink_message_t* msg, mavlink_npfg_status_t* npfg_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    npfg_status->timestamp = mavlink_msg_npfg_status_get_timestamp(msg);
    npfg_status->lateral_acceleration_demand = mavlink_msg_npfg_status_get_lateral_acceleration_demand(msg);
    npfg_status->heading_ref = mavlink_msg_npfg_status_get_heading_ref(msg);
    npfg_status->heading_error = mavlink_msg_npfg_status_get_heading_error(msg);
    npfg_status->nav_bearing = mavlink_msg_npfg_status_get_nav_bearing(msg);
    npfg_status->bearing_feas = mavlink_msg_npfg_status_get_bearing_feas(msg);
    npfg_status->track_error = mavlink_msg_npfg_status_get_track_error(msg);
    npfg_status->track_error_bound = mavlink_msg_npfg_status_get_track_error_bound(msg);
    npfg_status->p_gain = mavlink_msg_npfg_status_get_p_gain(msg);
    npfg_status->airsp_incr_w = mavlink_msg_npfg_status_get_airsp_incr_w(msg);
    npfg_status->airsp_incr_e = mavlink_msg_npfg_status_get_airsp_incr_e(msg);
    npfg_status->wind_condition = mavlink_msg_npfg_status_get_wind_condition(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NPFG_STATUS_LEN? msg->len : MAVLINK_MSG_ID_NPFG_STATUS_LEN;
        memset(npfg_status, 0, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
    memcpy(npfg_status, _MAV_PAYLOAD(msg), len);
#endif
}
