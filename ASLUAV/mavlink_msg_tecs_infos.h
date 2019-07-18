#pragma once
// MESSAGE TECS_INFOS PACKING

#define MAVLINK_MSG_ID_TECS_INFOS 216

MAVPACKED(
typedef struct __mavlink_tecs_infos_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float altitudeSp; /*<  Altitude Setpoint*/
 float altitude_filtered; /*<  Altitude*/
 float flightPathAngleSp; /*<  Flight Path Angle Setpoint*/
 float flightPathAngle; /*<  Flight Path Angle*/
 float airspeedSp; /*<  Airspeed Setpoint*/
 float airspeed_filtered; /*<  Airspeed*/
 float airspeedDerivativeSp; /*<  Airspeed Derivative Setpoint*/
 float airspeedDerivative; /*<  Airspeed Derivative*/
 float totalEnergyError; /*<  Total Energy Error*/
 float energyDistributionError; /*<  Energy Distribution Error*/
 float totalEnergyRateError; /*<  Total Energy Rate Error*/
 float energyDistributionRateError; /*<  Energy Distribution Rate Error*/
 float throttle_integ; /*<  Throttle Integrator*/
 float pitch_integ; /*<  Pitch Integrator*/
 uint8_t mode; /*<  Tecs Mode*/
 uint8_t active; /*<  Am I the active Tecs*/
}) mavlink_tecs_infos_t;

#define MAVLINK_MSG_ID_TECS_INFOS_LEN 66
#define MAVLINK_MSG_ID_TECS_INFOS_MIN_LEN 66
#define MAVLINK_MSG_ID_216_LEN 66
#define MAVLINK_MSG_ID_216_MIN_LEN 66

#define MAVLINK_MSG_ID_TECS_INFOS_CRC 141
#define MAVLINK_MSG_ID_216_CRC 141



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TECS_INFOS { \
    216, \
    "TECS_INFOS", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tecs_infos_t, timestamp) }, \
         { "altitudeSp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tecs_infos_t, altitudeSp) }, \
         { "altitude_filtered", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tecs_infos_t, altitude_filtered) }, \
         { "flightPathAngleSp", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_tecs_infos_t, flightPathAngleSp) }, \
         { "flightPathAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_tecs_infos_t, flightPathAngle) }, \
         { "airspeedSp", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_tecs_infos_t, airspeedSp) }, \
         { "airspeed_filtered", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_tecs_infos_t, airspeed_filtered) }, \
         { "airspeedDerivativeSp", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_tecs_infos_t, airspeedDerivativeSp) }, \
         { "airspeedDerivative", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_tecs_infos_t, airspeedDerivative) }, \
         { "totalEnergyError", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_tecs_infos_t, totalEnergyError) }, \
         { "energyDistributionError", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_tecs_infos_t, energyDistributionError) }, \
         { "totalEnergyRateError", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_tecs_infos_t, totalEnergyRateError) }, \
         { "energyDistributionRateError", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_tecs_infos_t, energyDistributionRateError) }, \
         { "throttle_integ", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_tecs_infos_t, throttle_integ) }, \
         { "pitch_integ", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_tecs_infos_t, pitch_integ) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_tecs_infos_t, mode) }, \
         { "active", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_tecs_infos_t, active) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TECS_INFOS { \
    "TECS_INFOS", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tecs_infos_t, timestamp) }, \
         { "altitudeSp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tecs_infos_t, altitudeSp) }, \
         { "altitude_filtered", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tecs_infos_t, altitude_filtered) }, \
         { "flightPathAngleSp", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_tecs_infos_t, flightPathAngleSp) }, \
         { "flightPathAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_tecs_infos_t, flightPathAngle) }, \
         { "airspeedSp", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_tecs_infos_t, airspeedSp) }, \
         { "airspeed_filtered", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_tecs_infos_t, airspeed_filtered) }, \
         { "airspeedDerivativeSp", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_tecs_infos_t, airspeedDerivativeSp) }, \
         { "airspeedDerivative", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_tecs_infos_t, airspeedDerivative) }, \
         { "totalEnergyError", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_tecs_infos_t, totalEnergyError) }, \
         { "energyDistributionError", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_tecs_infos_t, energyDistributionError) }, \
         { "totalEnergyRateError", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_tecs_infos_t, totalEnergyRateError) }, \
         { "energyDistributionRateError", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_tecs_infos_t, energyDistributionRateError) }, \
         { "throttle_integ", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_tecs_infos_t, throttle_integ) }, \
         { "pitch_integ", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_tecs_infos_t, pitch_integ) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_tecs_infos_t, mode) }, \
         { "active", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_tecs_infos_t, active) }, \
         } \
}
#endif

/**
 * @brief Pack a tecs_infos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param altitudeSp  Altitude Setpoint
 * @param altitude_filtered  Altitude
 * @param flightPathAngleSp  Flight Path Angle Setpoint
 * @param flightPathAngle  Flight Path Angle
 * @param airspeedSp  Airspeed Setpoint
 * @param airspeed_filtered  Airspeed
 * @param airspeedDerivativeSp  Airspeed Derivative Setpoint
 * @param airspeedDerivative  Airspeed Derivative
 * @param totalEnergyError  Total Energy Error
 * @param energyDistributionError  Energy Distribution Error
 * @param totalEnergyRateError  Total Energy Rate Error
 * @param energyDistributionRateError  Energy Distribution Rate Error
 * @param throttle_integ  Throttle Integrator
 * @param pitch_integ  Pitch Integrator
 * @param mode  Tecs Mode
 * @param active  Am I the active Tecs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tecs_infos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float altitudeSp, float altitude_filtered, float flightPathAngleSp, float flightPathAngle, float airspeedSp, float airspeed_filtered, float airspeedDerivativeSp, float airspeedDerivative, float totalEnergyError, float energyDistributionError, float totalEnergyRateError, float energyDistributionRateError, float throttle_integ, float pitch_integ, uint8_t mode, uint8_t active)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TECS_INFOS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, altitudeSp);
    _mav_put_float(buf, 12, altitude_filtered);
    _mav_put_float(buf, 16, flightPathAngleSp);
    _mav_put_float(buf, 20, flightPathAngle);
    _mav_put_float(buf, 24, airspeedSp);
    _mav_put_float(buf, 28, airspeed_filtered);
    _mav_put_float(buf, 32, airspeedDerivativeSp);
    _mav_put_float(buf, 36, airspeedDerivative);
    _mav_put_float(buf, 40, totalEnergyError);
    _mav_put_float(buf, 44, energyDistributionError);
    _mav_put_float(buf, 48, totalEnergyRateError);
    _mav_put_float(buf, 52, energyDistributionRateError);
    _mav_put_float(buf, 56, throttle_integ);
    _mav_put_float(buf, 60, pitch_integ);
    _mav_put_uint8_t(buf, 64, mode);
    _mav_put_uint8_t(buf, 65, active);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TECS_INFOS_LEN);
#else
    mavlink_tecs_infos_t packet;
    packet.timestamp = timestamp;
    packet.altitudeSp = altitudeSp;
    packet.altitude_filtered = altitude_filtered;
    packet.flightPathAngleSp = flightPathAngleSp;
    packet.flightPathAngle = flightPathAngle;
    packet.airspeedSp = airspeedSp;
    packet.airspeed_filtered = airspeed_filtered;
    packet.airspeedDerivativeSp = airspeedDerivativeSp;
    packet.airspeedDerivative = airspeedDerivative;
    packet.totalEnergyError = totalEnergyError;
    packet.energyDistributionError = energyDistributionError;
    packet.totalEnergyRateError = totalEnergyRateError;
    packet.energyDistributionRateError = energyDistributionRateError;
    packet.throttle_integ = throttle_integ;
    packet.pitch_integ = pitch_integ;
    packet.mode = mode;
    packet.active = active;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TECS_INFOS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TECS_INFOS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TECS_INFOS_MIN_LEN, MAVLINK_MSG_ID_TECS_INFOS_LEN, MAVLINK_MSG_ID_TECS_INFOS_CRC);
}

/**
 * @brief Pack a tecs_infos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param altitudeSp  Altitude Setpoint
 * @param altitude_filtered  Altitude
 * @param flightPathAngleSp  Flight Path Angle Setpoint
 * @param flightPathAngle  Flight Path Angle
 * @param airspeedSp  Airspeed Setpoint
 * @param airspeed_filtered  Airspeed
 * @param airspeedDerivativeSp  Airspeed Derivative Setpoint
 * @param airspeedDerivative  Airspeed Derivative
 * @param totalEnergyError  Total Energy Error
 * @param energyDistributionError  Energy Distribution Error
 * @param totalEnergyRateError  Total Energy Rate Error
 * @param energyDistributionRateError  Energy Distribution Rate Error
 * @param throttle_integ  Throttle Integrator
 * @param pitch_integ  Pitch Integrator
 * @param mode  Tecs Mode
 * @param active  Am I the active Tecs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tecs_infos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float altitudeSp,float altitude_filtered,float flightPathAngleSp,float flightPathAngle,float airspeedSp,float airspeed_filtered,float airspeedDerivativeSp,float airspeedDerivative,float totalEnergyError,float energyDistributionError,float totalEnergyRateError,float energyDistributionRateError,float throttle_integ,float pitch_integ,uint8_t mode,uint8_t active)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TECS_INFOS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, altitudeSp);
    _mav_put_float(buf, 12, altitude_filtered);
    _mav_put_float(buf, 16, flightPathAngleSp);
    _mav_put_float(buf, 20, flightPathAngle);
    _mav_put_float(buf, 24, airspeedSp);
    _mav_put_float(buf, 28, airspeed_filtered);
    _mav_put_float(buf, 32, airspeedDerivativeSp);
    _mav_put_float(buf, 36, airspeedDerivative);
    _mav_put_float(buf, 40, totalEnergyError);
    _mav_put_float(buf, 44, energyDistributionError);
    _mav_put_float(buf, 48, totalEnergyRateError);
    _mav_put_float(buf, 52, energyDistributionRateError);
    _mav_put_float(buf, 56, throttle_integ);
    _mav_put_float(buf, 60, pitch_integ);
    _mav_put_uint8_t(buf, 64, mode);
    _mav_put_uint8_t(buf, 65, active);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TECS_INFOS_LEN);
#else
    mavlink_tecs_infos_t packet;
    packet.timestamp = timestamp;
    packet.altitudeSp = altitudeSp;
    packet.altitude_filtered = altitude_filtered;
    packet.flightPathAngleSp = flightPathAngleSp;
    packet.flightPathAngle = flightPathAngle;
    packet.airspeedSp = airspeedSp;
    packet.airspeed_filtered = airspeed_filtered;
    packet.airspeedDerivativeSp = airspeedDerivativeSp;
    packet.airspeedDerivative = airspeedDerivative;
    packet.totalEnergyError = totalEnergyError;
    packet.energyDistributionError = energyDistributionError;
    packet.totalEnergyRateError = totalEnergyRateError;
    packet.energyDistributionRateError = energyDistributionRateError;
    packet.throttle_integ = throttle_integ;
    packet.pitch_integ = pitch_integ;
    packet.mode = mode;
    packet.active = active;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TECS_INFOS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TECS_INFOS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TECS_INFOS_MIN_LEN, MAVLINK_MSG_ID_TECS_INFOS_LEN, MAVLINK_MSG_ID_TECS_INFOS_CRC);
}

/**
 * @brief Encode a tecs_infos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tecs_infos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tecs_infos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tecs_infos_t* tecs_infos)
{
    return mavlink_msg_tecs_infos_pack(system_id, component_id, msg, tecs_infos->timestamp, tecs_infos->altitudeSp, tecs_infos->altitude_filtered, tecs_infos->flightPathAngleSp, tecs_infos->flightPathAngle, tecs_infos->airspeedSp, tecs_infos->airspeed_filtered, tecs_infos->airspeedDerivativeSp, tecs_infos->airspeedDerivative, tecs_infos->totalEnergyError, tecs_infos->energyDistributionError, tecs_infos->totalEnergyRateError, tecs_infos->energyDistributionRateError, tecs_infos->throttle_integ, tecs_infos->pitch_integ, tecs_infos->mode, tecs_infos->active);
}

/**
 * @brief Encode a tecs_infos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tecs_infos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tecs_infos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tecs_infos_t* tecs_infos)
{
    return mavlink_msg_tecs_infos_pack_chan(system_id, component_id, chan, msg, tecs_infos->timestamp, tecs_infos->altitudeSp, tecs_infos->altitude_filtered, tecs_infos->flightPathAngleSp, tecs_infos->flightPathAngle, tecs_infos->airspeedSp, tecs_infos->airspeed_filtered, tecs_infos->airspeedDerivativeSp, tecs_infos->airspeedDerivative, tecs_infos->totalEnergyError, tecs_infos->energyDistributionError, tecs_infos->totalEnergyRateError, tecs_infos->energyDistributionRateError, tecs_infos->throttle_integ, tecs_infos->pitch_integ, tecs_infos->mode, tecs_infos->active);
}

/**
 * @brief Send a tecs_infos message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param altitudeSp  Altitude Setpoint
 * @param altitude_filtered  Altitude
 * @param flightPathAngleSp  Flight Path Angle Setpoint
 * @param flightPathAngle  Flight Path Angle
 * @param airspeedSp  Airspeed Setpoint
 * @param airspeed_filtered  Airspeed
 * @param airspeedDerivativeSp  Airspeed Derivative Setpoint
 * @param airspeedDerivative  Airspeed Derivative
 * @param totalEnergyError  Total Energy Error
 * @param energyDistributionError  Energy Distribution Error
 * @param totalEnergyRateError  Total Energy Rate Error
 * @param energyDistributionRateError  Energy Distribution Rate Error
 * @param throttle_integ  Throttle Integrator
 * @param pitch_integ  Pitch Integrator
 * @param mode  Tecs Mode
 * @param active  Am I the active Tecs
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tecs_infos_send(mavlink_channel_t chan, uint64_t timestamp, float altitudeSp, float altitude_filtered, float flightPathAngleSp, float flightPathAngle, float airspeedSp, float airspeed_filtered, float airspeedDerivativeSp, float airspeedDerivative, float totalEnergyError, float energyDistributionError, float totalEnergyRateError, float energyDistributionRateError, float throttle_integ, float pitch_integ, uint8_t mode, uint8_t active)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TECS_INFOS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, altitudeSp);
    _mav_put_float(buf, 12, altitude_filtered);
    _mav_put_float(buf, 16, flightPathAngleSp);
    _mav_put_float(buf, 20, flightPathAngle);
    _mav_put_float(buf, 24, airspeedSp);
    _mav_put_float(buf, 28, airspeed_filtered);
    _mav_put_float(buf, 32, airspeedDerivativeSp);
    _mav_put_float(buf, 36, airspeedDerivative);
    _mav_put_float(buf, 40, totalEnergyError);
    _mav_put_float(buf, 44, energyDistributionError);
    _mav_put_float(buf, 48, totalEnergyRateError);
    _mav_put_float(buf, 52, energyDistributionRateError);
    _mav_put_float(buf, 56, throttle_integ);
    _mav_put_float(buf, 60, pitch_integ);
    _mav_put_uint8_t(buf, 64, mode);
    _mav_put_uint8_t(buf, 65, active);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TECS_INFOS, buf, MAVLINK_MSG_ID_TECS_INFOS_MIN_LEN, MAVLINK_MSG_ID_TECS_INFOS_LEN, MAVLINK_MSG_ID_TECS_INFOS_CRC);
#else
    mavlink_tecs_infos_t packet;
    packet.timestamp = timestamp;
    packet.altitudeSp = altitudeSp;
    packet.altitude_filtered = altitude_filtered;
    packet.flightPathAngleSp = flightPathAngleSp;
    packet.flightPathAngle = flightPathAngle;
    packet.airspeedSp = airspeedSp;
    packet.airspeed_filtered = airspeed_filtered;
    packet.airspeedDerivativeSp = airspeedDerivativeSp;
    packet.airspeedDerivative = airspeedDerivative;
    packet.totalEnergyError = totalEnergyError;
    packet.energyDistributionError = energyDistributionError;
    packet.totalEnergyRateError = totalEnergyRateError;
    packet.energyDistributionRateError = energyDistributionRateError;
    packet.throttle_integ = throttle_integ;
    packet.pitch_integ = pitch_integ;
    packet.mode = mode;
    packet.active = active;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TECS_INFOS, (const char *)&packet, MAVLINK_MSG_ID_TECS_INFOS_MIN_LEN, MAVLINK_MSG_ID_TECS_INFOS_LEN, MAVLINK_MSG_ID_TECS_INFOS_CRC);
#endif
}

/**
 * @brief Send a tecs_infos message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tecs_infos_send_struct(mavlink_channel_t chan, const mavlink_tecs_infos_t* tecs_infos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tecs_infos_send(chan, tecs_infos->timestamp, tecs_infos->altitudeSp, tecs_infos->altitude_filtered, tecs_infos->flightPathAngleSp, tecs_infos->flightPathAngle, tecs_infos->airspeedSp, tecs_infos->airspeed_filtered, tecs_infos->airspeedDerivativeSp, tecs_infos->airspeedDerivative, tecs_infos->totalEnergyError, tecs_infos->energyDistributionError, tecs_infos->totalEnergyRateError, tecs_infos->energyDistributionRateError, tecs_infos->throttle_integ, tecs_infos->pitch_integ, tecs_infos->mode, tecs_infos->active);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TECS_INFOS, (const char *)tecs_infos, MAVLINK_MSG_ID_TECS_INFOS_MIN_LEN, MAVLINK_MSG_ID_TECS_INFOS_LEN, MAVLINK_MSG_ID_TECS_INFOS_CRC);
#endif
}

#if MAVLINK_MSG_ID_TECS_INFOS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tecs_infos_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float altitudeSp, float altitude_filtered, float flightPathAngleSp, float flightPathAngle, float airspeedSp, float airspeed_filtered, float airspeedDerivativeSp, float airspeedDerivative, float totalEnergyError, float energyDistributionError, float totalEnergyRateError, float energyDistributionRateError, float throttle_integ, float pitch_integ, uint8_t mode, uint8_t active)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, altitudeSp);
    _mav_put_float(buf, 12, altitude_filtered);
    _mav_put_float(buf, 16, flightPathAngleSp);
    _mav_put_float(buf, 20, flightPathAngle);
    _mav_put_float(buf, 24, airspeedSp);
    _mav_put_float(buf, 28, airspeed_filtered);
    _mav_put_float(buf, 32, airspeedDerivativeSp);
    _mav_put_float(buf, 36, airspeedDerivative);
    _mav_put_float(buf, 40, totalEnergyError);
    _mav_put_float(buf, 44, energyDistributionError);
    _mav_put_float(buf, 48, totalEnergyRateError);
    _mav_put_float(buf, 52, energyDistributionRateError);
    _mav_put_float(buf, 56, throttle_integ);
    _mav_put_float(buf, 60, pitch_integ);
    _mav_put_uint8_t(buf, 64, mode);
    _mav_put_uint8_t(buf, 65, active);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TECS_INFOS, buf, MAVLINK_MSG_ID_TECS_INFOS_MIN_LEN, MAVLINK_MSG_ID_TECS_INFOS_LEN, MAVLINK_MSG_ID_TECS_INFOS_CRC);
#else
    mavlink_tecs_infos_t *packet = (mavlink_tecs_infos_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->altitudeSp = altitudeSp;
    packet->altitude_filtered = altitude_filtered;
    packet->flightPathAngleSp = flightPathAngleSp;
    packet->flightPathAngle = flightPathAngle;
    packet->airspeedSp = airspeedSp;
    packet->airspeed_filtered = airspeed_filtered;
    packet->airspeedDerivativeSp = airspeedDerivativeSp;
    packet->airspeedDerivative = airspeedDerivative;
    packet->totalEnergyError = totalEnergyError;
    packet->energyDistributionError = energyDistributionError;
    packet->totalEnergyRateError = totalEnergyRateError;
    packet->energyDistributionRateError = energyDistributionRateError;
    packet->throttle_integ = throttle_integ;
    packet->pitch_integ = pitch_integ;
    packet->mode = mode;
    packet->active = active;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TECS_INFOS, (const char *)packet, MAVLINK_MSG_ID_TECS_INFOS_MIN_LEN, MAVLINK_MSG_ID_TECS_INFOS_LEN, MAVLINK_MSG_ID_TECS_INFOS_CRC);
#endif
}
#endif

#endif

// MESSAGE TECS_INFOS UNPACKING


/**
 * @brief Get field timestamp from tecs_infos message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_tecs_infos_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field altitudeSp from tecs_infos message
 *
 * @return  Altitude Setpoint
 */
static inline float mavlink_msg_tecs_infos_get_altitudeSp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field altitude_filtered from tecs_infos message
 *
 * @return  Altitude
 */
static inline float mavlink_msg_tecs_infos_get_altitude_filtered(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field flightPathAngleSp from tecs_infos message
 *
 * @return  Flight Path Angle Setpoint
 */
static inline float mavlink_msg_tecs_infos_get_flightPathAngleSp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field flightPathAngle from tecs_infos message
 *
 * @return  Flight Path Angle
 */
static inline float mavlink_msg_tecs_infos_get_flightPathAngle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field airspeedSp from tecs_infos message
 *
 * @return  Airspeed Setpoint
 */
static inline float mavlink_msg_tecs_infos_get_airspeedSp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field airspeed_filtered from tecs_infos message
 *
 * @return  Airspeed
 */
static inline float mavlink_msg_tecs_infos_get_airspeed_filtered(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field airspeedDerivativeSp from tecs_infos message
 *
 * @return  Airspeed Derivative Setpoint
 */
static inline float mavlink_msg_tecs_infos_get_airspeedDerivativeSp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field airspeedDerivative from tecs_infos message
 *
 * @return  Airspeed Derivative
 */
static inline float mavlink_msg_tecs_infos_get_airspeedDerivative(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field totalEnergyError from tecs_infos message
 *
 * @return  Total Energy Error
 */
static inline float mavlink_msg_tecs_infos_get_totalEnergyError(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field energyDistributionError from tecs_infos message
 *
 * @return  Energy Distribution Error
 */
static inline float mavlink_msg_tecs_infos_get_energyDistributionError(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field totalEnergyRateError from tecs_infos message
 *
 * @return  Total Energy Rate Error
 */
static inline float mavlink_msg_tecs_infos_get_totalEnergyRateError(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field energyDistributionRateError from tecs_infos message
 *
 * @return  Energy Distribution Rate Error
 */
static inline float mavlink_msg_tecs_infos_get_energyDistributionRateError(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field throttle_integ from tecs_infos message
 *
 * @return  Throttle Integrator
 */
static inline float mavlink_msg_tecs_infos_get_throttle_integ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field pitch_integ from tecs_infos message
 *
 * @return  Pitch Integrator
 */
static inline float mavlink_msg_tecs_infos_get_pitch_integ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field mode from tecs_infos message
 *
 * @return  Tecs Mode
 */
static inline uint8_t mavlink_msg_tecs_infos_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Get field active from tecs_infos message
 *
 * @return  Am I the active Tecs
 */
static inline uint8_t mavlink_msg_tecs_infos_get_active(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  65);
}

/**
 * @brief Decode a tecs_infos message into a struct
 *
 * @param msg The message to decode
 * @param tecs_infos C-struct to decode the message contents into
 */
static inline void mavlink_msg_tecs_infos_decode(const mavlink_message_t* msg, mavlink_tecs_infos_t* tecs_infos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tecs_infos->timestamp = mavlink_msg_tecs_infos_get_timestamp(msg);
    tecs_infos->altitudeSp = mavlink_msg_tecs_infos_get_altitudeSp(msg);
    tecs_infos->altitude_filtered = mavlink_msg_tecs_infos_get_altitude_filtered(msg);
    tecs_infos->flightPathAngleSp = mavlink_msg_tecs_infos_get_flightPathAngleSp(msg);
    tecs_infos->flightPathAngle = mavlink_msg_tecs_infos_get_flightPathAngle(msg);
    tecs_infos->airspeedSp = mavlink_msg_tecs_infos_get_airspeedSp(msg);
    tecs_infos->airspeed_filtered = mavlink_msg_tecs_infos_get_airspeed_filtered(msg);
    tecs_infos->airspeedDerivativeSp = mavlink_msg_tecs_infos_get_airspeedDerivativeSp(msg);
    tecs_infos->airspeedDerivative = mavlink_msg_tecs_infos_get_airspeedDerivative(msg);
    tecs_infos->totalEnergyError = mavlink_msg_tecs_infos_get_totalEnergyError(msg);
    tecs_infos->energyDistributionError = mavlink_msg_tecs_infos_get_energyDistributionError(msg);
    tecs_infos->totalEnergyRateError = mavlink_msg_tecs_infos_get_totalEnergyRateError(msg);
    tecs_infos->energyDistributionRateError = mavlink_msg_tecs_infos_get_energyDistributionRateError(msg);
    tecs_infos->throttle_integ = mavlink_msg_tecs_infos_get_throttle_integ(msg);
    tecs_infos->pitch_integ = mavlink_msg_tecs_infos_get_pitch_integ(msg);
    tecs_infos->mode = mavlink_msg_tecs_infos_get_mode(msg);
    tecs_infos->active = mavlink_msg_tecs_infos_get_active(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TECS_INFOS_LEN? msg->len : MAVLINK_MSG_ID_TECS_INFOS_LEN;
        memset(tecs_infos, 0, MAVLINK_MSG_ID_TECS_INFOS_LEN);
    memcpy(tecs_infos, _MAV_PAYLOAD(msg), len);
#endif
}
