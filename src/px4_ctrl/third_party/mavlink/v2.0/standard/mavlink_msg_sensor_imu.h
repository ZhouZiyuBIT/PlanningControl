#pragma once
// MESSAGE SENSOR_IMU PACKING

#define MAVLINK_MSG_ID_SENSOR_IMU 18182


typedef struct __mavlink_sensor_imu_t {
 float wx; /*<  angular velocity x*/
 float wy; /*<  angular velocity y*/
 float wz; /*<  angular velocity z*/
 float ax; /*<  acceleration x*/
 float ay; /*<  acceleration y*/
 float az; /*<  acceleration z*/
} mavlink_sensor_imu_t;

#define MAVLINK_MSG_ID_SENSOR_IMU_LEN 24
#define MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN 24
#define MAVLINK_MSG_ID_18182_LEN 24
#define MAVLINK_MSG_ID_18182_MIN_LEN 24

#define MAVLINK_MSG_ID_SENSOR_IMU_CRC 235
#define MAVLINK_MSG_ID_18182_CRC 235



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSOR_IMU { \
    18182, \
    "SENSOR_IMU", \
    6, \
    {  { "wx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_imu_t, wx) }, \
         { "wy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_imu_t, wy) }, \
         { "wz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_imu_t, wz) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_imu_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_imu_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_imu_t, az) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSOR_IMU { \
    "SENSOR_IMU", \
    6, \
    {  { "wx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_imu_t, wx) }, \
         { "wy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_imu_t, wy) }, \
         { "wz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_imu_t, wz) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_imu_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_imu_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_imu_t, az) }, \
         } \
}
#endif

/**
 * @brief Pack a sensor_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param wx  angular velocity x
 * @param wy  angular velocity y
 * @param wz  angular velocity z
 * @param ax  acceleration x
 * @param ay  acceleration y
 * @param az  acceleration z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float wx, float wy, float wz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_IMU_LEN];
    _mav_put_float(buf, 0, wx);
    _mav_put_float(buf, 4, wy);
    _mav_put_float(buf, 8, wz);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_IMU_LEN);
#else
    mavlink_sensor_imu_t packet;
    packet.wx = wx;
    packet.wy = wy;
    packet.wz = wz;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN, MAVLINK_MSG_ID_SENSOR_IMU_LEN, MAVLINK_MSG_ID_SENSOR_IMU_CRC);
}

/**
 * @brief Pack a sensor_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wx  angular velocity x
 * @param wy  angular velocity y
 * @param wz  angular velocity z
 * @param ax  acceleration x
 * @param ay  acceleration y
 * @param az  acceleration z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float wx,float wy,float wz,float ax,float ay,float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_IMU_LEN];
    _mav_put_float(buf, 0, wx);
    _mav_put_float(buf, 4, wy);
    _mav_put_float(buf, 8, wz);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_IMU_LEN);
#else
    mavlink_sensor_imu_t packet;
    packet.wx = wx;
    packet.wy = wy;
    packet.wz = wz;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN, MAVLINK_MSG_ID_SENSOR_IMU_LEN, MAVLINK_MSG_ID_SENSOR_IMU_CRC);
}

/**
 * @brief Encode a sensor_imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_imu_t* sensor_imu)
{
    return mavlink_msg_sensor_imu_pack(system_id, component_id, msg, sensor_imu->wx, sensor_imu->wy, sensor_imu->wz, sensor_imu->ax, sensor_imu->ay, sensor_imu->az);
}

/**
 * @brief Encode a sensor_imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_imu_t* sensor_imu)
{
    return mavlink_msg_sensor_imu_pack_chan(system_id, component_id, chan, msg, sensor_imu->wx, sensor_imu->wy, sensor_imu->wz, sensor_imu->ax, sensor_imu->ay, sensor_imu->az);
}

/**
 * @brief Send a sensor_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param wx  angular velocity x
 * @param wy  angular velocity y
 * @param wz  angular velocity z
 * @param ax  acceleration x
 * @param ay  acceleration y
 * @param az  acceleration z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_imu_send(mavlink_channel_t chan, float wx, float wy, float wz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_IMU_LEN];
    _mav_put_float(buf, 0, wx);
    _mav_put_float(buf, 4, wy);
    _mav_put_float(buf, 8, wz);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_IMU, buf, MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN, MAVLINK_MSG_ID_SENSOR_IMU_LEN, MAVLINK_MSG_ID_SENSOR_IMU_CRC);
#else
    mavlink_sensor_imu_t packet;
    packet.wx = wx;
    packet.wy = wy;
    packet.wz = wz;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_IMU, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN, MAVLINK_MSG_ID_SENSOR_IMU_LEN, MAVLINK_MSG_ID_SENSOR_IMU_CRC);
#endif
}

/**
 * @brief Send a sensor_imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensor_imu_send_struct(mavlink_channel_t chan, const mavlink_sensor_imu_t* sensor_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_imu_send(chan, sensor_imu->wx, sensor_imu->wy, sensor_imu->wz, sensor_imu->ax, sensor_imu->ay, sensor_imu->az);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_IMU, (const char *)sensor_imu, MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN, MAVLINK_MSG_ID_SENSOR_IMU_LEN, MAVLINK_MSG_ID_SENSOR_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSOR_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float wx, float wy, float wz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, wx);
    _mav_put_float(buf, 4, wy);
    _mav_put_float(buf, 8, wz);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_IMU, buf, MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN, MAVLINK_MSG_ID_SENSOR_IMU_LEN, MAVLINK_MSG_ID_SENSOR_IMU_CRC);
#else
    mavlink_sensor_imu_t *packet = (mavlink_sensor_imu_t *)msgbuf;
    packet->wx = wx;
    packet->wy = wy;
    packet->wz = wz;
    packet->ax = ax;
    packet->ay = ay;
    packet->az = az;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_IMU, (const char *)packet, MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN, MAVLINK_MSG_ID_SENSOR_IMU_LEN, MAVLINK_MSG_ID_SENSOR_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSOR_IMU UNPACKING


/**
 * @brief Get field wx from sensor_imu message
 *
 * @return  angular velocity x
 */
static inline float mavlink_msg_sensor_imu_get_wx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field wy from sensor_imu message
 *
 * @return  angular velocity y
 */
static inline float mavlink_msg_sensor_imu_get_wy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field wz from sensor_imu message
 *
 * @return  angular velocity z
 */
static inline float mavlink_msg_sensor_imu_get_wz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ax from sensor_imu message
 *
 * @return  acceleration x
 */
static inline float mavlink_msg_sensor_imu_get_ax(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ay from sensor_imu message
 *
 * @return  acceleration y
 */
static inline float mavlink_msg_sensor_imu_get_ay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field az from sensor_imu message
 *
 * @return  acceleration z
 */
static inline float mavlink_msg_sensor_imu_get_az(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a sensor_imu message into a struct
 *
 * @param msg The message to decode
 * @param sensor_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_imu_decode(const mavlink_message_t* msg, mavlink_sensor_imu_t* sensor_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensor_imu->wx = mavlink_msg_sensor_imu_get_wx(msg);
    sensor_imu->wy = mavlink_msg_sensor_imu_get_wy(msg);
    sensor_imu->wz = mavlink_msg_sensor_imu_get_wz(msg);
    sensor_imu->ax = mavlink_msg_sensor_imu_get_ax(msg);
    sensor_imu->ay = mavlink_msg_sensor_imu_get_ay(msg);
    sensor_imu->az = mavlink_msg_sensor_imu_get_az(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSOR_IMU_LEN? msg->len : MAVLINK_MSG_ID_SENSOR_IMU_LEN;
        memset(sensor_imu, 0, MAVLINK_MSG_ID_SENSOR_IMU_LEN);
    memcpy(sensor_imu, _MAV_PAYLOAD(msg), len);
#endif
}
