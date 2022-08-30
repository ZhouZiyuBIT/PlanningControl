#pragma once
// MESSAGE QUADROTOR_STATE PACKING

#define MAVLINK_MSG_ID_QUADROTOR_STATE 18180


typedef struct __mavlink_quadrotor_state_t {
 float Px; /*<  Position x*/
 float Py; /*<  Position y*/
 float Pz; /*<  Position z*/
 float Qw; /*<  Quaternion w*/
 float Qx; /*<  Quaternion x*/
 float Qy; /*<  Quaternion y*/
 float Qz; /*<  Quaternion z*/
 float Vx; /*<  Velocity x*/
 float Vy; /*<  Velocity y*/
 float Vz; /*<  Velocity z*/
} mavlink_quadrotor_state_t;

#define MAVLINK_MSG_ID_QUADROTOR_STATE_LEN 40
#define MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN 40
#define MAVLINK_MSG_ID_18180_LEN 40
#define MAVLINK_MSG_ID_18180_MIN_LEN 40

#define MAVLINK_MSG_ID_QUADROTOR_STATE_CRC 243
#define MAVLINK_MSG_ID_18180_CRC 243



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_QUADROTOR_STATE { \
    18180, \
    "QUADROTOR_STATE", \
    10, \
    {  { "Px", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_quadrotor_state_t, Px) }, \
         { "Py", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_quadrotor_state_t, Py) }, \
         { "Pz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_quadrotor_state_t, Pz) }, \
         { "Qw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_quadrotor_state_t, Qw) }, \
         { "Qx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_quadrotor_state_t, Qx) }, \
         { "Qy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_quadrotor_state_t, Qy) }, \
         { "Qz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_quadrotor_state_t, Qz) }, \
         { "Vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_quadrotor_state_t, Vx) }, \
         { "Vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_quadrotor_state_t, Vy) }, \
         { "Vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_quadrotor_state_t, Vz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_QUADROTOR_STATE { \
    "QUADROTOR_STATE", \
    10, \
    {  { "Px", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_quadrotor_state_t, Px) }, \
         { "Py", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_quadrotor_state_t, Py) }, \
         { "Pz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_quadrotor_state_t, Pz) }, \
         { "Qw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_quadrotor_state_t, Qw) }, \
         { "Qx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_quadrotor_state_t, Qx) }, \
         { "Qy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_quadrotor_state_t, Qy) }, \
         { "Qz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_quadrotor_state_t, Qz) }, \
         { "Vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_quadrotor_state_t, Vx) }, \
         { "Vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_quadrotor_state_t, Vy) }, \
         { "Vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_quadrotor_state_t, Vz) }, \
         } \
}
#endif

/**
 * @brief Pack a quadrotor_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Px  Position x
 * @param Py  Position y
 * @param Pz  Position z
 * @param Qw  Quaternion w
 * @param Qx  Quaternion x
 * @param Qy  Quaternion y
 * @param Qz  Quaternion z
 * @param Vx  Velocity x
 * @param Vy  Velocity y
 * @param Vz  Velocity z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quadrotor_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float Px, float Py, float Pz, float Qw, float Qx, float Qy, float Qz, float Vx, float Vy, float Vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUADROTOR_STATE_LEN];
    _mav_put_float(buf, 0, Px);
    _mav_put_float(buf, 4, Py);
    _mav_put_float(buf, 8, Pz);
    _mav_put_float(buf, 12, Qw);
    _mav_put_float(buf, 16, Qx);
    _mav_put_float(buf, 20, Qy);
    _mav_put_float(buf, 24, Qz);
    _mav_put_float(buf, 28, Vx);
    _mav_put_float(buf, 32, Vy);
    _mav_put_float(buf, 36, Vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN);
#else
    mavlink_quadrotor_state_t packet;
    packet.Px = Px;
    packet.Py = Py;
    packet.Pz = Pz;
    packet.Qw = Qw;
    packet.Qx = Qx;
    packet.Qy = Qy;
    packet.Qz = Qz;
    packet.Vx = Vx;
    packet.Vy = Vy;
    packet.Vz = Vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QUADROTOR_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_CRC);
}

/**
 * @brief Pack a quadrotor_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Px  Position x
 * @param Py  Position y
 * @param Pz  Position z
 * @param Qw  Quaternion w
 * @param Qx  Quaternion x
 * @param Qy  Quaternion y
 * @param Qz  Quaternion z
 * @param Vx  Velocity x
 * @param Vy  Velocity y
 * @param Vz  Velocity z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quadrotor_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float Px,float Py,float Pz,float Qw,float Qx,float Qy,float Qz,float Vx,float Vy,float Vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUADROTOR_STATE_LEN];
    _mav_put_float(buf, 0, Px);
    _mav_put_float(buf, 4, Py);
    _mav_put_float(buf, 8, Pz);
    _mav_put_float(buf, 12, Qw);
    _mav_put_float(buf, 16, Qx);
    _mav_put_float(buf, 20, Qy);
    _mav_put_float(buf, 24, Qz);
    _mav_put_float(buf, 28, Vx);
    _mav_put_float(buf, 32, Vy);
    _mav_put_float(buf, 36, Vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN);
#else
    mavlink_quadrotor_state_t packet;
    packet.Px = Px;
    packet.Py = Py;
    packet.Pz = Pz;
    packet.Qw = Qw;
    packet.Qx = Qx;
    packet.Qy = Qy;
    packet.Qz = Qz;
    packet.Vx = Vx;
    packet.Vy = Vy;
    packet.Vz = Vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QUADROTOR_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_CRC);
}

/**
 * @brief Encode a quadrotor_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param quadrotor_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quadrotor_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_quadrotor_state_t* quadrotor_state)
{
    return mavlink_msg_quadrotor_state_pack(system_id, component_id, msg, quadrotor_state->Px, quadrotor_state->Py, quadrotor_state->Pz, quadrotor_state->Qw, quadrotor_state->Qx, quadrotor_state->Qy, quadrotor_state->Qz, quadrotor_state->Vx, quadrotor_state->Vy, quadrotor_state->Vz);
}

/**
 * @brief Encode a quadrotor_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param quadrotor_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quadrotor_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_quadrotor_state_t* quadrotor_state)
{
    return mavlink_msg_quadrotor_state_pack_chan(system_id, component_id, chan, msg, quadrotor_state->Px, quadrotor_state->Py, quadrotor_state->Pz, quadrotor_state->Qw, quadrotor_state->Qx, quadrotor_state->Qy, quadrotor_state->Qz, quadrotor_state->Vx, quadrotor_state->Vy, quadrotor_state->Vz);
}

/**
 * @brief Send a quadrotor_state message
 * @param chan MAVLink channel to send the message
 *
 * @param Px  Position x
 * @param Py  Position y
 * @param Pz  Position z
 * @param Qw  Quaternion w
 * @param Qx  Quaternion x
 * @param Qy  Quaternion y
 * @param Qz  Quaternion z
 * @param Vx  Velocity x
 * @param Vy  Velocity y
 * @param Vz  Velocity z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_quadrotor_state_send(mavlink_channel_t chan, float Px, float Py, float Pz, float Qw, float Qx, float Qy, float Qz, float Vx, float Vy, float Vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUADROTOR_STATE_LEN];
    _mav_put_float(buf, 0, Px);
    _mav_put_float(buf, 4, Py);
    _mav_put_float(buf, 8, Pz);
    _mav_put_float(buf, 12, Qw);
    _mav_put_float(buf, 16, Qx);
    _mav_put_float(buf, 20, Qy);
    _mav_put_float(buf, 24, Qz);
    _mav_put_float(buf, 28, Vx);
    _mav_put_float(buf, 32, Vy);
    _mav_put_float(buf, 36, Vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_STATE, buf, MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_CRC);
#else
    mavlink_quadrotor_state_t packet;
    packet.Px = Px;
    packet.Py = Py;
    packet.Pz = Pz;
    packet.Qw = Qw;
    packet.Qx = Qx;
    packet.Qy = Qy;
    packet.Qz = Qz;
    packet.Vx = Vx;
    packet.Vy = Vy;
    packet.Vz = Vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_STATE, (const char *)&packet, MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_CRC);
#endif
}

/**
 * @brief Send a quadrotor_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_quadrotor_state_send_struct(mavlink_channel_t chan, const mavlink_quadrotor_state_t* quadrotor_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_quadrotor_state_send(chan, quadrotor_state->Px, quadrotor_state->Py, quadrotor_state->Pz, quadrotor_state->Qw, quadrotor_state->Qx, quadrotor_state->Qy, quadrotor_state->Qz, quadrotor_state->Vx, quadrotor_state->Vy, quadrotor_state->Vz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_STATE, (const char *)quadrotor_state, MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_QUADROTOR_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_quadrotor_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float Px, float Py, float Pz, float Qw, float Qx, float Qy, float Qz, float Vx, float Vy, float Vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, Px);
    _mav_put_float(buf, 4, Py);
    _mav_put_float(buf, 8, Pz);
    _mav_put_float(buf, 12, Qw);
    _mav_put_float(buf, 16, Qx);
    _mav_put_float(buf, 20, Qy);
    _mav_put_float(buf, 24, Qz);
    _mav_put_float(buf, 28, Vx);
    _mav_put_float(buf, 32, Vy);
    _mav_put_float(buf, 36, Vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_STATE, buf, MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_CRC);
#else
    mavlink_quadrotor_state_t *packet = (mavlink_quadrotor_state_t *)msgbuf;
    packet->Px = Px;
    packet->Py = Py;
    packet->Pz = Pz;
    packet->Qw = Qw;
    packet->Qx = Qx;
    packet->Qy = Qy;
    packet->Qz = Qz;
    packet->Vx = Vx;
    packet->Vy = Vy;
    packet->Vz = Vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_STATE, (const char *)packet, MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN, MAVLINK_MSG_ID_QUADROTOR_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE QUADROTOR_STATE UNPACKING


/**
 * @brief Get field Px from quadrotor_state message
 *
 * @return  Position x
 */
static inline float mavlink_msg_quadrotor_state_get_Px(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Py from quadrotor_state message
 *
 * @return  Position y
 */
static inline float mavlink_msg_quadrotor_state_get_Py(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Pz from quadrotor_state message
 *
 * @return  Position z
 */
static inline float mavlink_msg_quadrotor_state_get_Pz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field Qw from quadrotor_state message
 *
 * @return  Quaternion w
 */
static inline float mavlink_msg_quadrotor_state_get_Qw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field Qx from quadrotor_state message
 *
 * @return  Quaternion x
 */
static inline float mavlink_msg_quadrotor_state_get_Qx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Qy from quadrotor_state message
 *
 * @return  Quaternion y
 */
static inline float mavlink_msg_quadrotor_state_get_Qy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field Qz from quadrotor_state message
 *
 * @return  Quaternion z
 */
static inline float mavlink_msg_quadrotor_state_get_Qz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field Vx from quadrotor_state message
 *
 * @return  Velocity x
 */
static inline float mavlink_msg_quadrotor_state_get_Vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field Vy from quadrotor_state message
 *
 * @return  Velocity y
 */
static inline float mavlink_msg_quadrotor_state_get_Vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field Vz from quadrotor_state message
 *
 * @return  Velocity z
 */
static inline float mavlink_msg_quadrotor_state_get_Vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a quadrotor_state message into a struct
 *
 * @param msg The message to decode
 * @param quadrotor_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_quadrotor_state_decode(const mavlink_message_t* msg, mavlink_quadrotor_state_t* quadrotor_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    quadrotor_state->Px = mavlink_msg_quadrotor_state_get_Px(msg);
    quadrotor_state->Py = mavlink_msg_quadrotor_state_get_Py(msg);
    quadrotor_state->Pz = mavlink_msg_quadrotor_state_get_Pz(msg);
    quadrotor_state->Qw = mavlink_msg_quadrotor_state_get_Qw(msg);
    quadrotor_state->Qx = mavlink_msg_quadrotor_state_get_Qx(msg);
    quadrotor_state->Qy = mavlink_msg_quadrotor_state_get_Qy(msg);
    quadrotor_state->Qz = mavlink_msg_quadrotor_state_get_Qz(msg);
    quadrotor_state->Vx = mavlink_msg_quadrotor_state_get_Vx(msg);
    quadrotor_state->Vy = mavlink_msg_quadrotor_state_get_Vy(msg);
    quadrotor_state->Vz = mavlink_msg_quadrotor_state_get_Vz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_QUADROTOR_STATE_LEN? msg->len : MAVLINK_MSG_ID_QUADROTOR_STATE_LEN;
        memset(quadrotor_state, 0, MAVLINK_MSG_ID_QUADROTOR_STATE_LEN);
    memcpy(quadrotor_state, _MAV_PAYLOAD(msg), len);
#endif
}
