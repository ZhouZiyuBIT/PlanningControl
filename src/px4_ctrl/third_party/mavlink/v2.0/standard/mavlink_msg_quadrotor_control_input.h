#pragma once
// MESSAGE QUADROTOR_CONTROL_INPUT PACKING

#define MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT 18181


typedef struct __mavlink_quadrotor_control_input_t {
 float thrust; /*<  Thrust*/
 float Wx; /*<  Angular velocity x*/
 float Wy; /*<  Angular velocity y*/
 float Wz; /*<  Angular velocity z*/
} mavlink_quadrotor_control_input_t;

#define MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN 16
#define MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN 16
#define MAVLINK_MSG_ID_18181_LEN 16
#define MAVLINK_MSG_ID_18181_MIN_LEN 16

#define MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_CRC 223
#define MAVLINK_MSG_ID_18181_CRC 223



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_QUADROTOR_CONTROL_INPUT { \
    18181, \
    "QUADROTOR_CONTROL_INPUT", \
    4, \
    {  { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_quadrotor_control_input_t, thrust) }, \
         { "Wx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_quadrotor_control_input_t, Wx) }, \
         { "Wy", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_quadrotor_control_input_t, Wy) }, \
         { "Wz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_quadrotor_control_input_t, Wz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_QUADROTOR_CONTROL_INPUT { \
    "QUADROTOR_CONTROL_INPUT", \
    4, \
    {  { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_quadrotor_control_input_t, thrust) }, \
         { "Wx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_quadrotor_control_input_t, Wx) }, \
         { "Wy", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_quadrotor_control_input_t, Wy) }, \
         { "Wz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_quadrotor_control_input_t, Wz) }, \
         } \
}
#endif

/**
 * @brief Pack a quadrotor_control_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param thrust  Thrust
 * @param Wx  Angular velocity x
 * @param Wy  Angular velocity y
 * @param Wz  Angular velocity z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quadrotor_control_input_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float thrust, float Wx, float Wy, float Wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN];
    _mav_put_float(buf, 0, thrust);
    _mav_put_float(buf, 4, Wx);
    _mav_put_float(buf, 8, Wy);
    _mav_put_float(buf, 12, Wz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN);
#else
    mavlink_quadrotor_control_input_t packet;
    packet.thrust = thrust;
    packet.Wx = Wx;
    packet.Wy = Wy;
    packet.Wz = Wz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_CRC);
}

/**
 * @brief Pack a quadrotor_control_input message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param thrust  Thrust
 * @param Wx  Angular velocity x
 * @param Wy  Angular velocity y
 * @param Wz  Angular velocity z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quadrotor_control_input_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float thrust,float Wx,float Wy,float Wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN];
    _mav_put_float(buf, 0, thrust);
    _mav_put_float(buf, 4, Wx);
    _mav_put_float(buf, 8, Wy);
    _mav_put_float(buf, 12, Wz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN);
#else
    mavlink_quadrotor_control_input_t packet;
    packet.thrust = thrust;
    packet.Wx = Wx;
    packet.Wy = Wy;
    packet.Wz = Wz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_CRC);
}

/**
 * @brief Encode a quadrotor_control_input struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param quadrotor_control_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quadrotor_control_input_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_quadrotor_control_input_t* quadrotor_control_input)
{
    return mavlink_msg_quadrotor_control_input_pack(system_id, component_id, msg, quadrotor_control_input->thrust, quadrotor_control_input->Wx, quadrotor_control_input->Wy, quadrotor_control_input->Wz);
}

/**
 * @brief Encode a quadrotor_control_input struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param quadrotor_control_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quadrotor_control_input_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_quadrotor_control_input_t* quadrotor_control_input)
{
    return mavlink_msg_quadrotor_control_input_pack_chan(system_id, component_id, chan, msg, quadrotor_control_input->thrust, quadrotor_control_input->Wx, quadrotor_control_input->Wy, quadrotor_control_input->Wz);
}

/**
 * @brief Send a quadrotor_control_input message
 * @param chan MAVLink channel to send the message
 *
 * @param thrust  Thrust
 * @param Wx  Angular velocity x
 * @param Wy  Angular velocity y
 * @param Wz  Angular velocity z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_quadrotor_control_input_send(mavlink_channel_t chan, float thrust, float Wx, float Wy, float Wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN];
    _mav_put_float(buf, 0, thrust);
    _mav_put_float(buf, 4, Wx);
    _mav_put_float(buf, 8, Wy);
    _mav_put_float(buf, 12, Wz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT, buf, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_CRC);
#else
    mavlink_quadrotor_control_input_t packet;
    packet.thrust = thrust;
    packet.Wx = Wx;
    packet.Wy = Wy;
    packet.Wz = Wz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT, (const char *)&packet, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_CRC);
#endif
}

/**
 * @brief Send a quadrotor_control_input message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_quadrotor_control_input_send_struct(mavlink_channel_t chan, const mavlink_quadrotor_control_input_t* quadrotor_control_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_quadrotor_control_input_send(chan, quadrotor_control_input->thrust, quadrotor_control_input->Wx, quadrotor_control_input->Wy, quadrotor_control_input->Wz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT, (const char *)quadrotor_control_input, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_quadrotor_control_input_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float thrust, float Wx, float Wy, float Wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, thrust);
    _mav_put_float(buf, 4, Wx);
    _mav_put_float(buf, 8, Wy);
    _mav_put_float(buf, 12, Wz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT, buf, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_CRC);
#else
    mavlink_quadrotor_control_input_t *packet = (mavlink_quadrotor_control_input_t *)msgbuf;
    packet->thrust = thrust;
    packet->Wx = Wx;
    packet->Wy = Wy;
    packet->Wz = Wz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT, (const char *)packet, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE QUADROTOR_CONTROL_INPUT UNPACKING


/**
 * @brief Get field thrust from quadrotor_control_input message
 *
 * @return  Thrust
 */
static inline float mavlink_msg_quadrotor_control_input_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Wx from quadrotor_control_input message
 *
 * @return  Angular velocity x
 */
static inline float mavlink_msg_quadrotor_control_input_get_Wx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Wy from quadrotor_control_input message
 *
 * @return  Angular velocity y
 */
static inline float mavlink_msg_quadrotor_control_input_get_Wy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field Wz from quadrotor_control_input message
 *
 * @return  Angular velocity z
 */
static inline float mavlink_msg_quadrotor_control_input_get_Wz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a quadrotor_control_input message into a struct
 *
 * @param msg The message to decode
 * @param quadrotor_control_input C-struct to decode the message contents into
 */
static inline void mavlink_msg_quadrotor_control_input_decode(const mavlink_message_t* msg, mavlink_quadrotor_control_input_t* quadrotor_control_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    quadrotor_control_input->thrust = mavlink_msg_quadrotor_control_input_get_thrust(msg);
    quadrotor_control_input->Wx = mavlink_msg_quadrotor_control_input_get_Wx(msg);
    quadrotor_control_input->Wy = mavlink_msg_quadrotor_control_input_get_Wy(msg);
    quadrotor_control_input->Wz = mavlink_msg_quadrotor_control_input_get_Wz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN? msg->len : MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN;
        memset(quadrotor_control_input, 0, MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_LEN);
    memcpy(quadrotor_control_input, _MAV_PAYLOAD(msg), len);
#endif
}
