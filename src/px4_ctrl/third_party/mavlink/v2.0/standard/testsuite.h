/** @file
 *    @brief MAVLink comm protocol testsuite generated from standard.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef STANDARD_TESTSUITE_H
#define STANDARD_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_standard(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_standard(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_quadrotor_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_QUADROTOR_STATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_quadrotor_state_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0
    };
    mavlink_quadrotor_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Px = packet_in.Px;
        packet1.Py = packet_in.Py;
        packet1.Pz = packet_in.Pz;
        packet1.Qw = packet_in.Qw;
        packet1.Qx = packet_in.Qx;
        packet1.Qy = packet_in.Qy;
        packet1.Qz = packet_in.Qz;
        packet1.Vx = packet_in.Vx;
        packet1.Vy = packet_in.Vy;
        packet1.Vz = packet_in.Vz;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_QUADROTOR_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_quadrotor_state_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_quadrotor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_quadrotor_state_pack(system_id, component_id, &msg , packet1.Px , packet1.Py , packet1.Pz , packet1.Qw , packet1.Qx , packet1.Qy , packet1.Qz , packet1.Vx , packet1.Vy , packet1.Vz );
    mavlink_msg_quadrotor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_quadrotor_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Px , packet1.Py , packet1.Pz , packet1.Qw , packet1.Qx , packet1.Qy , packet1.Qz , packet1.Vx , packet1.Vy , packet1.Vz );
    mavlink_msg_quadrotor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_quadrotor_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_quadrotor_state_send(MAVLINK_COMM_1 , packet1.Px , packet1.Py , packet1.Pz , packet1.Qw , packet1.Qx , packet1.Qy , packet1.Qz , packet1.Vx , packet1.Vy , packet1.Vz );
    mavlink_msg_quadrotor_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_quadrotor_control_input(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_quadrotor_control_input_t packet_in = {
        17.0,45.0,73.0,101.0
    };
    mavlink_quadrotor_control_input_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.thrust = packet_in.thrust;
        packet1.Wx = packet_in.Wx;
        packet1.Wy = packet_in.Wy;
        packet1.Wz = packet_in.Wz;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_QUADROTOR_CONTROL_INPUT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_quadrotor_control_input_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_quadrotor_control_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_quadrotor_control_input_pack(system_id, component_id, &msg , packet1.thrust , packet1.Wx , packet1.Wy , packet1.Wz );
    mavlink_msg_quadrotor_control_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_quadrotor_control_input_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.thrust , packet1.Wx , packet1.Wy , packet1.Wz );
    mavlink_msg_quadrotor_control_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_quadrotor_control_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_quadrotor_control_input_send(MAVLINK_COMM_1 , packet1.thrust , packet1.Wx , packet1.Wy , packet1.Wz );
    mavlink_msg_quadrotor_control_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sensor_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSOR_IMU >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensor_imu_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_sensor_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.wx = packet_in.wx;
        packet1.wy = packet_in.wy;
        packet1.wz = packet_in.wz;
        packet1.ax = packet_in.ax;
        packet1.ay = packet_in.ay;
        packet1.az = packet_in.az;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSOR_IMU_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensor_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_imu_pack(system_id, component_id, &msg , packet1.wx , packet1.wy , packet1.wz , packet1.ax , packet1.ay , packet1.az );
    mavlink_msg_sensor_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.wx , packet1.wy , packet1.wz , packet1.ax , packet1.ay , packet1.az );
    mavlink_msg_sensor_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensor_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_imu_send(MAVLINK_COMM_1 , packet1.wx , packet1.wy , packet1.wz , packet1.ax , packet1.ay , packet1.az );
    mavlink_msg_sensor_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_standard(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_quadrotor_state(system_id, component_id, last_msg);
    mavlink_test_quadrotor_control_input(system_id, component_id, last_msg);
    mavlink_test_sensor_imu(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // STANDARD_TESTSUITE_H
